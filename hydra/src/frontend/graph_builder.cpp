/* -----------------------------------------------------------------------------
 * Copyright 2022 Massachusetts Institute of Technology.
 * All Rights Reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  1. Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *
 *  2. Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Research was sponsored by the United States Air Force Research Laboratory and
 * the United States Air Force Artificial Intelligence Accelerator and was
 * accomplished under Cooperative Agreement Number FA8750-19-2-1000. The views
 * and conclusions contained in this document are those of the authors and should
 * not be interpreted as representing the official policies, either expressed or
 * implied, of the United States Air Force or the U.S. Government. The U.S.
 * Government is authorized to reproduce and distribute reprints for Government
 * purposes notwithstanding any copyright notation herein.
 * -------------------------------------------------------------------------- */
#include "hydra/frontend/graph_builder.h"

#include <config_utilities/config.h>
#include <config_utilities/printing.h>
#include <config_utilities/validation.h>
#include <glog/logging.h>
#include <kimera_pgmo/utils/mesh_io.h>
#include <spark_dsg/node_attributes.h>
#include <spark_dsg/printing.h>

#include "hydra/common/global_info.h"
#include "hydra/common/launch_callbacks.h"
#include "hydra/common/pipeline_queues.h"
#include "hydra/frontend/deformation_graph_builder.h"
#include "hydra/frontend/keyframe_selector.h"
#include "hydra/frontend/mesh_compression.h"
#include "hydra/frontend/mesh_segmenter.h"
#include "hydra/utils/pgmo_mesh_traits.h"  // IWYU pragma: keep
#include "hydra/utils/timing_utilities.h"

using namespace spark_dsg;

namespace hydra {
namespace {

static const auto registration =
    config::RegistrationWithConfig<GraphBuilder,
                                   GraphBuilder,
                                   GraphBuilder::Config,
                                   SharedDsgInfo::Ptr,
                                   SharedModuleState::Ptr>("GraphBuilder");

}

using hydra::timing::ScopedTimer;

void declare_config(GraphBuilder::Config& config) {
  using namespace config;
  name("GraphBuilder::Config");
  base<VerbosityConfig>(config);

  field(config.no_packet_collation, "no_packet_collation");
  field(config.clear_object_meshes, "clear_object_meshes");
  field(config.enable_mesh_objects, "enable_mesh_objects");
  field(config.mesh_compression, "mesh_compression");

  field(config.graph_updater, "graph_updater");
  field(config.graph_connector, "graph_connector");

  field(config.object_config, "objects");
  config.surface_places.setOptional();
  field(config.surface_places, "surface_places");

  config.keyframe_selector.setOptional();
  field(config.keyframe_selector, "keyframe_selector");
  config.deformation_graph_builder.setOptional();
  field(config.deformation_graph_builder, "deformation_graph_builder");
  config.freespace_places.setOptional();
  field(config.freespace_places, "freespace_places");
  config.traversability_places.setOptional();
  field(config.traversability_places, "traversability_places");
  config.frontier_places.setOptional();
  field(config.frontier_places, "frontier_places");

  field(config.sinks, "sinks");
}

GraphBuilder::Config::Config()
    : VerbosityConfig(VerbosityConfig::default_verbosity("graph_builder")),
      mesh_compression(MeshCompression::Config{0.005}),
      graph_updater({{DsgLayers::OBJECTS, {'O', std::nullopt, {}, {}}}}),
      keyframe_selector(KeyframeSelector::Config()),
      deformation_graph_builder(DeformationGraphBuilder::Config()) {}

GraphBuilder::GraphBuilder(const Config& config,
                           const SharedDsgInfo::Ptr& dsg,
                           const SharedModuleState::Ptr& state)
    : config(config::checkValid(config)),
      queue_(std::make_shared<InputQueue>()),
      sequence_number_(1),  // starts at 1 to differentiate from SharedDsgInfo default
      dsg_(dsg),
      state_(state),
      mesh_compression_(config.mesh_compression.create()),
      graph_updater_(config.graph_updater),
      graph_connector_(config.graph_connector),
      map_window_(GlobalInfo::instance().createVolumetricWindow()),
      surface_places_(config.surface_places.create(
          GlobalInfo::instance().labelspace().surface_places_labels)),
      sinks_(Sink::instantiate(config.sinks)) {
  const auto& global_info = GlobalInfo::instance();
  if (config.enable_mesh_objects) {
    segmenter_ = std::make_unique<MeshSegmenter>(
        config.object_config, global_info.labelspace().object_labels);
  }

  CHECK(dsg_ != nullptr);
  CHECK(dsg_->graph != nullptr);
  dsg_->graph->setMesh(global_info.createMesh());

  addInputCallback(std::bind(&GraphBuilder::updateMesh, this, std::placeholders::_1));

  // TODO(nathan) this needs to be pushed to an actual config at some point
  functors_.emplace("keyframe_selector", config.keyframe_selector.create());
  functors_.emplace("deformation_graph_builder",
                    config.deformation_graph_builder.create());
  functors_.emplace("freespace_places", config.freespace_places.create());
  functors_.emplace("traversability_places", config.traversability_places.create());
  functors_.emplace("frontier_places", config.frontier_places.create());
  for (const auto& [_, functor] : functors_) {
    callbacks_.push_back([&](auto msg) {
      if (msg && functor) {
        functor->call(*msg, *dsg_, *curr_output_, map_window_.get());
      }
    });
  }

  addPostMeshCallback(
      std::bind(&GraphBuilder::updateObjects, this, std::placeholders::_1));
  addPostMeshCallback(
      std::bind(&GraphBuilder::updatePlaces2d, this, std::placeholders::_1));
}

GraphBuilder::~GraphBuilder() {
  // intentionally the private implementation to avoid calling virtual method
  stopImpl();
}

void GraphBuilder::start() {
  spin_thread_.reset(new std::thread(&GraphBuilder::spin, this));
  MLOG(0) << "started!";
}

void GraphBuilder::stop() { stopImpl(); }

void GraphBuilder::stopImpl() {
  should_shutdown_ = true;

  if (spin_thread_) {
    MLOG(1) << "stopping frontend!";
    spin_thread_->join();
    spin_thread_.reset();
    MLOG(1) << "stopped!";
    MLOG(1) << queue_->size() << " messages left";
  }
}

void GraphBuilder::save(const DataDirectory& output) {
  std::lock_guard<std::mutex> lock(mutex_);
  const auto output_path = output.path("frontend");

  dsg_->graph->save(output_path / "dsg.json", false);
  dsg_->graph->save(output_path / "dsg_with_mesh.json");
  frontend_graph_logger_.save(output_path);

  const auto mesh = dsg_->graph->mesh();
  if (mesh && !mesh->empty()) {
    kimera_pgmo::WriteMesh(output_path / "mesh.ply", *mesh);
  }
}

std::string GraphBuilder::printInfo() const {
  return config::toString(config) + "\n" + Sink::printSinks(sinks_);
}

void GraphBuilder::spin() {
  using namespace std::chrono_literals;

  spin_finished_ = true;
  bool should_shutdown = false;
  ActiveWindowOutput::Ptr input;
  while (!should_shutdown) {
    if (input && spin_finished_) {
      // start a spin to process input independent of this thread of
      // execution. spin_finished_ will flip to true once the thread terminates
      spin_finished_ = false;
      std::thread spin_thread(&GraphBuilder::dispatchSpin, this, input);
      spin_thread.detach();
      input.reset();
    }

    bool has_data = queue_->poll();
    if (GlobalInfo::instance().force_shutdown() || !has_data) {
      // copy over shutdown request
      should_shutdown = should_shutdown_;
    }

    if (!has_data) {
      continue;
    }

    if (!spin_finished_ && config.no_packet_collation) {
      std::this_thread::sleep_for(1ms);
      continue;
    }

    // from this point on, we build an input packet by collating the maps together of
    // subsequent outputs. This doesn't take effect until multiple packets from the
    // reconstruction module start arriving between frontend updates
    if (!input) {
      input = queue_->front();
    } else {
      // any usage of front after this is invalid
      input->updateFrom(std::move(*queue_->front()), false);
    }

    queue_->pop();
  }

  while (!spin_finished_) {
    // wait for current spin to finish before shutting down
    std::this_thread::sleep_for(10ms);
  }
}

bool GraphBuilder::spinOnce() {
  bool has_data = queue_->poll();
  if (!has_data) {
    return false;
  }

  auto input = queue_->pop();
  spinOnce(input);
  return true;
}

void GraphBuilder::addSink(const Sink::Ptr& sink) {
  if (sink) {
    sinks_.push_back(sink);
  }
}

void GraphBuilder::setLcdQueue(const OutputQueue::Ptr& queue) {
  lcd_input_queue_ = queue;
}

void GraphBuilder::addInputCallback(InputCallback callback) {
  callbacks_.push_back([callback](ActiveWindowOutput::Ptr msg) {
    if (!msg) {
      return;
    }

    callback(*msg);
  });
}

void GraphBuilder::addPostMeshCallback(InputCallback callback) {
  post_mesh_callbacks_.push_back(callback);
}

void GraphBuilder::dispatchSpin(ActiveWindowOutput::Ptr msg) {
  spinOnce(msg);
  spin_finished_ = true;
}

void GraphBuilder::spinOnce(const ActiveWindowOutput::Ptr& msg) {
  MLOG(2) << "Popped input packet @ " << msg->timestamp_ns << " [ns]";
  std::lock_guard<std::mutex> lock(mutex_);
  ScopedTimer timer("frontend/spin", msg->timestamp_ns);

  curr_output_ = std::make_shared<FrontendOutput>(msg->timestamp_ns, sequence_number_);
  updateImpl(msg);
  curr_output_->mesh_update = std::move(last_mesh_update_);

  // TODO(nathan) ideally make the copy lighter-weight
  // we need to copy over the latest updates to the backend and to LCD
  {  // start critical section
    std::unique_lock<std::mutex> lock(state_->backend_graph->mutex);
    ScopedTimer merge_timer("frontend/merge_graph", msg->timestamp_ns);
    state_->backend_graph->sequence_number = sequence_number_;
    state_->backend_graph->graph->mergeGraph(*dsg_->graph);
  }  // end critical section

  if (lcd_input_queue_) {  // LCD graph critical section
    std::unique_lock<std::mutex> lock(state_->lcd_graph->mutex);
    ScopedTimer merge_timer("frontend/merge_lcd_graph", msg->timestamp_ns);
    state_->lcd_graph->sequence_number = sequence_number_;
    state_->lcd_graph->graph->mergeGraph(*dsg_->graph);
  }

  PipelineQueues::instance().backend_queue.push(curr_output_);
  if (lcd_input_queue_) {
    lcd_input_queue_->push(curr_output_);
  }

  // mutex not required because nothing is modifying the graph
  frontend_graph_logger_.logGraph(*dsg_->graph);

  if (dsg_->graph && curr_output_) {
    ScopedTimer sink_timer("frontend/sinks", msg->timestamp_ns);
    Sink::callAll(sinks_, msg->timestamp_ns, *dsg_->graph, *curr_output_);
  }

  ++sequence_number_;
}

void GraphBuilder::updateImpl(const ActiveWindowOutput::Ptr& msg) {
  // TODO(nathan) remove this temporary patch once we fix serialization/mesh storage
  if (config.clear_object_meshes) {
    auto iter = msg->graph_update.find(2);
    if (iter != msg->graph_update.end()) {
      for (auto& node_update : iter->second->updates) {
        if (!node_update.attributes) {
          continue;
        }
        auto derived =
            dynamic_cast<KhronosObjectAttributes*>(node_update.attributes.get());
        if (derived) {
          derived->mesh.clear();
        }
      }
    }
  }

  graph_updater_.update(msg->graph_update, *dsg_->graph);

  {  // start timing scope
    ScopedTimer timer("frontend/launch_callbacks", msg->timestamp_ns, true, 1, false);
    launchCallbacks(callbacks_, msg);
  }

  {  // start timing scope
    ScopedTimer timer("frontend/interlayer_edges", msg->timestamp_ns, true, 1, false);
    graph_connector_.connect(*dsg_->graph);
  }

  for (const auto& [name, functor] : functors_) {
    if (functor) {
      functor->callPostUpdate(*dsg_, *curr_output_);
    }
  }
}

void GraphBuilder::updateMesh(const ActiveWindowOutput& input) {
  {
    ScopedTimer timer("frontend/mesh_compression", input.timestamp_ns, true, 1, false);
    last_mesh_update_ = mesh_compression_->update(input, map_window_.get());
  }

  {  // start timing scope
    ScopedTimer timer("frontend/mesh_update", input.timestamp_ns, true, 1, false);
    last_mesh_update_->updateMesh(*dsg_->graph->mesh(), mesh_offsets_);
  }  // end timing scope

  ScopedTimer timer("frontend/postmesh_callbacks", input.timestamp_ns, true, 1, false);
  launchCallbacks(post_mesh_callbacks_, input);
}

void GraphBuilder::updateObjects(const ActiveWindowOutput& input) {
  if (!segmenter_) {
    return;
  }

  if (!last_mesh_update_) {
    LOG(ERROR) << "Cannot detect objects without valid mesh";
    return;
  }

  const auto stamp = input.timestamp_ns;
  const auto clusters = segmenter_->detect(stamp, *last_mesh_update_, mesh_offsets_);
  {  // start dsg critical section
    std::unique_lock<std::mutex> lock(dsg_->mutex);
    segmenter_->updateGraph(stamp, mesh_offsets_, clusters, *dsg_->graph);
  }  // end dsg critical section
}

void GraphBuilder::updatePlaces2d(const ActiveWindowOutput& input) {
  if (!surface_places_) {
    return;
  }

  if (!last_mesh_update_) {
    LOG(ERROR) << "Cannot detect places without valid mesh";
    return;
  }

  ScopedTimer timer("frontend/places_2d", input.timestamp_ns, true, 1, false);
  surface_places_->detect(input, *last_mesh_update_, mesh_offsets_);

  // start graph critical section
  std::unique_lock<std::mutex> graph_lock(dsg_->mutex);
  surface_places_->updateGraph(input, mesh_offsets_, *dsg_->graph);
}

}  // namespace hydra
