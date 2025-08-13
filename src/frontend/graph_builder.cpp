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
#include <kimera_pgmo/compression/block_compression.h>
#include <kimera_pgmo/compression/delta_compression.h>
#include <kimera_pgmo/utils/common_functions.h>
#include <kimera_pgmo/utils/mesh_io.h>
#include <spark_dsg/printing.h>

#include "hydra/common/global_info.h"
#include "hydra/common/launch_callbacks.h"
#include "hydra/common/pipeline_queues.h"
#include "hydra/frontend/frontier_extractor.h"
#include "hydra/frontend/mesh_segmenter.h"
#include "hydra/utils/pgmo_mesh_interface.h"
#include "hydra/utils/pgmo_mesh_traits.h"  // IWYU pragma: keep
#include "hydra/utils/printing.h"
#include "hydra/utils/timing_utilities.h"

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
  field(config.lcd_use_bow_vectors, "lcd_use_bow_vectors");

  {
    NameSpace ns("pgmo");
    field(config.pgmo.mesh_resolution, "mesh_resolution");
    field(config.pgmo.d_graph_resolution, "d_graph_resolution");
    field(config.pgmo.time_horizon, "time_horizon");
  }

  field(config.graph_connector, "graph_connector");
  field(config.graph_updater, "graph_updater");
  field(config.enable_mesh_objects, "enable_mesh_objects");
  field(config.object_config, "objects");
  config.pose_graph_tracker.setOptional();
  field(config.pose_graph_tracker, "pose_graph_tracker");
  // surface (i.e., 2D) places
  config.surface_places.setOptional();
  field(config.surface_places, "surface_places");
  // freespace (i.e., 3D) places
  config.freespace_places.setOptional();
  field(config.freespace_places, "freespace_places");
  // frontier (i.e. 3D, boundary to unknown space) places
  field(config.use_frontiers, "use_frontiers");
  config.frontier_places.setOptional();
  field(config.frontier_places, "frontier_places");
  field(config.view_database, "view_database");
  field(config.sinks, "sinks");
  field(config.no_packet_collation, "no_packet_collation");
  field(config.verbosity, "verbosity");
}

GraphBuilder::GraphBuilder(const Config& config,
                           const SharedDsgInfo::Ptr& dsg,
                           const SharedModuleState::Ptr& state)
    : config(config::checkValid(config)),
      sequence_number_(1),  // starts at 1 to differentiate from SharedDsgInfo default
      queue_(std::make_shared<InputQueue>()),
      dsg_(dsg),
      state_(state),
      graph_updater_(config.graph_updater),
      graph_connector_(config.graph_connector),
      map_window_(GlobalInfo::instance().createVolumetricWindow()),
      tracker_(config.pose_graph_tracker.create()),
      surface_places_(config.surface_places.create()),
      freespace_places_(config.freespace_places.create()),
      frontier_places_(config.frontier_places.create()),
      view_database_(config.view_database),
      sinks_(Sink::instantiate(config.sinks)) {
  const auto& global_info = GlobalInfo::instance();
  if (config.enable_mesh_objects) {
    segmenter_ = std::make_unique<MeshSegmenter>(
        config.object_config, global_info.getLabelSpaceConfig().object_labels);
  }

  if (!config.use_frontiers) {
    frontier_places_.reset();
  }

  CHECK(dsg_ != nullptr);
  CHECK(dsg_->graph != nullptr);
  dsg_->graph->setMesh(global_info.createMesh());

  mesh_compression_.reset(
      new kimera_pgmo::DeltaCompression(config.pgmo.mesh_resolution));
  deformation_compression_.reset(
      new kimera_pgmo::BlockCompression(config.pgmo.d_graph_resolution));

  addInputCallback(std::bind(&GraphBuilder::updateMesh, this, std::placeholders::_1));
  addInputCallback(
      std::bind(&GraphBuilder::updateDeformationGraph, this, std::placeholders::_1));
  addInputCallback(
      std::bind(&GraphBuilder::updatePoseGraph, this, std::placeholders::_1));
  addInputCallback(std::bind(&GraphBuilder::updatePlaces, this, std::placeholders::_1));
  addInputCallback(
      std::bind(&GraphBuilder::updateFrontiers, this, std::placeholders::_1));

  addPostMeshCallback(
      std::bind(&GraphBuilder::updateObjects, this, std::placeholders::_1));
  addPostMeshCallback(
      std::bind(&GraphBuilder::updatePlaces2d, this, std::placeholders::_1));

  if (config.lcd_use_bow_vectors) {
    PipelineQueues::instance().bow_queue.reset(new PipelineQueues::BowQueue());
  }
}

GraphBuilder::~GraphBuilder() {
  // intentionally the private implementation to avoid calling virtual method
  stopImpl();
}

void GraphBuilder::start() {
  spin_thread_.reset(new std::thread(&GraphBuilder::spin, this));
  LOG(INFO) << "[Hydra Frontend] started!";
}

void GraphBuilder::stop() { stopImpl(); }

void GraphBuilder::stopImpl() {
  should_shutdown_ = true;

  if (spin_thread_) {
    VLOG(2) << "[Hydra Frontend] stopping frontend!";
    spin_thread_->join();
    spin_thread_.reset();
    VLOG(2) << "[Hydra Frontend] stopped!";
  }

  VLOG(2) << "[Hydra Frontend]: " << queue_->size() << " messages left";
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
  bool should_shutdown = false;
  spin_finished_ = true;

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
      using namespace std::chrono_literals;
      std::this_thread::sleep_for(1ms);
      continue;
    }

    processNextInput(*queue_->front());

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
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

void GraphBuilder::processNextInput(const ActiveWindowOutput& msg) {
  if (tracker_) {
    const auto packet = tracker_->update(msg.timestamp_ns, msg.world_T_body());
    pose_graph_updates_.push(packet);
  } else {
    LOG_FIRST_N(WARNING, 1)
        << "PoseGraphTracker disabled, no agent layer will be created";
    return;
  }

  if (!msg.sensor_data) {
    return;
  }

  const auto& data = *msg.sensor_data;
  if (data.feature.rows() * data.feature.cols() == 0) {
    return;  // no feature present
  }

  auto view = std::make_unique<FeatureView>(data.timestamp_ns,
                                            data.getSensorPose().inverse(),
                                            data.feature,
                                            &data.getSensor());
  PipelineQueues::instance().input_features_queue.push(std::move(view));
}

bool GraphBuilder::spinOnce() {
  bool has_data = queue_->poll();
  if (!has_data) {
    return false;
  }

  ActiveWindowOutput::Ptr input = queue_->front();
  processNextInput(*input);
  queue_->pop();

  spinOnce(input);
  return true;
}

void GraphBuilder::addSink(const Sink::Ptr& sink) {
  if (sink) {
    sinks_.push_back(sink);
  }
}

void GraphBuilder::addInputCallback(InputCallback callback) {
  input_callbacks_.push_back([callback](ActiveWindowOutput::Ptr msg) {
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
  auto& queues = PipelineQueues::instance();

  VLOG(5) << "[Hydra Frontend] Popped input packet @ " << msg->timestamp_ns << " [ns]";
  std::lock_guard<std::mutex> lock(mutex_);
  ScopedTimer timer("frontend/spin", msg->timestamp_ns);

  backend_input_.reset(new BackendInput());
  backend_input_->timestamp_ns = msg->timestamp_ns;
  backend_input_->sequence_number = sequence_number_;
  if (queues.lcd_queue) {
    lcd_input_.reset(new LcdInput());
    lcd_input_->timestamp_ns = msg->timestamp_ns;
    lcd_input_->sequence_number = sequence_number_;
  }

  updateImpl(msg);

  // TODO(nathan) ideally make the copy lighter-weight
  // we need to copy over the latest updates to the backend and to LCD
  // no fancy threading: we just mark the update time and copy all changes in one go
  {  // start critical section
    std::unique_lock<std::mutex> lock(state_->backend_graph->mutex);
    ScopedTimer merge_timer("frontend/merge_graph", msg->timestamp_ns);
    state_->backend_graph->sequence_number = sequence_number_;
    state_->backend_graph->graph->mergeGraph(*dsg_->graph);
  }  // end critical section

  if (queues.lcd_queue) {
    // n.b., critical section in this scope!
    std::unique_lock<std::mutex> lock(state_->lcd_graph->mutex);
    ScopedTimer merge_timer("frontend/merge_lcd_graph", msg->timestamp_ns);
    state_->lcd_graph->sequence_number = sequence_number_;
    state_->lcd_graph->graph->mergeGraph(*dsg_->graph);
  }

  backend_input_->mesh_update = std::move(last_mesh_update_);
  queues.backend_queue.push(backend_input_);
  if (queues.lcd_queue) {
    queues.lcd_queue->push(lcd_input_);
  }

  // mutex not required because nothing is modifying the graph
  frontend_graph_logger_.logGraph(*dsg_->graph);

  if (dsg_->graph && backend_input_) {
    ScopedTimer sink_timer("frontend/sinks", msg->timestamp_ns);
    Sink::callAll(sinks_, msg->timestamp_ns, *dsg_->graph, *backend_input_);
  }

  ++sequence_number_;
}

void GraphBuilder::updateImpl(const ActiveWindowOutput::Ptr& msg) {
  graph_updater_.update(msg->graph_update, *dsg_->graph);

  {  // start timing scope
    ScopedTimer timer("frontend/launch_callbacks", msg->timestamp_ns, true, 1, false);
    launchCallbacks(input_callbacks_, msg);
  }

  {  // start timing scope
    ScopedTimer timer("frontend/interlayer_edges", msg->timestamp_ns, true, 1, false);
    graph_connector_.connect(*dsg_->graph);
  }

  view_database_.updateAssignments(
      *dsg_->graph, [&](const Eigen::Vector3d& pos, uint64_t timestamp) {
        if (!map_window_) {
          return false;
        }

        const auto fmt = getDefaultFormat(3);
        LOG_IF(INFO, config.verbosity >= 2)
            << "view @ " << timestamp << "[ns]: " << pos.format(fmt) << " vs. "
            << msg->world_T_body().translation().format(fmt);
        return !map_window_->inBounds(
            msg->timestamp_ns, msg->world_T_body(), timestamp, pos);
      });

  // TODO(nathan) follow up on whether or not we need to do stuff with the 3D places and
  // mesh
}

void GraphBuilder::updateMesh(const ActiveWindowOutput& input) {
  {  // start timing scope
    ScopedTimer timer("frontend/mesh_archive", input.timestamp_ns, true, 1, false);
    // TODO(nathan) add this back when we fix the khronos active window
    // const auto pose = input.world_T_body();
    // const auto block_size = input.map().blockSize();
    const spatial_hash::IndexSet archived_blocks(input.archived_mesh_indices.begin(),
                                                 input.archived_mesh_indices.end());
    mesh_compression_->archiveBlocks([&](const auto& index, const auto& /* info */) {
      return archived_blocks.count(index);
    });
  }  // end timing scope

  const auto& mesh = input.map().getMeshLayer();

  {
    ScopedTimer timer("frontend/mesh_compression", input.timestamp_ns, true, 1, false);
    VLOG(5) << "[Hydra Frontend] Updating mesh with " << mesh.numBlocks() << " blocks";
    const BlockMeshIter wrapper(mesh);
    last_mesh_update_ = mesh_compression_->update(wrapper, input.timestamp_ns);
  }  // end timing scope

  {  // start timing scope
    // TODO(nathan) we should probably have a mutex before modifying the mesh, but
    // nothing else uses it at the moment
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

void GraphBuilder::updateDeformationGraph(const ActiveWindowOutput& input) {
  ScopedTimer timer("frontend/dgraph_compresssion", input.timestamp_ns, true, 1, false);
  const auto& prefix = GlobalInfo::instance().getRobotPrefix();
  const auto time_ns = std::chrono::nanoseconds(input.timestamp_ns);
  double time_s =
      std::chrono::duration_cast<std::chrono::duration<double>>(time_ns).count();
  auto interface = PgmoMeshLayerInterface(input.map().getMeshLayer());

  pcl::PointCloud<pcl::PointXYZRGBA> new_vertices;
  std::vector<size_t> new_indices;
  std::vector<pcl::Vertices> new_triangles;
  kimera_pgmo::HashedIndexMapping new_remapping;
  deformation_compression_->pruneStoredMesh(time_s - config.pgmo.time_horizon);
  deformation_compression_->compressAndIntegrate(
      interface, new_vertices, new_triangles, new_indices, new_remapping, time_s);

  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr vertices(
      new pcl::PointCloud<pcl::PointXYZRGBA>());
  deformation_compression_->getVertices(vertices);

  // Add nodes and edges to graph
  const auto new_edges =
      deformation_graph_.addPointsAndSurfaces(new_indices, new_triangles);
  if (backend_input_) {
    backend_input_->deformation_graph = *CHECK_NOTNULL(kimera_pgmo::makePoseGraph(
        prefix.id, time_s, new_edges, new_indices, *vertices));
  }
}

void GraphBuilder::updateFrontiers(const ActiveWindowOutput& input) {
  if (!frontier_places_) {
    return;
  }

  const auto timestamp = input.timestamp_ns;
  frontier_places_->updateRecentBlocks(input.world_t_body, input.map().blockSize());

  {  // start graph critical section
    std::unique_lock<std::mutex> graph_lock(dsg_->mutex);

    ScopedTimer timer("frontend/frontiers", timestamp, true, 1, false);
    NodeIdSet active_nodes = freespace_places_->getActiveNodes();
    frontier_places_->detectFrontiers(input, *dsg_->graph, active_nodes);
    frontier_places_->addFrontiers(timestamp, *dsg_->graph);
  }  // end graph update critical section
}

void GraphBuilder::updatePlaces(const ActiveWindowOutput& input) {
  if (!freespace_places_) {
    return;
  }

  NodeIdSet active_nodes;
  freespace_places_->detect(input);
  {  // start graph critical section
    std::unique_lock<std::mutex> graph_lock(dsg_->mutex);
    freespace_places_->updateGraph(input.timestamp_ns, *dsg_->graph);

    // force active flag on places
    active_nodes = freespace_places_->getActiveNodes();
    std::vector<NodeId> archived_places;
    for (const auto& [node_id, node] :
         dsg_->graph->getLayer(DsgLayers::PLACES).nodes()) {
      auto& attrs = node->attributes();
      const auto prev_active = attrs.is_active;
      attrs.is_active = active_nodes.count(node_id);
      if (prev_active && !attrs.is_active) {
        archived_places.push_back(node_id);
      }
    }

    if (lcd_input_) {
      lcd_input_->archived_places.insert(archived_places.begin(),
                                         archived_places.end());
    }

    if (frontier_places_) {
      frontier_places_->setArchivedPlaces(archived_places);
    }
  }  // end graph update critical section
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
  surface_places_->detect(input, *last_mesh_update_, mesh_offsets_, *dsg_->graph);

  // start graph critical section
  std::unique_lock<std::mutex> graph_lock(dsg_->mutex);
  surface_places_->updateGraph(input.timestamp_ns, input, mesh_offsets_, *dsg_->graph);
}

void GraphBuilder::updatePoseGraph(const ActiveWindowOutput& input) {
  ScopedTimer timer("frontend/update_posegraph", input.timestamp_ns);

  PoseGraphPacket packet;
  while (!pose_graph_updates_.empty()) {
    packet.updateFrom(pose_graph_updates_.pop());
  }

  if (backend_input_) {
    backend_input_->agent_updates = packet;
  }

  const auto& prefix = GlobalInfo::instance().getRobotPrefix();
  // TODO(nathan) thinking about locking more
  std::lock_guard<std::mutex> lock(dsg_->mutex);
  const auto new_node_ids = packet.addToGraph(*dsg_->graph, prefix.id);
  if (lcd_input_) {
    lcd_input_->new_agent_nodes = new_node_ids;
  }

  assignBowVectors();
}

void GraphBuilder::assignBowVectors() {
  auto& queue = PipelineQueues::instance().bow_queue;
  if (!queue) {
    return;
  }

  const auto& prefix = GlobalInfo::instance().getRobotPrefix();
  const auto layer_id = dsg_->graph->getLayerKey(DsgLayers::AGENTS)->layer;
  const auto agents = dsg_->graph->findLayer(layer_id, prefix.key);
  if (!agents) {
    // we have no nodes added to the pose graph yet, so don't do work
    return;
  }

  // TODO(nathan) take care of synchronization better
  // lcd_input_->new_agent_nodes.clear();

  // add bow messages received since last spin
  while (!queue->empty()) {
    cached_bow_messages_.push_back(queue->pop());
  }

  const auto prior_size = cached_bow_messages_.size();

  auto iter = cached_bow_messages_.begin();
  while (iter != cached_bow_messages_.end()) {
    const auto& msg = *iter;
    if (static_cast<int>(msg->robot_id) != prefix.id) {
      VLOG(1) << "[Hydra Frontend] rejected bow message from robot " << msg->robot_id;
      iter = cached_bow_messages_.erase(iter);
    }

    const NodeSymbol node_id(prefix.key, msg->pose_id);
    const auto node = agents->findNode(node_id);
    if (!node) {
      ++iter;
      continue;
    }

    auto& attrs = node->attributes<AgentNodeAttributes>();
    attrs.dbow_ids = Eigen::Map<const AgentNodeAttributes::BowIdVector>(
        msg->bow_vector.word_ids.data(), msg->bow_vector.word_ids.size());
    attrs.dbow_values = Eigen::Map<const Eigen::VectorXf>(
        msg->bow_vector.word_values.data(), msg->bow_vector.word_values.size());
    VLOG(5) << "[Hydra Frontend] assigned bow vector for " << node_id.str();

    iter = cached_bow_messages_.erase(iter);
  }

  size_t num_assigned = prior_size - cached_bow_messages_.size();
  VLOG(3) << "[Hydra Frontend] assigned " << num_assigned << " bow vectors of "
          << prior_size << " original";
}

}  // namespace hydra
