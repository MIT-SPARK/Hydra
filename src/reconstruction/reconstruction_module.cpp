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
#include "hydra/reconstruction/reconstruction_module.h"

#include <config_utilities/printing.h>
#include <config_utilities/validation.h>

#include "hydra/common/hydra_config.h"
#include "hydra/reconstruction/mesh_integrator.h"
#include "hydra/reconstruction/projective_integrator.h"
#include "hydra/utils/timing_utilities.h"

namespace hydra {

using pose_graph_tools::PoseGraph;
using timing::ScopedTimer;
using voxblox::BlockIndexList;
using voxblox::Layer;

template <typename Voxel>
void mergeLayer(const Layer<Voxel>& layer_in,
                typename Layer<Voxel>::Ptr& layer_out,
                bool overwrite_updated = false) {
  if (!layer_out) {
    layer_out.reset(
        new Layer<Voxel>(layer_in.voxel_size(), layer_in.voxels_per_side()));
  }
  mergeLayer(layer_in, *layer_out, overwrite_updated);
}

ReconstructionModule::ReconstructionModule(const ReconstructionConfig& config,
                                           const OutputQueue::Ptr& output_queue)
    : config_(config::checkValid(config)),
      sensor_(config_.sensor.create()),
      num_poses_received_(0),
      pose_graph_tracker_(new PoseGraphTracker(config.pose_graphs)),
      output_queue_(output_queue) {
  queue_.reset(new ReconstructionInputQueue());
  queue_->max_size = config_.max_input_queue_size;

  map_.reset(new VolumetricMap(HydraConfig::instance().getMapConfig(), true, true));
  tsdf_integrator_ = std::make_unique<ProjectiveIntegrator>(config_.tsdf);
  mesh_integrator_ = std::make_unique<MeshIntegrator>(config_.mesh);
}

ReconstructionModule::~ReconstructionModule() { stop(); }

void ReconstructionModule::start() {
  spin_thread_.reset(new std::thread(&ReconstructionModule::spin, this));
  LOG(INFO) << "[Hydra Reconstruction] started!";
}

void ReconstructionModule::stop() {
  should_shutdown_ = true;

  if (spin_thread_) {
    VLOG(2) << "[Hydra Reconstruction] stopping reconstruction!";
    spin_thread_->join();
    spin_thread_.reset();
    VLOG(2) << "[Hydra Reconstruction] stopped!";
  }

  VLOG(2) << "[Hydra Reconstruction] input queue: " << queue_->size();
  if (output_queue_) {
    VLOG(2) << "[Hydra Reconstruction] output queue: " << output_queue_->size();
  } else {
    VLOG(2) << "[Hydra Reconstruction] output queue: n/a";
  }
}

void ReconstructionModule::save(const LogSetup&) {}

std::string ReconstructionModule::printInfo() const {
  std::stringstream ss;
  ss << std::endl << config::toString(config_);
  return ss.str();
}

void ReconstructionModule::spin() {
  // TODO(nathan) fix shutdown logic
  while (!should_shutdown_) {
    bool has_data = queue_->poll();
    if (!has_data) {
      continue;
    }

    spinOnce(*queue_->front());
    queue_->pop();
  }
}

bool ReconstructionModule::spinOnce() {
  bool has_data = queue_->poll();
  if (!has_data) {
    return false;
  }

  spinOnce(*queue_->front());
  queue_->pop();
  return true;
}

void ReconstructionModule::addOutputCallback(const OutputCallback& callback) {
  output_callbacks_.push_back(callback);
}

void ReconstructionModule::addVisualizationCallback(const VizCallback& callback) {
  visualization_callbacks_.push_back(callback);
}

bool ReconstructionModule::spinOnce(const ReconstructionInput& msg) {
  if (!msg.sensor_input) {
    LOG(ERROR) << "[Hydra Reconstruction] received invalid sensor data in input!";
    return false;
  }

  ScopedTimer timer("places/spin", msg.timestamp_ns);
  VLOG(2) << "[Hydra Reconstruction]: Processing msg @ " << msg.timestamp_ns;
  VLOG(2) << "[Hydra Reconstruction]: " << queue_->size() << " message(s) left";

  pose_graph_tracker_->update(msg);
  if (msg.agent_node_measurements) {
    agent_node_measurements_ = *msg.agent_node_measurements;
  }

  ++num_poses_received_;
  const bool do_full_update = (num_poses_received_ % config_.num_poses_per_update == 0);
  update(msg, do_full_update);
  return do_full_update;
}

void ReconstructionModule::fillOutput(const ReconstructionInput& input,
                                      ReconstructionOutput& output) {
  output.timestamp_ns = input.timestamp_ns;
  output.current_position = input.world_t_body;
  pose_graph_tracker_->fillPoseGraphs(output);
  if (agent_node_measurements_.nodes.size() > 0) {
    output.agent_node_measurements.reset(new PoseGraph(agent_node_measurements_));
    agent_node_measurements_ = PoseGraph();
  }

  // note that this is pre-archival
  map_->getMeshLayer().merge(output.mesh);
  if (config_.copy_dense_representations) {
    mergeLayer(map_->getTsdfLayer(), output.tsdf);
    mergeLayer(*map_->getOccupancyLayer(), output.occupied);
  }

  if (config_.clear_distant_blocks) {
    const auto to_archive = findBlocksToArchive(input.world_t_body.cast<float>());
    output.archived_blocks.insert(
        output.archived_blocks.end(), to_archive.begin(), to_archive.end());
    map_->removeBlocks(to_archive);
  }
}

ReconstructionModule::OutputMsgStatus ReconstructionModule::getNextOutputMessage() {
  if (!output_queue_) {
    return {std::make_shared<ReconstructionOutput>(), false};
  }

  // this is janky, but avoid pushing updates to queue if there's already stuff there
  const auto curr_size = output_queue_->size();
  if (curr_size >= 1) {
    if (!pending_output_) {
      pending_output_ = std::make_shared<ReconstructionOutput>();
    }

    return {pending_output_, true};
  }

  if (pending_output_) {
    auto to_return = pending_output_;
    pending_output_.reset();
    return {to_return, false};
  }

  return {std::make_shared<ReconstructionOutput>(), false};
}

void ReconstructionModule::update(const ReconstructionInput& msg, bool full_update) {
  VLOG(1) << "[Hydra Reconstruction] starting " << ((full_update) ? "full" : "partial")
          << " update @ " << msg.timestamp_ns << " [ns]";
  FrameData data;
  if (!msg.fillFrameData(data)) {
    LOG(ERROR) << "[Hydra Reconstruction] unable to construct valid input packet!";
    return;
  }

  if (!data.normalizeData()) {
    LOG(ERROR) << "[Hydra Reconstruction] unable to convert all data";
  }

  if (!sensor_->finalizeRepresentations(data)) {
    LOG(ERROR) << "[Hydra Reconstruction] unable to compute inputs for integration";
    return;
  }

  {  // timing scope
    ScopedTimer timer("places/tsdf", msg.timestamp_ns);
    tsdf_integrator_->updateMap(*sensor_, data, *map_);
  }  // timing scope

  if (map_->getTsdfLayer().getNumberOfAllocatedBlocks() == 0) {
    return;
  }

  {  // timing scope
    ScopedTimer timer("places/mesh", msg.timestamp_ns);
    mesh_integrator_->generateMesh(*map_, true, true);
  }  // timing scope

  if (!full_update) {
    return;
  }

  if (config_.show_stats) {
    VLOG(config_.stats_verbosity) << "Memory used: {" << map_->printStats() << "}";
  }

  auto&& [output, is_pending] = getNextOutputMessage();
  fillOutput(msg, *output);
  if (!is_pending) {
    output_queue_->push(output);
  } else {
    VLOG(1)
        << "[Hydra Reconstruction] Merging pending updates because frontend is slow!";
  }

  VLOG(5) << "[Hydra Reconstruction] Exported " << output->pose_graphs.size()
          << " pose graphs";
  for (const auto& callback : output_callbacks_) {
    callback(*output);
  }

  for (const auto& callback : visualization_callbacks_) {
    callback(msg.timestamp_ns, data.getSensorPose(*sensor_), map_->getTsdfLayer());
  }

  auto& tsdf = map_->getTsdfLayer();
  auto& mesh = map_->getMeshLayer();
  BlockIndexList blocks;
  tsdf.getAllUpdatedBlocks(voxblox::Update::kEsdf, &blocks);
  for (const auto& idx : blocks) {
    tsdf.getBlockByIndex(idx).updated().reset(voxblox::Update::kEsdf);
    mesh.getMeshBlock(idx)->updated = false;
  }
}

// TODO(nathan) push to map?
BlockIndexList ReconstructionModule::findBlocksToArchive(
    const Eigen::Vector3f& center) const {
  const auto& tsdf = map_->getTsdfLayer();
  BlockIndexList blocks;
  tsdf.getAllAllocatedBlocks(&blocks);

  BlockIndexList to_archive;
  for (const auto& idx : blocks) {
    auto block = tsdf.getBlockPtrByIndex(idx);
    if ((center - block->origin()).norm() < config_.dense_representation_radius_m) {
      continue;
    }

    // TODO(nathan) filter by update flag?
    to_archive.push_back(idx);
  }

  return to_archive;
}

}  // namespace hydra
