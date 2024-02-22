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
#include <pose_graph_tools_ros/conversions.h>
#include <voxblox/core/block.h>

#include "hydra/common/hydra_config.h"
#include "hydra/reconstruction/mesh_integrator.h"
#include "hydra/reconstruction/projective_integrator.h"
#include "hydra/reconstruction/volumetric_map.h"
#include "hydra/utils/timing_utilities.h"
#include "voxblox/core/common.h"
#include "voxblox/core/voxel.h"

namespace hydra {

using pose_graph_tools_msgs::PoseGraph;
using timing::ScopedTimer;
using voxblox::BlockIndexList;
using voxblox::Layer;

ReconstructionModule::ReconstructionModule(const ReconstructionConfig& config,
                                           const OutputQueue::Ptr& output_queue)
    : config_(config::checkValid(config)),
      sensor_(config_.sensor.create()),
      num_poses_received_(0),
      pose_graph_tracker_(new PoseGraphTracker(config.pose_graphs)),
      output_queue_(output_queue) {
  queue_.reset(new ReconstructionInputQueue());
  queue_->max_size = config_.max_input_queue_size;

  map_.reset(
      new VolumetricMap(HydraConfig::instance().getMapConfig(), true, true, true));
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

  const auto success = spinOnce(*queue_->front());
  queue_->pop();
  return success;
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
  pose_graph_tracker_->fillPoseGraphs(output);
  if (agent_node_measurements_.nodes.size() > 0) {
    output.agent_node_measurements.reset(new pose_graph_tools::PoseGraph(
        pose_graph_tools::fromMsg(agent_node_measurements_)));
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

template <typename VoxelType>
void computeSdfGradient(const std::shared_ptr<voxblox::Block<VoxelType>> block_ptr,
                        const size_t i,
                        const size_t j,
                        const size_t k,
                        Eigen::Vector3f& gradient) {
  Eigen::Vector3f new_gradient;
  new_gradient.setZero();
  int offsets[3] = {-1, 0, 1};
  int n_neighbors = 0;
  for (int di : offsets) {
    for (int dj : offsets) {
      for (int dk : offsets) {
        if (di == 0 && dj == 0 && dk == 0) {
          continue;
        }
        if (!block_ptr->isValidVoxelIndex({i + di, j + dj, k + dk})) {
          continue;
        }
        if (block_ptr->getVoxelByVoxelIndex({i + di, j + dj, k + dk}).weight <= 1e-6) {
          continue;
        }
        float val_neighbor =
            block_ptr->getVoxelByVoxelIndex({i + di, j + dj, k + dk}).distance;
        Eigen::Vector3f grad_observation = {di, dj, dk};
        float neighbor_distance = grad_observation.norm();
        grad_observation *=
            val_neighbor - block_ptr->getVoxelByVoxelIndex({i, j, k}).distance;
        grad_observation /= neighbor_distance;
        new_gradient += grad_observation;
        n_neighbors += 1;
      }
    }
  }
  if (n_neighbors > 0) {
    new_gradient /= n_neighbors;
    new_gradient /= (gradient.norm() + 1e-6);  // SDF should have gradient with norm 1
    gradient = new_gradient;
  }
}

bool ReconstructionModule::update(const ReconstructionInput& msg, bool full_update) {
  VLOG(1) << "[Hydra Reconstruction] starting " << ((full_update) ? "full" : "partial")
          << " update @ " << msg.timestamp_ns << " [ns]";
  FrameData data;
  if (!msg.fillFrameData(data)) {
    LOG(ERROR) << "[Hydra Reconstruction] unable to construct valid input packet!";
    return false;
  }

  if (!data.normalizeData()) {
    LOG(ERROR) << "[Hydra Reconstruction] unable to convert all data";
    return false;
  }

  if (!sensor_->finalizeRepresentations(data)) {
    LOG(ERROR) << "[Hydra Reconstruction] unable to compute inputs for integration";
    return false;
  }

  {  // timing scope
    ScopedTimer timer("places/tsdf", msg.timestamp_ns);
    tsdf_integrator_->updateMap(*sensor_, data, *map_);
  }  // timing scope

  if (map_->getTsdfLayer().getNumberOfAllocatedBlocks() == 0) {
    return false;
  }

  /////////////////
  // This is where we implement laplace smoothing
  std::vector<voxblox::GlobalIndex> indices_to_restore;
  VolumetricMap::TsdfLayer& m = map_->getTsdfLayer();
  VolumetricMap::ExtrapolationLayer* m_ex = map_->getExtrapolationLayer();
  BlockIndexList smoothing_blocks;
  m.getAllAllocatedBlocks(&smoothing_blocks);

  LOG(WARNING) << "Starting smoothing";
  // 1. iterate through tsdf. For each cell with weight >= 0, update extrapolation layer
  for (voxblox::BlockIndex bix : smoothing_blocks) {
    auto block_ptr = m.getBlockPtrByIndex(bix);
    for (size_t v = 0; v < block_ptr->num_voxels(); ++v) {
      voxblox::VoxelIndex ind = block_ptr->computeVoxelIndexFromLinearIndex(v);
      int i = ind.x();
      int j = ind.y();
      int k = ind.z();

      voxblox::TsdfVoxel& voxel = block_ptr->getVoxelByLinearIndex(v);
      voxblox::GlobalIndex gix = voxblox::getGlobalVoxelIndexFromBlockAndVoxelIndex(
          bix, ind, block_ptr->voxels_per_side());
      if (voxel.weight >= 1e-6) {
        ExtrapolationVoxel* extrap_voxel = m_ex->getVoxelPtrByGlobalIndex(gix);
        extrap_voxel->distance = voxel.distance;
        extrap_voxel->nearest_distance = 0;
        computeSdfGradient(block_ptr, i, j, k, extrap_voxel->gradient);
      }
    }
  }

  LOG(WARNING) << "Finished tsdf update";
  int offsets[3] = {-1, 0, 1};
  // 2. update each gradient (to the mean of its neighbors and restore norm-1 ?)
  // 4. update each distance-to-observed-voxel value
  for (voxblox::BlockIndex bix : smoothing_blocks) {
    auto block_ptr = m_ex->getBlockPtrByIndex(bix);
    for (size_t v = 0; v < block_ptr->num_voxels(); ++v) {
      voxblox::VoxelIndex ind = block_ptr->computeVoxelIndexFromLinearIndex(v);
      int i = ind.x();
      int j = ind.y();
      int k = ind.z();
      ExtrapolationVoxel& voxel = block_ptr->getVoxelByLinearIndex(v);
      if (voxel.nearest_distance == 0) {
        continue;
      }
      Eigen::Vector3f accum;
      accum.setZero();
      int n_neighbors = 0;
      for (int di : offsets) {
        for (int dj : offsets) {
          for (int dk : offsets) {
            if (di == 0 && dj == 0 && dk == 0) {
              continue;
            }
            if (!block_ptr->isValidVoxelIndex({i + di, j + dj, k + dk})) {
              continue;
            }

            float d_neighbor = sqrt(di * di + dj * dj + dk * dk);
            accum +=
                block_ptr->getVoxelByVoxelIndex({i + di, j + dj, k + dk}).gradient /
                d_neighbor;
            n_neighbors += 1;

            // update distance-to-observed;
            voxel.nearest_distance =
                std::min(voxel.nearest_distance,
                         block_ptr->getVoxelByVoxelIndex({i + di, j + dj, k + dk})
                                 .nearest_distance +
                             d_neighbor);
          }
        }
      }
      voxel.gradient = accum / n_neighbors;
      voxel.gradient /= voxel.gradient.norm() + 1e-6;
    }
  }
  LOG(WARNING) << "Finished gradient update";

  // 3. update each value (update based on propagating neighbor values based on current
  // voxel's gradient)

  for (voxblox::BlockIndex bix : smoothing_blocks) {
    auto block_ptr = m_ex->getBlockPtrByIndex(bix);
    for (size_t v = 0; v < block_ptr->num_voxels(); ++v) {
      voxblox::VoxelIndex ind = block_ptr->computeVoxelIndexFromLinearIndex(v);
      int i = ind.x();
      int j = ind.y();
      int k = ind.z();
      ExtrapolationVoxel& voxel = block_ptr->getVoxelByLinearIndex(v);
      if (voxel.nearest_distance == 0) {
        continue;
      }
      float accum = 0;
      int n_neighbors = 0;
      for (int di : offsets) {
        for (int dj : offsets) {
          for (int dk : offsets) {
            if (di == 0 && dj == 0 && dk == 0) {
              continue;
            }
            if (!block_ptr->isValidVoxelIndex({i + di, j + dj, k + dk})) {
              continue;
            }

            ExtrapolationVoxel& neighbor_voxel =
                block_ptr->getVoxelByVoxelIndex({i + di, j + dj, k + dk});

            Eigen::Vector3f vec_from_neighbor = {-di, -dj, -dk};
            float d_neighbor = vec_from_neighbor.norm();
            accum += neighbor_voxel.distance +
                     neighbor_voxel.gradient.dot(vec_from_neighbor);
            // LOG(WARNING) << "Direction grad update: "
            //             << neighbor_voxel.gradient.dot(vec_from_neighbor);
            n_neighbors += 1;

            // update distance-to-observed;
            voxel.nearest_distance = std::min(
                voxel.nearest_distance, neighbor_voxel.nearest_distance + d_neighbor);
          }
        }
      }
      voxel.distance = accum / n_neighbors;
      // LOG(WARNING) << "Voxel distance: " << voxel.distance;

      // 5. for each updated value, write back to tsdf and store index
      voxblox::GlobalIndex gix = voxblox::getGlobalVoxelIndexFromBlockAndVoxelIndex(
          bix, ind, block_ptr->voxels_per_side());
      voxblox::TsdfVoxel* tsdf_voxel = m.getVoxelPtrByGlobalIndex(gix);
      tsdf_voxel->distance = voxel.distance;
      tsdf_voxel->weight = 1;
      indices_to_restore.push_back(gix);
    }
  }
  LOG(WARNING) << "Finished value update";

  ////////////////////
  // original approach with no gradient information
  // int n_iters = 1;
  // for (int iter_count = 0; iter_count < n_iters; ++iter_count) {
  //  for (voxblox::BlockIndex bix : smoothing_blocks) {
  //    auto block_ptr = m.getBlockPtrByIndex(bix);
  //    for (size_t v = 0; v < block_ptr->num_voxels(); ++v) {
  //      voxblox::VoxelIndex ind = block_ptr->computeVoxelIndexFromLinearIndex(v);
  //      int i = ind.x();
  //      int j = ind.y();
  //      int k = ind.z();
  //      voxblox::TsdfVoxel& voxel = block_ptr->getVoxelByLinearIndex(v);
  //      if (voxel.weight >= 1e-6) {
  //        continue;
  //      }
  //      voxblox::GlobalIndex gid = voxblox::getGlobalVoxelIndexFromBlockAndVoxelIndex(
  //          bix, ind, block_ptr->voxels_per_side());
  //      indices_to_restore.push_back(gid);
  //      double accum = 0;
  //      int n_neighbors = 0;
  //      for (int di : offsets) {
  //        for (int dj : offsets) {
  //          for (int dk : offsets) {
  //            if (di == 0 && dj == 0 && dk == 0) {
  //              continue;
  //            }
  //            if (!block_ptr->isValidVoxelIndex({i + di, j + dj, k + dk})) {
  //              continue;
  //            }
  //            accum +=
  //                1 / sqrt(di * di + dj * dj + dk * dk) *
  //                block_ptr->getVoxelByVoxelIndex({i + di, j + dj, k + dk}).distance;
  //            n_neighbors += 1;
  //          }
  //        }
  //      }
  //      voxel.distance = accum / n_neighbors;
  //      voxel.weight = 1;
  //    }
  //  }
  //}

  ////////////////////////////

  {  // timing scope
    ScopedTimer timer("places/mesh", msg.timestamp_ns);
    mesh_integrator_->generateMesh(*map_, true, true);
  }  // timing scope

  if (!full_update) {
    return false;
  }

  if (config_.show_stats) {
    VLOG(config_.stats_verbosity) << "Memory used: {" << map_->printStats() << "}";
  }

  auto&& [output, is_pending] = getNextOutputMessage();
  fillOutput(msg, *output);
  if (output_queue_ && !is_pending) {
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

  for (voxblox::GlobalIndex gix : indices_to_restore) {
    voxblox::TsdfVoxel* vp = map_->getTsdfLayer().getVoxelPtrByGlobalIndex(gix);
    if (vp == nullptr) {
      continue;
    }
    vp->weight = 0;
  }

  return true;
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
