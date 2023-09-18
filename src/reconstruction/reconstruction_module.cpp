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
#include <kimera_semantics/semantic_tsdf_integrator_fast.h>
#include <tf2_eigen/tf2_eigen.h>
#include <yaml-cpp/yaml.h>

#include "hydra/common/hydra_config.h"
#include "hydra/reconstruction/mesh_integrator.h"
#include "hydra/utils/display_utilities.h"
#include "hydra/utils/timing_utilities.h"

namespace hydra {

using kimera::SemanticConfig;
using kimera::SemanticVoxel;
using places::VertexVoxel;
using pose_graph_tools::PoseGraph;
using pose_graph_tools::PoseGraphEdge;
using pose_graph_tools::PoseGraphNode;
using timing::ScopedTimer;
using voxblox::BlockIndexList;
using voxblox::Layer;
using voxblox::MeshLayer;
using voxblox::TsdfVoxel;

inline Eigen::Affine3d getPose(const geometry_msgs::Pose& pose) {
  Eigen::Affine3d world_T_body;
  tf2::convert(pose, world_T_body);
  return world_T_body;
}

std::string showQuaternion(const Eigen::Quaterniond& q) {
  std::stringstream ss;
  ss << "{w: " << q.w() << ", x: " << q.x() << ", y: " << q.y() << ", z: " << q.z()
     << "}";
  return ss.str();
}

bool loadExtrinsicsFromKimera(ReconstructionConfig& config,
                              const std::string& filename) {
  const auto node = YAML::LoadFile(filename);
  const auto elements = node["T_BS"]["data"].as<std::vector<double>>();
  if (elements.size() != 16u) {
    LOG(ERROR) << "Invalid transform matrix";
    return false;
  }

  Eigen::Matrix4d body_T_camera;
  for (size_t r = 0; r < 4; r++) {
    for (size_t c = 0; c < 4; c++) {
      body_T_camera(r, c) = elements.at(4 * r + c);
    }
  }

  config.body_R_camera = Eigen::Quaterniond(body_T_camera.block<3, 3>(0, 0));
  config.body_t_camera = body_T_camera.block<3, 1>(0, 3);
  LOG(INFO) << "Loaded extrinsics from Kimera: R="
            << showQuaternion(config.body_R_camera)
            << ", t=" << config.body_t_camera.transpose();
  return true;
}

SemanticConfig createSemanticConfig(const ReconstructionConfig& config) {
  SemanticConfig new_config;
  new_config.measurement_probability = config.semantic_measurement_probability;

  const auto& label_space = HydraConfig::instance().getLabelSpaceConfig();
  new_config.dynamic_labels = label_space.dynamic_labels;
  new_config.invalid_labels = label_space.invalid_labels;
  new_config.desired_number_labels = HydraConfig::instance().getTotalLabels();
  new_config.color_mode = kimera::ColorMode::kColor;  // disable overwriting rgb
  return new_config;
}

ReconstructionModule::ReconstructionModule(const ReconstructionConfig& config,
                                           const RobotPrefixConfig& prefix,
                                           const OutputQueue::Ptr& output_queue)
    : config_(config),
      prefix_(prefix),
      output_queue_(output_queue),
      num_poses_received_(0) {
  queue_.reset(new ReconstructionInputQueue());
  queue_->max_size = config_.max_input_queue_size;

  const auto voxel_size = config_.voxel_size;
  const auto voxels_per_side = config_.voxels_per_side;
  tsdf_.reset(new Layer<TsdfVoxel>(voxel_size, voxels_per_side));
  semantics_.reset(new Layer<SemanticVoxel>(voxel_size, voxels_per_side));
  vertices_.reset(new Layer<VertexVoxel>(voxel_size, voxels_per_side));
  mesh_.reset(new SemanticMeshLayer(tsdf_->block_size()));

  const auto semantic_config = createSemanticConfig(config_);
  tsdf_integrator_ = std::make_unique<kimera::FastSemanticTsdfIntegrator>(
      config_.tsdf, semantic_config, tsdf_.get(), semantics_.get());
  mesh_integrator_.reset(
      new MeshIntegrator(config_.mesh, tsdf_, vertices_, mesh_, semantics_));
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

voxblox::Transformation ReconstructionModule::getCameraPose(
    const ReconstructionInput& msg) const {
  voxblox::Transformation body_T_camera(config_.body_R_camera.cast<float>(),
                                        config_.body_t_camera.cast<float>());
  voxblox::Transformation world_T_body(msg.world_R_body.cast<float>(),
                                       msg.world_t_body.cast<float>());

  const auto result = world_T_body * body_T_camera;
  return result;
}

void ReconstructionModule::addOutputCallback(const OutputCallback& callback) {
  output_callbacks_.push_back(callback);
}

bool ReconstructionModule::spinOnce(const ReconstructionInput& msg) {
  if (msg.pointcloud.empty() || msg.pointcloud_colors.empty() ||
      msg.pointcloud_labels.empty()) {
    LOG(ERROR) << "[Hydra Reconstruction] received invalid pointcloud in input!";
    return false;
  }

  ScopedTimer timer("places/spin", msg.timestamp_ns);
  VLOG(2) << "[Hydra Reconstruction]: Processing msg @ " << msg.timestamp_ns;
  VLOG(2) << "[Hydra Reconstruction]: " << queue_->size() << " message(s) left";

  Eigen::Affine3d curr_pose(Eigen::Translation3d(msg.world_t_body) * msg.world_R_body);
  if (config_.make_pose_graph && num_poses_received_ > 0) {
    PoseGraph::ConstPtr pose_graph(
        new PoseGraph(makePoseGraph(msg.timestamp_ns, curr_pose)));
    pose_graphs_.push_back(pose_graph);
  } else {
    pose_graphs_.insert(
        pose_graphs_.end(), msg.pose_graphs.begin(), msg.pose_graphs.end());
  }

  if (msg.agent_node_measurements) {
    agent_node_measurements_ = *msg.agent_node_measurements;
  }

  prev_pose_ = curr_pose;
  prev_time_ = msg.timestamp_ns;
  ++num_poses_received_;

  const bool do_full_update = (num_poses_received_ % config_.num_poses_per_update == 0);
  update(msg, do_full_update);
  return do_full_update;
}

template <typename Voxel>
void mergeLayer(const Layer<Voxel>& layer_in, typename Layer<Voxel>::Ptr& layer_out) {
  if (!layer_out) {
    layer_out.reset(
        new Layer<Voxel>(layer_in.voxel_size(), layer_in.voxels_per_side()));
  }

  BlockIndexList blocks;
  layer_in.getAllAllocatedBlocks(&blocks);
  for (const auto& idx : blocks) {
    auto block = layer_in.getBlockPtrByIndex(idx);
    if (!block) {
      continue;
    }

    auto new_block = layer_out->allocateBlockPtrByIndex(idx);
    for (size_t i = 0; i < block->num_voxels(); ++i) {
      new_block->getVoxelByLinearIndex(i) = block->getVoxelByLinearIndex(i);
    }

    new_block->updated() |= block->updated();
    new_block->has_data() = block->has_data();
    // copy other block attributes...
  }
}

void ReconstructionModule::fillOutput(const ReconstructionInput& input,
                                      ReconstructionOutput& output) {
  output.timestamp_ns = input.timestamp_ns;
  output.current_position = input.world_t_body;
  if (agent_node_measurements_.nodes.size() > 0) {
    output.agent_node_measurements.reset(new PoseGraph(agent_node_measurements_));
    agent_node_measurements_ = PoseGraph();
  }

  // note that this is pre-archival
  if (config_.copy_dense_representations) {
    mergeLayer(*tsdf_, output.tsdf);
    mergeLayer(*vertices_, output.occupied);
  }

  mesh_->merge(output.mesh);

  // move and clear cached pose graphs
  VLOG(5) << "[Hydra Reconstruction] Current queued pose graphs: "
          << pose_graphs_.size();
  output.pose_graphs.insert(
      output.pose_graphs.end(), pose_graphs_.begin(), pose_graphs_.end());
  pose_graphs_.clear();

  if (config_.clear_distant_blocks) {
    const auto to_archive = findBlocksToArchive(input.world_t_body.cast<float>());
    output.archived_blocks.insert(
        output.archived_blocks.end(), to_archive.begin(), to_archive.end());
    for (const auto& index : to_archive) {
      semantics_->removeBlock(index);
      tsdf_->removeBlock(index);
      mesh_->removeBlock(index);
      vertices_->removeBlock(index);
    }
  }
}

void ReconstructionModule::update(const ReconstructionInput& msg, bool full_update) {
  const auto world_T_camera = getCameraPose(msg);

  {  // timing scope
    ScopedTimer timer("places/tsdf", msg.timestamp_ns);
    tsdf_integrator_->integratePointCloud(world_T_camera,
                                          msg.pointcloud,
                                          msg.pointcloud_colors,
                                          msg.pointcloud_labels,
                                          false);
  }  // timing scope

  if (!tsdf_ || tsdf_->getNumberOfAllocatedBlocks() == 0) {
    return;
  }

  {  // timing scope
    ScopedTimer timer("places/mesh", msg.timestamp_ns);
    mesh_integrator_->generateMesh(true, true);
  }  // timing scope

  if (!full_update) {
    return;
  }

  if (config_.show_stats) {
    showStats();
  }

  bool is_pending = false;
  ReconstructionOutput::Ptr output;
  if (!output_queue_) {
    output = std::make_shared<ReconstructionOutput>();
  } else {
    // this is janky, but avoid pushing updates to queue if there's already stuff there
    const auto curr_size = output_queue_->size();
    if (curr_size >= 1) {
      if (!pending_output_) {
        pending_output_ = std::make_shared<ReconstructionOutput>();
      }
      output = pending_output_;
      is_pending = true;
    } else {
      output =
          pending_output_ ? pending_output_ : std::make_shared<ReconstructionOutput>();
      pending_output_.reset();
    }
  }

  fillOutput(msg, *output);

  VLOG(5) << "[Hydra Reconstruction] Exporting " << output->pose_graphs.size()
          << " pose graphs";

  if (!is_pending) {
    output_queue_->push(output);
  } else {
    LOG(WARNING)
        << "[Hydra Reconstruction] Merging pending updates because frontend is slow!";
  }

  for (const auto& callback : output_callbacks_) {
    callback(*output);
  }

  BlockIndexList blocks;
  tsdf_->getAllUpdatedBlocks(voxblox::Update::kEsdf, &blocks);
  for (const auto& idx : blocks) {
    tsdf_->getBlockByIndex(idx).updated().reset(voxblox::Update::kEsdf);
    mesh_->getMeshBlock(idx)->updated = false;
  }
}

void ReconstructionModule::fillPoseGraphNode(PoseGraphNode& node,
                                             uint64_t timestamp_ns,
                                             const Eigen::Affine3d& pose,
                                             size_t index) const {
  node.header.stamp.fromNSec(timestamp_ns);
  node.header.frame_id = config_.odom_frame;
  node.robot_id = prefix_.id;
  node.key = index;
  tf2::convert(pose, node.pose);
}

void ReconstructionModule::makePoseGraphEdge(PoseGraph& graph) const {
  PoseGraphEdge edge;
  edge.header.stamp = graph.nodes.back().header.stamp;
  edge.header.frame_id = config_.odom_frame;
  edge.key_from = graph.nodes.front().key;
  edge.key_to = graph.nodes.back().key;
  edge.robot_from = prefix_.id;
  edge.robot_to = prefix_.id;
  edge.type = PoseGraphEdge::ODOM;

  Eigen::Affine3d world_T_body_i = getPose(graph.nodes.front().pose);
  Eigen::Affine3d world_T_body_j = getPose(graph.nodes.back().pose);
  Eigen::Affine3d body_i_T_body_j = world_T_body_i.inverse() * world_T_body_j;
  tf2::convert(body_i_T_body_j, edge.pose);
  graph.edges.push_back(edge);
}

PoseGraph ReconstructionModule::makePoseGraph(uint64_t timestamp_ns,
                                              const Eigen::Affine3d& curr_pose) {
  PoseGraph pose_graph;
  pose_graph.header.stamp.fromNSec(timestamp_ns);
  pose_graph.header.frame_id = config_.odom_frame;

  pose_graph.nodes.push_back(PoseGraphNode());
  fillPoseGraphNode(
      pose_graph.nodes.back(), prev_time_, prev_pose_, num_poses_received_ - 1);

  pose_graph.nodes.push_back(PoseGraphNode());
  fillPoseGraphNode(
      pose_graph.nodes.back(), timestamp_ns, curr_pose, num_poses_received_);

  makePoseGraphEdge(pose_graph);
  return pose_graph;
}

void ReconstructionModule::showStats() const {
  std::stringstream ss;
  const auto tsdf_size = tsdf_->getMemorySize();
  ss << "TSDF: " << getHumanReadableMemoryString(tsdf_size);

  const auto semantic_size = semantics_->getMemorySize();
  ss << ", SEMANTICS: " << getHumanReadableMemoryString(semantic_size);

  const auto occ_size = vertices_->getMemorySize();
  ss << ", OCCUPANCY: " << getHumanReadableMemoryString(occ_size);

  const auto mesh_size = mesh_->getMemorySize();
  ss << ", MESH: " << getHumanReadableMemoryString(mesh_size);

  const auto total_size = tsdf_size + semantic_size + occ_size + mesh_size;
  ss << ", TOTAL: " << getHumanReadableMemoryString(total_size);

  LOG(INFO) << "Memory used: {" << ss.str() << "}";
}

BlockIndexList ReconstructionModule::findBlocksToArchive(
    const voxblox::Point& center) const {
  BlockIndexList blocks;
  tsdf_->getAllAllocatedBlocks(&blocks);

  BlockIndexList to_archive;
  for (const auto& idx : blocks) {
    auto block = tsdf_->getBlockPtrByIndex(idx);
    if ((center - block->origin()).norm() < config_.dense_representation_radius_m) {
      continue;
    }

    // TODO(nathan) filter by update flag?
    to_archive.push_back(idx);
  }

  return to_archive;
}

}  // namespace hydra
