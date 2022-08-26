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
#include "hydra_topology/reconstruction_module.h"

#include <kimera_semantics/semantic_tsdf_integrator_factory.h>
#include <tf2_eigen/tf2_eigen.h>

namespace hydra {

using kimera::SemanticTsdfIntegratorFactory;
using kimera::SemanticVoxel;
using pose_graph_tools::PoseGraph;
using pose_graph_tools::PoseGraphEdge;
using pose_graph_tools::PoseGraphNode;
using topology::GvdIntegrator;
using topology::GvdVoxel;
using voxblox::BlockIndexList;
using voxblox::Layer;
using voxblox::MeshLayer;
using voxblox::TsdfVoxel;

inline Eigen::Affine3d getPose(const geometry_msgs::Pose& pose) {
  Eigen::Affine3d world_T_body;
  tf2::convert(pose, world_T_body);
  return world_T_body;
}

ReconstructionModule::ReconstructionModule(const RobotPrefixConfig& prefix,
                                           const ReconstructionConfig& config,
                                           const OutputQueue::Ptr& output_queue)
    : prefix_(prefix),
      config_(config),
      output_queue_(output_queue),
      num_poses_received_(0) {
  config_.semantics.semantic_label_to_color_.reset(
      new kimera::SemanticLabel2Color(config_.semantic_label_file));

  queue_.reset(new ReconstructionInputQueue());

  tsdf_.reset(new Layer<TsdfVoxel>(config_.voxel_size, config_.voxels_per_side));
  semantics_.reset(
      new Layer<SemanticVoxel>(config_.voxel_size, config_.voxels_per_side));
  gvd_.reset(new Layer<GvdVoxel>(config_.voxel_size, config_.voxels_per_side));
  mesh_.reset(new MeshLayer(tsdf_->block_size()));

  tsdf_integrator_ = SemanticTsdfIntegratorFactory::create(
      "fast", config_.tsdf, config_.semantics, tsdf_.get(), semantics_.get());
  gvd_integrator_.reset(new GvdIntegrator(config_.gvd, tsdf_.get(), gvd_, mesh_));

  output_.reset(new ReconstructionOutput());
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
}

void ReconstructionModule::save(const std::string&) {}

void ReconstructionModule::spin() {
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

void ReconstructionModule::spinOnce(const ReconstructionInput& msg) {
  if (!msg.pointcloud || !msg.pointcloud_colors) {
    LOG(ERROR) << "received invalid pointcloud in input!";
    return;
  }

  Eigen::Affine3d curr_pose(Eigen::Translation3d(msg.world_t_body) * msg.world_R_body);
  if (!msg.pose_graph && num_poses_received_ != 0) {
    PoseGraph::ConstPtr pose_graph(
        new PoseGraph(makePoseGraph(msg.timestamp_ns, curr_pose)));
    output_->pose_graphs.push_back(pose_graph);
  } else if (msg.pose_graph) {
    output_->pose_graphs.push_back(msg.pose_graph);
  }

  prev_pose_ = curr_pose;
  prev_time_ = msg.timestamp_ns;
  ++num_poses_received_;

  // TODO(nathan) handle body camera transform
  const bool do_full_update = (num_poses_received_ % config_.num_poses_per_update == 0);
  voxblox::Transformation world_T_body(msg.world_R_body.cast<float>(),
                                       msg.world_t_body.cast<float>());
  const auto archived_blocks = update(world_T_body,
                                      *msg.pointcloud,
                                      *msg.pointcloud_colors,
                                      msg.timestamp_ns,
                                      do_full_update);
  if (!do_full_update) {
    return;
  }

  output_->timestamp_ns = msg.timestamp_ns;
  output_->current_position = msg.world_t_body;
  addPlacesToOutput(*output_, msg.timestamp_ns);
  addMeshToOutput(archived_blocks, *output_, msg.timestamp_ns);

  if (output_queue_) {
    output_queue_->push(output_);
  }

  for (const auto& callback : output_callbacks_) {
    callback(*this, *output_);
  }

  output_.reset(new ReconstructionOutput());
}

void ReconstructionModule::addPlacesToOutput(ReconstructionOutput& output,
                                             size_t timestamp_ns) {
  output.places.reset(new hydra_msgs::ActiveLayer());
  auto& places = const_cast<hydra_msgs::ActiveLayer&>(*output.places);
  places.header.stamp.fromNSec(timestamp_ns);
  places.header.frame_id = config_.world_frame;
  // non-const, as clearDeletedNodes modifies internal state
  auto& extractor = gvd_integrator_->getGraphExtractor();
  std::unordered_set<NodeId> active_nodes = extractor.getActiveNodes();
  std::unordered_set<NodeId> removed_nodes = extractor.getDeletedNodes();
  extractor.clearDeletedNodes();
  places.layer_contents = extractor.getGraph().serializeLayer(active_nodes);
  places.deleted_nodes.insert(
      places.deleted_nodes.begin(), removed_nodes.begin(), removed_nodes.end());

  VLOG(5) << "[Hydra Reconstruction] exporting active: " << active_nodes.size()
          << " and deleted: " << removed_nodes.size() << "nodes";
}

void ReconstructionModule::addMeshToOutput(const BlockIndexList& archived_blocks,
                                           ReconstructionOutput& output,
                                           size_t timestamp_ns) {
  output.mesh.reset(new hydra_msgs::ActiveMesh());
  auto& active_mesh = const_cast<hydra_msgs::ActiveMesh&>(*output.mesh);
  active_mesh.header.stamp.fromNSec(timestamp_ns);
  // TODO(nathan) selectable color mode?
  generateVoxbloxMeshMsg(mesh_.get(),
                         voxblox::ColorMode::kColor,
                         &active_mesh.mesh,
                         active_mesh.header.stamp);
  active_mesh.mesh.header.frame_id = config_.world_frame;
  active_mesh.mesh.header.stamp = active_mesh.header.stamp;

  auto iter = active_mesh.mesh.mesh_blocks.begin();
  while (iter != active_mesh.mesh.mesh_blocks.end()) {
    // we can't just check if the message is empty (it's valid for an observed and
    // active block to be empty), so we have to check if the GVD layer has pruned the
    // corresponding block yet)
    voxblox::BlockIndex idx(iter->index[0], iter->index[1], iter->index[2]);
    if (!gvd_->hasBlock(idx)) {
      iter = active_mesh.mesh.mesh_blocks.erase(iter);
      continue;
    }

    ++iter;
  }

  for (const auto& block_idx : archived_blocks) {
    voxblox_msgs::MeshBlock block;
    block.index[0] = block_idx.x();
    block.index[1] = block_idx.y();
    block.index[2] = block_idx.z();
    active_mesh.archived_blocks.mesh_blocks.push_back(block);
  }
}

void ReconstructionModule::showStats() const {
  const std::string tsdf_memory_str =
      hydra_utils::getHumanReadableMemoryString(tsdf_->getMemorySize());
  const std::string gvd_memory_str =
      hydra_utils::getHumanReadableMemoryString(gvd_->getMemorySize());
  const std::string mesh_memory_str =
      hydra_utils::getHumanReadableMemoryString(mesh_->getMemorySize());
  LOG(INFO) << "Memory used: [TSDF=" << tsdf_memory_str << ", GVD=" << gvd_memory_str
            << ", Mesh= " << mesh_memory_str << "]";
}

BlockIndexList ReconstructionModule::update(const voxblox::Transformation& T_G_C,
                                            const voxblox::Pointcloud& pointcloud,
                                            const voxblox::Colors& colors,
                                            size_t timestamp_ns,
                                            bool full_update) {
  BlockIndexList archived_blocks;
  tsdf_integrator_->integratePointCloud(T_G_C, pointcloud, colors, false);

  if (!tsdf_ || tsdf_->getNumberOfAllocatedBlocks() == 0) {
    return archived_blocks;
  }

  if (!full_update) {
    return archived_blocks;
  }

  gvd_integrator_->updateFromTsdfLayer(timestamp_ns, true);

  if (config_.clear_distant_blocks) {
    archived_blocks = gvd_integrator_->removeDistantBlocks(
        T_G_C.getPosition(), config_.dense_representation_radius_m);

    mesh_->clearDistantMesh(T_G_C.getPosition(), config_.dense_representation_radius_m);
  }

  if (config_.show_stats) {
    showStats();
  }

  return archived_blocks;
}

void ReconstructionModule::fillPoseGraphNode(PoseGraphNode& node,
                                             size_t timestamp_ns,
                                             const Eigen::Affine3d& pose,
                                             size_t index) const {
  node.header.stamp.fromNSec(timestamp_ns);
  node.header.frame_id = config_.world_frame;
  node.robot_id = prefix_.id;
  node.key = index;
  tf2::convert(pose, node.pose);
}

void ReconstructionModule::makePoseGraphEdge(PoseGraph& graph) const {
  PoseGraphEdge edge;
  edge.header.stamp = graph.nodes.back().header.stamp;
  edge.header.frame_id = config_.world_frame;
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

PoseGraph ReconstructionModule::makePoseGraph(size_t timestamp_ns,
                                              const Eigen::Affine3d& curr_pose) {
  PoseGraph pose_graph;
  pose_graph.header.stamp.fromNSec(timestamp_ns);
  pose_graph.header.frame_id = config_.world_frame;

  pose_graph.nodes.push_back(PoseGraphNode());
  fillPoseGraphNode(
      pose_graph.nodes.back(), prev_time_, prev_pose_, num_poses_received_ - 1);

  pose_graph.nodes.push_back(PoseGraphNode());
  fillPoseGraphNode(
      pose_graph.nodes.back(), timestamp_ns, curr_pose, num_poses_received_);

  makePoseGraphEdge(pose_graph);
  return pose_graph;
}

}  // namespace hydra
