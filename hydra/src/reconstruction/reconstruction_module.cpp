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

#include <kimera_semantics/semantic_tsdf_integrator_factory.h>
#include <tf2_eigen/tf2_eigen.h>

#include "hydra/places/gvd_integrator.h"
#include "hydra/reconstruction/voxel_aware_mesh_integrator.h"
#include "hydra/utils/display_utilities.h"
#include "hydra/utils/timing_utilities.h"

namespace hydra {

using kimera::SemanticTsdfIntegratorFactory;
using kimera::SemanticVoxel;
using places::GvdIntegrator;
using places::GvdVoxel;
using places::VertexVoxel;
using pose_graph_tools::PoseGraph;
using pose_graph_tools::PoseGraphEdge;
using pose_graph_tools::PoseGraphNode;
using timing::ScopedTimer;
using voxblox::BlockIndexList;
using voxblox::Layer;
using voxblox::MeshLayer;
using voxblox::TsdfVoxel;

using MeshIntegrator = VoxelAwareMeshIntegrator;

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
  queue_->max_size = config_.max_input_queue_size;

  tsdf_.reset(new Layer<TsdfVoxel>(config_.voxel_size, config_.voxels_per_side));
  semantics_.reset(
      new Layer<SemanticVoxel>(config_.voxel_size, config_.voxels_per_side));

  gvd_.reset(new Layer<GvdVoxel>(config_.voxel_size, config_.voxels_per_side));
  vertices_.reset(new Layer<VertexVoxel>(config_.voxel_size, config_.voxels_per_side));
  mesh_.reset(new MeshLayer(tsdf_->block_size()));

  tsdf_integrator_ = SemanticTsdfIntegratorFactory::create(
      "fast", config_.tsdf, config_.semantics, tsdf_.get(), semantics_.get());
  mesh_integrator_.reset(
      new MeshIntegrator(config_.mesh, tsdf_.get(), vertices_, mesh_.get()));
  gvd_integrator_.reset(new GvdIntegrator(config_.gvd, gvd_));
}

ReconstructionModule::~ReconstructionModule() { stop(); }

void ReconstructionModule::start() {
  spin_thread_.reset(new std::thread(&ReconstructionModule::spin, this));
  gvd_thread_.reset(new std::thread(&ReconstructionModule::updateGvdSpin, this));
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

  if (gvd_thread_) {
    VLOG(2) << "[Hydra Reconstruction] stopping gvd thread!";
    gvd_thread_->join();
    gvd_thread_.reset();
    VLOG(2) << "[Hydra Reconstruction] gvd thread stopped!";
  }

  VLOG(2) << "[Hydra Reconstruction] input queue: " << queue_->size();
  if (output_queue_) {
    VLOG(2) << "[Hydra Reconstruction] output queue: " << output_queue_->size();
  } else {
    VLOG(2) << "[Hydra Reconstruction] output queue: n/a";
  }
}

void ReconstructionModule::save(const std::string& output_path) {
  const auto& original_places = gvd_integrator_->getGraph();
  auto places = original_places.clone();

  std::unique_ptr<DynamicSceneGraph::Edges> edges(new DynamicSceneGraph::Edges());
  for (const auto& id_edge_pair : places->edges()) {
    edges->emplace(std::piecewise_construct,
                   std::forward_as_tuple(id_edge_pair.first),
                   std::forward_as_tuple(id_edge_pair.second.source,
                                         id_edge_pair.second.target,
                                         id_edge_pair.second.info->clone()));
  }

  DynamicSceneGraph::Ptr graph(new DynamicSceneGraph());
  graph->updateFromLayer(*places, std::move(edges));
  graph->save(output_path + "/places.json", false);
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

std::vector<bool> ReconstructionModule::inFreespace(const PositionMatrix& positions,
                                                    double freespace_distance_m) const {
  if (positions.cols() < 1) {
    return {};
  }

  std::vector<bool> flags(positions.cols(), false);
  // starting lock on tsdf update
  std::unique_lock<std::mutex> lock(gvd_mutex_);
  for (int i = 0; i < positions.cols(); ++i) {
    const auto* voxel = gvd_->getVoxelPtrByCoordinates(positions.col(i).cast<float>());
    if (!voxel) {
      continue;
    }

    flags[i] = voxel->observed && voxel->distance > freespace_distance_m;
  }

  return flags;
}

bool ReconstructionModule::spinOnce(const ReconstructionInput& msg) {
  if (!msg.pointcloud || !msg.pointcloud_colors) {
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

  prev_pose_ = curr_pose;
  prev_time_ = msg.timestamp_ns;
  ++num_poses_received_;

  const bool do_full_update = (num_poses_received_ % config_.num_poses_per_update == 0);
  update(msg, do_full_update);
  return do_full_update;
}

void ReconstructionModule::update(const ReconstructionInput& msg, bool full_update) {
  std::unique_lock<std::mutex> lock(tsdf_mutex_);
  const auto world_T_camera = getCameraPose(msg);

  {  // timing scope
    ScopedTimer timer("places/tsdf", msg.timestamp_ns);
    tsdf_integrator_->integratePointCloud(
        world_T_camera, *msg.pointcloud, *msg.pointcloud_colors, false);
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

  auto output = std::make_shared<ReconstructionOutput>();
  output->timestamp_ns = msg.timestamp_ns;
  output->current_position = msg.world_t_body;
  output->pose_graphs = pose_graphs_;
  VLOG(5) << "[Hydra Reconstruction] Current queued pose graphs: "
          << pose_graphs_.size();
  pose_graphs_.clear();

  gvd_queue_.push(output);
}

void ReconstructionModule::updateGvdSpin() {
  // TODO(nathan) fix shutdown logic
  while (!should_shutdown_) {
    if (!gvd_queue_.poll()) {
      continue;
    }

    updateGvd();
  }
}

void ReconstructionModule::updateGvd() {
  std::unique_ptr<ScopedTimer> timer;
  ReconstructionOutput::Ptr msg;
  BlockIndexList archived_blocks;
  {  // start critical section
    std::unique_lock<std::mutex> lock(tsdf_mutex_);

    std::list<PoseGraph::ConstPtr> pose_graphs;
    // this is awkward, but we want to make sure we use the latest available timestamp
    while (gvd_queue_.size() > 1) {
      // TODO(nathan) cache pose graphs
      const auto& curr_graphs = gvd_queue_.front()->pose_graphs;
      pose_graphs.insert(pose_graphs.end(), curr_graphs.begin(), curr_graphs.end());
      gvd_queue_.pop();
    }

    msg = gvd_queue_.front();
    msg->pose_graphs.insert(
        msg->pose_graphs.begin(), pose_graphs.begin(), pose_graphs.end());
    timer.reset(new ScopedTimer("places/spin_gvd", msg->timestamp_ns));
    gvd_integrator_->updateFromTsdf(
        msg->timestamp_ns, *tsdf_, *vertices_, *mesh_, true);

    if (config_.clear_distant_blocks) {
      archived_blocks = findBlocksToArchive(msg->current_position.cast<float>());
      for (const auto& index : archived_blocks) {
        semantics_->removeBlock(index);
        tsdf_->removeBlock(index);
        mesh_->removeMesh(index);
      }
    }

    addMeshToOutput(*msg, archived_blocks);
  }  // end critical section

  {  // start critical section
    // TODO(nathan) this critical section is broken
    std::unique_lock<std::mutex> lock(gvd_mutex_);
    gvd_integrator_->updateGvd(msg->timestamp_ns);
    gvd_integrator_->archiveBlocks(archived_blocks);
  }  // end critical section

  if (config_.show_stats) {
    showStats();
  }

  addPlacesToOutput(*msg);

  if (output_queue_) {
    VLOG(5) << "[Hydra Reconstruction] Exporting " << msg->pose_graphs.size()
            << " pose graphs";
    output_queue_->push(msg);
  }

  for (const auto& callback : output_callbacks_) {
    callback(*this, *msg);
  }

  gvd_queue_.pop();
}

void ReconstructionModule::fillPoseGraphNode(PoseGraphNode& node,
                                             uint64_t timestamp_ns,
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

PoseGraph ReconstructionModule::makePoseGraph(uint64_t timestamp_ns,
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

void ReconstructionModule::addPlacesToOutput(ReconstructionOutput& output) {
  output.places.reset(new ActiveLayerInfo());
  auto& places = *output.places;
  // non-const, as clearDeletedNodes modifies internal state
  auto& extractor = gvd_integrator_->getGraphExtractor();
  std::unordered_set<NodeId> active_nodes = extractor.getActiveNodes();
  const auto& removed_nodes = extractor.getDeletedNodes();
  const auto& removed_edges = extractor.getDeletedEdges();

  const auto& graph = extractor.getGraph();
  std::list<NodeId> invalid_nodes;
  for (const auto& active_id : active_nodes) {
    const auto& node = graph.getNode(active_id);
    const auto& attrs = node->get().attributes<PlaceNodeAttributes>();
    if (std::isnan(attrs.distance)) {
      invalid_nodes.push_back(active_id);
      continue;
    }

    if (attrs.position.hasNaN()) {
      invalid_nodes.push_back(active_id);
      continue;
    }

    for (const auto& info : attrs.voxblox_mesh_connections) {
      if (std::isnan(info.voxel_pos[0]) || std::isnan(info.voxel_pos[1]) ||
          std::isnan(info.voxel_pos[2])) {
        invalid_nodes.push_back(active_id);
        continue;
      }
    }
  }

  places.deleted_nodes.insert(
      places.deleted_nodes.begin(), removed_nodes.begin(), removed_nodes.end());
  places.deleted_edges.insert(
      places.deleted_edges.begin(), removed_edges.begin(), removed_edges.end());

  if (!invalid_nodes.empty()) {
    LOG(ERROR) << "Nodes found with invalid attributes: "
               << displayNodeSymbolContainer(invalid_nodes);
    for (const auto& invalid_id : invalid_nodes) {
      active_nodes.erase(invalid_id);
      // TODO(nathan) think about this some more
      places.deleted_nodes.push_back(invalid_id);
    }
  }

  std::set<spark_dsg::EdgeKey> edges;
  for (const auto& node_id : active_nodes) {
    const auto& node = graph.getNode(node_id)->get();
    places.active_attributes.emplace(node_id, node.attributes().clone());
    for (const auto& sibling : node.siblings()) {
      spark_dsg::EdgeKey key(node_id, sibling);
      if (edges.count(key)) {
        continue;
      }

      edges.insert(key);
      places.edges.emplace_back(
          node_id, sibling, graph.getEdge(node_id, sibling)->get().info->clone());
    }
  }
  extractor.clearDeleted();

  VLOG(5) << "[Hydra Reconstruction] exporting " << active_nodes.size()
          << " active and " << removed_nodes.size() << " deleted nodes";
}

void ReconstructionModule::addMeshToOutput(ReconstructionOutput& output,
                                           const BlockIndexList& archived_blocks) {
  output.archived_blocks.insert(archived_blocks.begin(), archived_blocks.end());

  output.mesh.reset(new voxblox::MeshLayer(mesh_->block_size()));
  auto& active_mesh = *output.mesh;

  voxblox::BlockIndexList mesh_indices;
  mesh_->getAllUpdatedMeshes(&mesh_indices);
  for (const auto& block_index : mesh_indices) {
    if (output.archived_blocks.count(block_index)) {
      continue;
    }

    auto block = active_mesh.allocateNewBlock(block_index);
    *block = *(mesh_->getMeshPtrByIndex(block_index));
  }

  // TODO(nathan) maybe only do this for archived blocks?
  for (const auto& block_index : mesh_indices) {
    auto block = mesh_->getMeshPtrByIndex(block_index);
    if (!block->hasVertices()) {
      mesh_->removeMesh(block_index);
    }
  }
}

void ReconstructionModule::showStats() const {
  const std::string tsdf_memory_str =
      getHumanReadableMemoryString(tsdf_->getMemorySize());
  const std::string semantic_memory_str =
      getHumanReadableMemoryString(semantics_->getMemorySize());
  const std::string gvd_memory_str =
      getHumanReadableMemoryString(gvd_->getMemorySize());
  const std::string mesh_memory_str =
      getHumanReadableMemoryString(mesh_->getMemorySize());
  const size_t total =
      tsdf_->getMemorySize() + semantics_->getMemorySize() + gvd_->getMemorySize();
  LOG(INFO) << "Memory used: [TSDF=" << tsdf_memory_str
            << ", Semantics=" << semantic_memory_str << ", GVD=" << gvd_memory_str
            << ", Mesh= " << mesh_memory_str
            << ", Total=" << getHumanReadableMemoryString(total) << "]";
}

BlockIndexList ReconstructionModule::findBlocksToArchive(
    const voxblox::Point& center) const {
  BlockIndexList blocks;
  tsdf_->getAllAllocatedBlocks(&blocks);

  BlockIndexList to_archive;
  for (const auto& idx : blocks) {
    auto block = gvd_->getBlockPtrByIndex(idx);
    if ((center - block->origin()).norm() < config_.dense_representation_radius_m) {
      continue;
    }

    // TODO(nathan) filter by update flag?
    to_archive.push_back(idx);
  }

  return to_archive;
}

}  // namespace hydra
