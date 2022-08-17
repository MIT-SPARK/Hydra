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
#pragma once
#include "hydra_dsg_builder/frontend_config.h"
#include "hydra_dsg_builder/incremental_types.h"

#include <geometry_msgs/TransformStamped.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>

#include <hydra_msgs/ActiveLayer.h>
#include <hydra_msgs/ActiveMesh.h>
#include <hydra_topology/nearest_neighbor_utilities.h>
#include <kimera_pgmo/MeshFrontend.h>
#include <pose_graph_tools/PoseGraph.h>
#include <spark_dsg/scene_graph_logger.h>

#include <memory>
#include <mutex>

namespace hydra {
namespace incremental {

class MeshSegmenter;

using PlacesLayerMsg = hydra_msgs::ActiveLayer;
using topology::NearestNodeFinder;

struct PlacesQueueState {
  bool empty = true;
  uint64_t timestamp_ns = 0;
};

class DsgFrontend {
 public:
  DsgFrontend(const ros::NodeHandle& nh, const SharedDsgInfo::Ptr& dsg);

  virtual ~DsgFrontend();

  void start();

  void stop();

  pcl::PolygonMesh getFrontendMesh() const {
    pcl::PolygonMesh mesh;
    mesh.polygons = mesh_frontend_.getFullMeshFaces();

    const auto vertices = mesh_frontend_.getFullMeshVertices();
    pcl::toPCLPointCloud2(*vertices, mesh.cloud);
    return mesh;
  }

  std::vector<ros::Time> getFrontendMeshStamps() const {
    return mesh_frontend_.getFullMeshTimes();
  }

 private:
  void handleActivePlaces(const PlacesLayerMsg::ConstPtr& msg);

  void handleLatestMesh(const hydra_msgs::ActiveMesh::ConstPtr& msg);

  void handleLatestPoseGraph(const pose_graph_tools::PoseGraph::ConstPtr& msg);

  void startMeshFrontend();

  void runMeshFrontend();

  void startPlaces();

  void runPlaces();

  PlacesQueueState getPlacesQueueState();

  void processLatestPlacesMsg(const PlacesLayerMsg::ConstPtr& msg);

  void addPlaceObjectEdges(NodeIdSet* extra_objects_to_check = nullptr);

  void updatePlaceMeshMapping();

  void addAgentPlaceEdges();

  std::optional<Eigen::Vector3d> getLatestPose();

 private:
  ros::NodeHandle nh_;
  std::atomic<bool> should_shutdown_{false};

  DsgFrontendConfig config_;

  SharedDsgInfo::Ptr dsg_;
  kimera_pgmo::MeshFrontend mesh_frontend_;
  std::unique_ptr<MeshSegmenter> segmenter_;

  std::mutex mesh_frontend_mutex_;
  std::atomic<uint64_t> last_mesh_timestamp_;
  std::queue<hydra_msgs::ActiveMesh::ConstPtr> mesh_queue_;

  std::mutex places_queue_mutex_;
  std::atomic<uint64_t> last_places_timestamp_;
  std::queue<PlacesLayerMsg::ConstPtr> places_queue_;

  ros::Subscriber mesh_sub_;
  std::unique_ptr<ros::CallbackQueue> mesh_frontend_ros_queue_;
  std::unique_ptr<std::thread> mesh_frontend_thread_;
  tf2_ros::Buffer tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

  ros::Subscriber active_places_sub_;
  std::unique_ptr<NearestNodeFinder> places_nn_finder_;
  std::unique_ptr<std::thread> places_thread_;
  NodeIdSet unlabeled_place_nodes_;
  NodeIdSet previous_active_places_;

  std::set<NodeId> deleted_agent_edge_indices_;
  std::map<LayerPrefix, size_t> last_agent_edge_index_;

  char robot_prefix_;
  ros::Subscriber pose_graph_sub_;

  SceneGraphLogger frontend_graph_logger_;
};

}  // namespace incremental
}  // namespace hydra
