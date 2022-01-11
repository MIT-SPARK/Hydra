#pragma once
#include "kimera_dsg_builder/dsg_lcd_module.h"
#include "kimera_dsg_builder/incremental_mesh_segmenter.h"
#include "kimera_dsg_builder/incremental_types.h"

#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>

#include <kimera_dsg/node_symbol.h>
#include <kimera_dsg/scene_graph_logger.h>
#include <kimera_pgmo/MeshFrontend.h>
#include <kimera_topology/ActiveLayer.h>
#include <kimera_topology/ActiveMesh.h>
#include <kimera_topology/nearest_neighbor_utilities.h>
#include <kimera_vio_ros/BowQuery.h>
#include <pose_graph_tools/PoseGraph.h>

#include <memory>
#include <mutex>

namespace kimera {
namespace incremental {

using PlacesLayerMsg = kimera_topology::ActiveLayer;
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

  void saveState(const std::string& filepath) const;

 private:
  void handleActivePlaces(const PlacesLayerMsg::ConstPtr& msg);

  void handleLatestMesh(const kimera_topology::ActiveMesh::ConstPtr& msg);

  void handleLatestPoseGraph(const pose_graph_tools::PoseGraph::ConstPtr& msg);

  void handleDbowMsg(const kimera_vio_ros::BowQuery::ConstPtr& msg);

  void startMeshFrontend();

  void runMeshFrontend();

  void startPlaces();

  void runPlaces();

  void startLcd();

  void runLcd();

  PlacesQueueState getPlacesQueueState();

  void processLatestPlacesMsg(const PlacesLayerMsg::ConstPtr& msg);

  void addPlaceObjectEdges(NodeIdSet* extra_objects_to_check = nullptr);

  void updatePlaceMeshMapping();

  void addAgentPlaceEdges();

  lcd::DsgLcdConfig initializeLcdStructures();

  void assignBowVectors();

  std::optional<NodeId> getLatestAgentId();

  std::optional<Eigen::Vector3d> getLatestPose();

 private:
  ros::NodeHandle nh_;
  std::atomic<bool> should_shutdown_{false};

  SharedDsgInfo::Ptr dsg_;
  kimera_pgmo::MeshFrontend mesh_frontend_;
  std::unique_ptr<MeshSegmenter> segmenter_;

  std::mutex mesh_frontend_mutex_;
  std::atomic<uint64_t> last_mesh_timestamp_;
  std::queue<kimera_topology::ActiveMesh::ConstPtr> mesh_queue_;
  size_t mesh_queue_size_;

  std::mutex places_queue_mutex_;
  std::atomic<uint64_t> last_places_timestamp_;
  std::queue<PlacesLayerMsg::ConstPtr> places_queue_;

  ros::Subscriber mesh_sub_;
  std::unique_ptr<ros::CallbackQueue> mesh_frontend_ros_queue_;
  std::unique_ptr<std::thread> mesh_frontend_thread_;
  size_t min_object_size_;
  bool prune_mesh_indices_;
  std::string sensor_frame_;
  tf2_ros::Buffer tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

  ros::Subscriber active_places_sub_;
  std::unique_ptr<NearestNodeFinder> places_nn_finder_;
  std::unique_ptr<std::thread> places_thread_;
  NodeIdSet unlabeled_place_nodes_;
  NodeIdSet archived_places_;
  NodeIdSet previous_active_places_;
  kimera_pgmo::VoxbloxIndexMapping latest_mesh_mappings_;

  std::map<char, std::set<NodeId>> deleted_agent_edge_indices_;
  std::map<char, size_t> last_agent_edge_index_;

  SemanticNodeAttributes::ColorVector building_color_;

  double lcd_agent_horizon_s_;
  std::atomic<bool> lcd_shutting_down_{false};
  std::priority_queue<NodeId, std::vector<NodeId>, std::greater<NodeId>> lcd_queue_;
  std::unique_ptr<std::thread> lcd_thread_;
  std::unique_ptr<lcd::DsgLcdModule> lcd_module_;
  std::unique_ptr<lcd::ObjectDescriptorFactory> object_lcd_factory_;
  std::unique_ptr<lcd::PlaceDescriptorFactory> place_lcd_factory_;
  std::unique_ptr<lcd::ObjectRegistrationFunctor> object_lcd_registration_;
  std::unique_ptr<lcd::PlaceRegistrationFunctor> places_lcd_registration_;
  DynamicSceneGraph::Ptr lcd_graph_;
  char robot_prefix_;

  ros::Subscriber bow_sub_;
  ros::Subscriber pose_graph_sub_;
  std::list<kimera_vio_ros::BowQuery::ConstPtr> bow_messages_;
  std::map<NodeId, size_t> agent_key_map_;

  bool log_;
  std::string log_path_;
  SceneGraphLogger frontend_graph_logger_;
};

}  // namespace incremental
}  // namespace kimera
