#pragma once
#include "kimera_dsg_builder/incremental_mesh_segmenter.h"
#include "kimera_dsg_builder/incremental_room_finder.h"
#include "kimera_dsg_builder/incremental_types.h"

#include <ros/callback_queue.h>
#include <ros/ros.h>

#include <kimera_dsg/node_symbol.h>
#include <kimera_pgmo/MeshFrontend.h>
#include <kimera_topology/ActiveLayer.h>
#include <kimera_topology/nearest_neighbor_utilities.h>

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
  using NodeIdSet = std::unordered_set<NodeId>;

  DsgFrontend(const ros::NodeHandle& nh, const SharedDsgInfo::Ptr& dsg);

  virtual ~DsgFrontend();

  void start();

 private:
  void handleActivePlaces(const PlacesLayerMsg::ConstPtr& msg);

  void handleLatestMesh(const voxblox_msgs::Mesh::ConstPtr& msg);

  void startMeshFrontend();

  void runMeshFrontend();

  void startPlaces();

  void runPlaces();

  PlacesQueueState getPlacesQueueState();

  NodeIdSet processLatestPlacesMsg(const PlacesLayerMsg::ConstPtr& msg);

  ActiveNodeSet getNodesForRoomDetection(const NodeIdSet& latest_places);

  void storeUnlabeledPlaces(const ActiveNodeSet active_nodes);

  void addPlaceObjectEdges(NodeIdSet* extra_objects_to_check = nullptr);

  void updateBuildingNode();

 private:
  ros::NodeHandle nh_;
  std::atomic<bool> should_shutdown_{false};

  SharedDsgInfo::Ptr dsg_;
  kimera_pgmo::MeshFrontend mesh_frontend_;
  std::unique_ptr<MeshSegmenter> segmenter_;

  std::mutex mesh_frontend_mutex_;
  std::atomic<uint64_t> last_mesh_timestamp_;
  voxblox_msgs::Mesh::ConstPtr latest_mesh_msg_;

  std::mutex places_queue_mutex_;
  std::atomic<uint64_t> last_places_timestamp_;
  std::queue<PlacesLayerMsg::ConstPtr> places_queue_;

  ros::Subscriber mesh_sub_;
  std::unique_ptr<ros::CallbackQueue> mesh_frontend_ros_queue_;
  std::unique_ptr<std::thread> mesh_frontend_thread_;

  ros::Subscriber active_places_sub_;
  std::unique_ptr<NearestNodeFinder> places_nn_finder_;
  std::unique_ptr<RoomFinder> room_finder_;
  std::unique_ptr<std::thread> places_thread_;
  NodeIdSet unlabeled_place_nodes_;

  SemanticNodeAttributes::ColorVector building_color_;
};

}  // namespace incremental
}  // namespace kimera
