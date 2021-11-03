#pragma once
#include "kimera_dsg_builder/dsg_update_functions.h"
#include "kimera_dsg_builder/incremental_room_finder.h"
#include "kimera_dsg_builder/incremental_types.h"

#include <kimera_dsg_visualizer/dynamic_scene_graph_visualizer.h>
#include <kimera_pgmo/KimeraPgmoInterface.h>

#include <ros/callback_queue.h>
#include <ros/ros.h>

#include <map>
#include <memory>
#include <mutex>
#include <thread>

namespace kimera {
namespace incremental {

template <typename T>
void parseParam(const ros::NodeHandle& nh, const std::string& name, T& param) {
  int value = param;
  nh.getParam(name, value);
  param = value;
}

class DsgBackend : public kimera_pgmo::KimeraPgmoInterface {
 public:
  using Ptr = std::shared_ptr<DsgBackend>;

  DsgBackend(const ros::NodeHandle nh,
             const SharedDsgInfo::Ptr& dsg,
             const SharedDsgInfo::Ptr& backend_dsg);

  ~DsgBackend();

  DsgBackend(const DsgBackend& other) = delete;

  DsgBackend& operator=(const DsgBackend& other) = delete;

  // TODO(nathan) consider deferring subscription and stuff here
  virtual bool initialize(const ros::NodeHandle&) override { return true; }

  virtual bool createPublishers(const ros::NodeHandle&) override { return true; }

  virtual bool registerCallbacks(const ros::NodeHandle&) override { return true; }

  void start();

  inline void setUpdateFuncs(const std::list<LayerUpdateFunc>& update_funcs) {
    dsg_update_funcs_ = update_funcs;
  }

 private:
  void fullMeshCallback(const kimera_pgmo::TriangleMeshIdStamped::ConstPtr& msg);

  void deformationGraphCallback(const pose_graph_tools::PoseGraph::ConstPtr& msg);

  void poseGraphCallback(const pose_graph_tools::PoseGraph::ConstPtr& msg);

  void startVisualizer();

  void runVisualizer();

  void startPgmo();

  void runPgmo();

  void addNewAgentPoses();

  void addPlacesToDeformationGraph();

  void updateDsgMesh();

  void updatePlaceMeshMapping();

  void optimize();

  void callUpdateFunctions(const gtsam::Values& places_values = gtsam::Values(),
                           const gtsam::Values& pgmo_values = gtsam::Values());

  ActiveNodeSet getNodesForRoomDetection(const NodeIdSet& latest_places);

  void storeUnlabeledPlaces(const ActiveNodeSet active_nodes);

  void updateRoomsNodes();

  void updateBuildingNode();

  void logStatus(bool init = false) const;

 private:
  ros::NodeHandle nh_;
  std::atomic<bool> should_shutdown_{false};

  SharedDsgInfo::Ptr shared_dsg_;
  SharedDsgInfo::Ptr private_dsg_;

  int robot_id_;
  bool add_places_to_deformation_graph_;
  bool optimize_on_lc_;
  bool enable_node_merging_;
  bool call_update_periodically_;

  char robot_prefix_;
  char robot_vertex_prefix_;
  kimera_pgmo::Path trajectory_;
  std::vector<ros::Time> timestamps_;
  std::queue<size_t> unconnected_nodes_;
  NodeIdSet unlabeled_place_nodes_;

  ros::Subscriber full_mesh_sub_;
  ros::Subscriber deformation_graph_sub_;
  ros::Subscriber pose_graph_sub_;

  std::unique_ptr<DynamicSceneGraphVisualizer> visualizer_;
  std::unique_ptr<ros::CallbackQueue> visualizer_queue_;
  std::unique_ptr<std::thread> visualizer_thread_;
  std::unique_ptr<RoomFinder> room_finder_;

  std::list<LayerUpdateFunc> dsg_update_funcs_;

  bool have_new_mesh_;
  bool have_new_poses_;
  bool have_new_deformation_graph_;

  SemanticNodeAttributes::ColorVector building_color_;

  kimera_pgmo::TriangleMeshIdStamped::ConstPtr latest_mesh_;
  pose_graph_tools::PoseGraph::Ptr deformation_graph_updates_;
  pose_graph_tools::PoseGraph::Ptr pose_graph_updates_;

  std::mutex pgmo_mutex_;
  std::unique_ptr<std::thread> pgmo_thread_;

  bool log_;
  std::string log_path_;
  DsgBackendStatus status_;
};

}  // namespace incremental
}  // namespace kimera
