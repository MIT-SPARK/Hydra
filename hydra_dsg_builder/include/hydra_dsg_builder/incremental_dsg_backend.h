#pragma once
#include "hydra_dsg_builder/backend_config.h"
#include "hydra_dsg_builder/dsg_update_functions.h"
#include "hydra_dsg_builder/incremental_room_finder.h"
#include "hydra_dsg_builder/incremental_types.h"

#include <hydra_utils/dsg_streaming_interface.h>
#include <kimera_dsg/scene_graph_logger.h>
#include <kimera_pgmo/KimeraPgmoInterface.h>

#include <ros/callback_queue.h>
#include <ros/ros.h>

#include <map>
#include <memory>
#include <mutex>
#include <thread>

namespace hydra {
namespace incremental {

struct LoopClosureLog {
  gtsam::Symbol src;        // factor edge "source"
  gtsam::Symbol dest;       // factor edge "dest"
  gtsam::Pose3 src_T_dest;  // src_frame.between(dest_frame)
  bool dsg;
  int64_t level;
};

class DsgBackend : public kimera_pgmo::KimeraPgmoInterface {
 public:
  using Ptr = std::shared_ptr<DsgBackend>;
  using PoseGraphQueue = std::queue<pose_graph_tools::PoseGraph::ConstPtr>;

  DsgBackend(const ros::NodeHandle nh,
             const SharedDsgInfo::Ptr& dsg,
             const SharedDsgInfo::Ptr& backend_dsg);

  virtual ~DsgBackend();

  DsgBackend(const DsgBackend& other) = delete;

  DsgBackend& operator=(const DsgBackend& other) = delete;

  // TODO(nathan) consider deferring subscription and stuff here
  virtual bool initialize(const ros::NodeHandle&) override { return true; }

  virtual bool createPublishers(const ros::NodeHandle&) override { return true; }

  virtual bool registerCallbacks(const ros::NodeHandle&) override { return true; }

  void stop();

  void start();

  inline void setUpdateFuncs(const std::list<LayerUpdateFunc>& update_funcs) {
    dsg_update_funcs_ = update_funcs;
  }

  bool saveTrajectoryCallback(std_srvs::Empty::Request&, std_srvs::Empty::Response&);

  std::list<LoopClosureLog> getLoopClosures() {
    std::list<LoopClosureLog> to_return;
    {  // start pgmo critical section
      std::unique_lock<std::mutex> lock(pgmo_mutex_);
      to_return = loop_closures_;
    }  // end pgmo critical section
    return to_return;
  }

  void loadState(const std::string& state_path, const std::string& dgrf_path);

  void forceUpdate() {
    updateDsgMesh();
    callUpdateFunctions();
    private_dsg_->updated = true;
  }

  bool updatePrivateDsg();

  void updateDsgMesh(bool force_mesh_update = false);

  void optimize();

  void visualizePoseGraph() const;

  void visualizeDeformationGraphEdges() const;

  void startPgmo();

 protected:
  void setSolverParams();

  void fullMeshCallback(const kimera_pgmo::KimeraPgmoMesh::ConstPtr& msg);

  void deformationGraphCallback(const pose_graph_tools::PoseGraph::ConstPtr& msg);

  void poseGraphCallback(const pose_graph_tools::PoseGraph::ConstPtr& msg);

  bool saveMeshCallback(std_srvs::Empty::Request&, std_srvs::Empty::Response&);

  void runPgmo();

  void addPlacesToDeformationGraph();

  void callUpdateFunctions(const gtsam::Values& places_values = gtsam::Values(),
                           const gtsam::Values& pgmo_values = gtsam::Values());

  ActiveNodeSet getNodesForRoomDetection(const NodeIdSet& latest_places);

  void storeUnlabeledPlaces(const ActiveNodeSet active_nodes);

  void updateRoomsNodes();

  void updateBuildingNode();

  void logStatus(bool init = false) const;

  bool addInternalLCDToDeformationGraph();

  void logIncrementalLoopClosures(const pose_graph_tools::PoseGraph& msg);

  bool readPgmoUpdates();

  pose_graph_tools::PoseGraph::ConstPtr popDeformationGraphQueue();

  pose_graph_tools::PoseGraph::ConstPtr popAgentGraphQueue();

 protected:
  ros::NodeHandle nh_;
  std::atomic<bool> should_shutdown_{false};
  std::atomic<bool> have_loopclosures_{false};
  std::atomic<bool> have_graph_updates_{false};
  bool have_new_mesh_{false};

  DsgBackendConfig config_;

  SharedDsgInfo::Ptr shared_dsg_;
  SharedDsgInfo::Ptr private_dsg_;
  IsolatedSceneGraphLayer shared_places_copy_;

  std::atomic<uint64_t> last_timestamp_;

  ros::ServiceServer save_mesh_srv_;
  ros::ServiceServer save_traj_srv_;

  NodeIdSet unlabeled_place_nodes_;
  std::unique_ptr<RoomFinder> room_finder_;

  DsgBackendStatus status_;

  std::list<LayerUpdateFunc> dsg_update_funcs_;

  std::vector<int> mesh_vertex_graph_inds_;

  PoseGraphQueue deformation_graph_updates_;
  PoseGraphQueue pose_graph_updates_;

  std::mutex pgmo_mutex_;
  std::unique_ptr<std::thread> optimizer_thread_;

  ros::Publisher viz_mesh_mesh_edges_pub_;
  ros::Publisher viz_pose_mesh_edges_pub_;
  ros::Publisher pose_graph_pub_;
  ros::Publisher opt_mesh_pub_;

  SceneGraphLogger backend_graph_logger_;
  std::list<LoopClosureLog> loop_closures_;

 private:
  int robot_id_;
  char robot_prefix_;
  char robot_vertex_prefix_;

  kimera_pgmo::Path trajectory_;
  std::vector<ros::Time> timestamps_;
  std::queue<size_t> unconnected_nodes_;
  kimera_pgmo::KimeraPgmoMesh::ConstPtr latest_mesh_;
  std::vector<ros::Time> mesh_vertex_stamps_;

  ros::Subscriber full_mesh_sub_;
  ros::Subscriber deformation_graph_sub_;
  ros::Subscriber pose_graph_sub_;
  std::unique_ptr<hydra::DsgSender> dsg_sender_;
};

}  // namespace incremental
}  // namespace hydra
