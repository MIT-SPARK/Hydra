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
#include "hydra_dsg_builder/backend_config.h"
#include "hydra_dsg_builder/dsg_update_functions.h"
#include "hydra_dsg_builder/incremental_room_finder.h"
#include "hydra_dsg_builder/incremental_types.h"

#include <hydra_utils/dsg_streaming_interface.h>
#include <kimera_pgmo/KimeraPgmoInterface.h>
#include <spark_dsg/scene_graph_logger.h>

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

  void updateMergedNodes(const std::map<NodeId, NodeId>& new_merges);

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
  std::map<NodeId, NodeId> merged_nodes_;
  std::map<NodeId, std::set<NodeId>> merged_nodes_parents_;

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
