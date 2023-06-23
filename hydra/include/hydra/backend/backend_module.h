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
#include <kimera_pgmo/KimeraPgmoInterface.h>
#include <spark_dsg/scene_graph_logger.h>
#include <spark_dsg/zmq_interface.h>

#include <map>
#include <memory>
#include <thread>

#include "hydra/backend/backend_config.h"
#include "hydra/backend/merge_handler.h"
#include "hydra/backend/update_functions.h"
#include "hydra/common/common.h"
#include "hydra/common/robot_prefix_config.h"
#include "hydra/common/shared_module_state.h"

namespace hydra {

struct LoopClosureLog {
  gtsam::Symbol src;        // factor edge "source"
  gtsam::Symbol dest;       // factor edge "dest"
  gtsam::Pose3 src_T_dest;  // src_frame.between(dest_frame)
  bool dsg;
  int64_t level;
};

class BackendModule : public kimera_pgmo::KimeraPgmoInterface {
 public:
  using Ptr = std::shared_ptr<BackendModule>;
  using OutputCallback = std::function<void(const DynamicSceneGraph&,
                                            const kimera_pgmo::DeformationGraph&,
                                            size_t timestamp_ns)>;

  BackendModule(const RobotPrefixConfig& prefix,
                const BackendConfig& config,
                const kimera_pgmo::KimeraPgmoConfig& pgmo_config,
                const SharedDsgInfo::Ptr& dsg,
                const SharedDsgInfo::Ptr& backend_dsg,
                const SharedModuleState::Ptr& state);

  virtual ~BackendModule();

  BackendModule(const BackendModule& other) = delete;

  BackendModule& operator=(const BackendModule& other) = delete;

  void start();

  void stop();

  void save(const std::string& output_path);

  void spin();

  bool spinOnce(bool force_update = true);

  inline void triggerBackendDsgReset() { reset_backend_dsg_ = true; }

  // used by dsg_optimizer
  void spinOnce(const BackendInput& input, bool force_update = true);

  void loadState(const std::string& state_path, const std::string& dgrf_path);

  void setUpdateFuncs(const std::list<LayerUpdateFunc>& update_funcs);

  inline void addOutputCallback(const OutputCallback& callback_func) {
    output_callbacks_.push_back(callback_func);
  }

  virtual bool initialize(const ros::NodeHandle&) override { return true; }

  virtual bool createPublishers(const ros::NodeHandle&) override { return true; }

  virtual bool registerCallbacks(const ros::NodeHandle&) override { return true; }

  using KimeraPgmoInterface::setVerboseFlag;

 protected:
  void setSolverParams();

  void setDefaultUpdateFunctions();

  void addLoopClosure(const gtsam::Key& src,
                      const gtsam::Key& dest,
                      const gtsam::Pose3& src_T_dest,
                      double variance);

  virtual void updateFactorGraph(const BackendInput& input);

  virtual bool updateFromLcdQueue();

  virtual void copyMeshDelta(const BackendInput& input);

  virtual bool updatePrivateDsg(size_t timestamp_ns, bool force_update = true);

  virtual void addPlacesToDeformationGraph(size_t timestamp_ns);

  virtual void optimize(size_t timestamp_ns);

  virtual void updateDsgMesh(size_t timestamp_ns, bool force_mesh_update = false);

  virtual void resetBackendDsg(size_t timestamp_ns);

  virtual void callUpdateFunctions(
      size_t timestamp_ns,
      const gtsam::Values& places_values = gtsam::Values(),
      const gtsam::Values& pgmo_values = gtsam::Values(),
      bool new_loop_closure = false,
      const std::map<LayerId, std::map<NodeId, NodeId>>& given_merges = {});

  void runZmqUpdates();

  void updateMergedNodes(const std::map<NodeId, NodeId>& new_merges);

  void logStatus(bool init = false) const;

  void logIncrementalLoopClosures(const pose_graph_tools::PoseGraph& msg);

  void cachePlacePos();

  void updatePlacePosFromCache();

 protected:
  std::unique_ptr<std::thread> spin_thread_;
  std::atomic<bool> should_shutdown_{false};
  bool have_loopclosures_{false};
  bool have_new_loopclosures_{false};
  bool have_new_mesh_{false};
  size_t prev_num_archived_vertices_{0};
  size_t num_archived_vertices_{0};
  bool reset_backend_dsg_{false};

  RobotPrefixConfig prefix_;
  BackendConfig config_;

  std::unordered_map<NodeId, Eigen::Vector3d> place_pos_cache_;

  SharedDsgInfo::Ptr shared_dsg_;
  SharedDsgInfo::Ptr private_dsg_;
  IsolatedSceneGraphLayer shared_places_copy_;
  std::map<LayerId, NodeMergeLog> proposed_node_merges_;
  std::unique_ptr<MergeHandler> merge_handler_;
  SharedModuleState::Ptr state_;
  std::vector<ros::Time> mesh_timestamps_;
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr original_vertices_;

  std::list<LayerUpdateFunc> dsg_update_funcs_;
  std::shared_ptr<dsg_updates::UpdateObjectsFunctor> update_objects_functor_;
  std::shared_ptr<dsg_updates::UpdatePlacesFunctor> update_places_functor_;
  std::unique_ptr<dsg_updates::UpdateRoomsFunctor> update_rooms_functor_;
  std::unique_ptr<dsg_updates::UpdateBuildingsFunctor> update_buildings_functor_;

  BackendModuleStatus status_;
  SceneGraphLogger backend_graph_logger_;
  std::list<LoopClosureLog> loop_closures_;

  kimera_pgmo::Path trajectory_;
  std::vector<ros::Time> timestamps_;
  std::queue<size_t> unconnected_nodes_;

  std::list<OutputCallback> output_callbacks_;

  std::map<NodeId, std::string> room_name_map_;
  std::unique_ptr<std::thread> zmq_thread_;
  std::unique_ptr<spark_dsg::ZmqReceiver> zmq_receiver_;
};

}  // namespace hydra
