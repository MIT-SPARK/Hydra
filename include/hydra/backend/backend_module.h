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
#include <config_utilities/factory.h>
#include <kimera_pgmo/KimeraPgmoInterface.h>
#include <spark_dsg/scene_graph_logger.h>

#include <map>
#include <memory>
#include <mutex>
#include <thread>

#include "hydra/backend/backend_config.h"
#include "hydra/backend/merge_handler.h"
#include "hydra/backend/update_functions.h"
#include "hydra/common/common.h"
#include "hydra/common/module.h"
#include "hydra/common/shared_dsg_info.h"
#include "hydra/common/shared_module_state.h"
#include "hydra/utils/log_utilities.h"

namespace spark_dsg {
class ZmqReceiver;
}  // namespace spark_dsg

namespace hydra {

struct LoopClosureLog {
  gtsam::Symbol src;        // factor edge "source"
  gtsam::Symbol dest;       // factor edge "dest"
  gtsam::Pose3 src_T_dest;  // src_frame.between(dest_frame)
  bool dsg;
  int64_t level;
};

class BackendModule : public kimera_pgmo::KimeraPgmoInterface, public Module {
 public:
  using Ptr = std::shared_ptr<BackendModule>;
  using OutputCallback = std::function<void(const DynamicSceneGraph&,
                                            const kimera_pgmo::DeformationGraph&,
                                            size_t timestamp_ns)>;

  BackendModule(const BackendConfig& config,
                const SharedDsgInfo::Ptr& dsg,
                const SharedDsgInfo::Ptr& backend_dsg,
                const SharedModuleState::Ptr& state,
                const LogSetup::Ptr& logs = nullptr);

  virtual ~BackendModule();

  BackendModule(const BackendModule& other) = delete;

  BackendModule& operator=(const BackendModule& other) = delete;

  void start() override;

  void stop() override;

  void save(const LogSetup& log_setup) override;

  std::string printInfo() const override;

  void spin();

  bool spinOnce(bool force_update = true);

  inline void triggerBackendDsgReset() { reset_backend_dsg_ = true; }

  // used by dsg_optimizer
  virtual void spinOnce(const BackendInput& input, bool force_update = true);

  void loadState(const std::string& state_path, const std::string& dgrf_path);

  inline void addOutputCallback(const OutputCallback& callback_func) {
    output_callbacks_.push_back(callback_func);
  }

  virtual bool initialize(const ros::NodeHandle&) override { return true; }

  virtual bool createPublishers(const ros::NodeHandle&) override { return true; }

  virtual bool registerCallbacks(const ros::NodeHandle&) override { return true; }

  using KimeraPgmoInterface::setVerboseFlag;

  // TODO(nathan) handle this better
  inline const BackendConfig& config() const { return config_; }

  void setUpdateFunctor(LayerId layer, const dsg_updates::UpdateFunctor::Ptr& functor);

 protected:
  void setUpdateFuncs();

  void logPlaceDistance();

  void setSolverParams();

  void addLoopClosure(const gtsam::Key& src,
                      const gtsam::Key& dest,
                      const gtsam::Pose3& src_T_dest,
                      double variance);

  virtual void setupDefaultFunctors();

  virtual void updateFactorGraph(const BackendInput& input);

  virtual bool updateFromLcdQueue();

  virtual void copyMeshDelta(const BackendInput& input);

  virtual bool updatePrivateDsg(size_t timestamp_ns, bool force_update = true);

  virtual void addPlacesToDeformationGraph(size_t timestamp_ns);

  virtual void updateAgentNodeMeasurements(
      const pose_graph_tools_msgs::PoseGraph& meas);

  virtual void optimize(size_t timestamp_ns);

  virtual void updateDsgMesh(size_t timestamp_ns, bool force_mesh_update = false);

  virtual void resetBackendDsg(size_t timestamp_ns);

  virtual void callUpdateFunctions(
      size_t timestamp_ns,
      const gtsam::Values& places_values = gtsam::Values(),
      const gtsam::Values& pgmo_values = gtsam::Values(),
      bool new_loop_closure = false,
      const std::map<LayerId, std::map<NodeId, NodeId>>& given_merges = {});

  virtual void updateObjectMapping(const kimera_pgmo::MeshDelta& delta);

  void runZmqUpdates();

  void updateMergedNodes(const std::map<NodeId, NodeId>& new_merges);

  void logStatus(bool init = false) const;

  void logIncrementalLoopClosures(const pose_graph_tools_msgs::PoseGraph& msg);

  void cachePlacePos();

  void updatePlacePosFromCache();

  void labelRooms(const UpdateInfo& info, SharedDsgInfo* dsg);

 protected:
  BackendConfig config_;

  std::unique_ptr<std::thread> spin_thread_;
  std::atomic<bool> should_shutdown_{false};
  bool have_loopclosures_{false};
  bool have_new_loopclosures_{false};
  bool have_new_mesh_{false};
  size_t prev_num_archived_vertices_{0};
  size_t num_archived_vertices_{0};
  bool reset_backend_dsg_{false};

  std::unordered_map<NodeId, Eigen::Vector3d> place_pos_cache_;

  SharedDsgInfo::Ptr shared_dsg_;
  SharedDsgInfo::Ptr private_dsg_;
  SceneGraphLayer::Ptr shared_places_copy_;
  std::unique_ptr<MergeHandler> merge_handler_;
  SharedModuleState::Ptr state_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr original_vertices_;
  std::vector<uint64_t> vertex_stamps_;

  std::list<LayerUpdateFunc> dsg_update_funcs_;
  std::list<LayerCleanupFunc> dsg_post_update_funcs_;
  std::map<LayerId, dsg_updates::UpdateFunctor::Ptr> layer_functors_;

  BackendModuleStatus status_;
  SceneGraphLogger backend_graph_logger_;
  LogSetup::Ptr logs_;
  std::list<LoopClosureLog> loop_closures_;

  kimera_pgmo::Path trajectory_;
  std::vector<size_t> timestamps_;
  std::queue<size_t> unconnected_nodes_;

  std::list<OutputCallback> output_callbacks_;

  std::map<NodeId, std::string> room_name_map_;
  std::unique_ptr<std::thread> zmq_thread_;
  std::unique_ptr<spark_dsg::ZmqReceiver> zmq_receiver_;

  // TODO(lschmid): This mutex currently simply locks all data for manipulation.
  std::mutex mutex_;

  inline static const auto registration_ =
      config::RegistrationWithConfig<BackendModule,
                                     BackendModule,
                                     BackendConfig,
                                     SharedDsgInfo::Ptr,
                                     SharedDsgInfo::Ptr,
                                     SharedModuleState::Ptr,
                                     LogSetup::Ptr>("BackendModule");
};

}  // namespace hydra
