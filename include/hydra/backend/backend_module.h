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
#include <kimera_pgmo/kimera_pgmo_interface.h>
#include <spark_dsg/scene_graph_logger.h>

#include <map>
#include <memory>
#include <mutex>
#include <thread>

#include "hydra/backend/merge_tracker.h"
#include "hydra/backend/pgmo_configs.h"
#include "hydra/backend/update_frontiers_functor.h"
#include "hydra/backend/update_surface_places_functor.h"
#include "hydra/common/common.h"
#include "hydra/common/module.h"
#include "hydra/common/output_sink.h"
#include "hydra/common/shared_dsg_info.h"
#include "hydra/common/shared_module_state.h"
#include "hydra/rooms/room_finder_config.h"
#include "hydra/utils/log_utilities.h"

namespace spark_dsg {
class ZmqReceiver;
class ZmqSender;
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
  using Sink = OutputSink<uint64_t,
                          const DynamicSceneGraph&,
                          const kimera_pgmo::DeformationGraph&>;

  struct Config {
    bool visualize_place_factors = true;
    bool enable_rooms = true;
    RoomFinderConfig room_finder;
    bool enable_buildings = true;
    Color building_color = Color(169, 8, 194);  // purple
    SemanticNodeAttributes::Label building_semantic_label = 22u;
    HydraPgmoConfig pgmo;
    // dsg
    bool add_places_to_deformation_graph = true;
    bool optimize_on_lc = true;
    bool enable_node_merging = true;
    bool use_mesh_subscribers = false;
    mutable std::map<LayerId, bool> merge_update_map{{DsgLayers::OBJECTS, false},
                                                     {DsgLayers::PLACES, true},
                                                     {DsgLayers::ROOMS, false},
                                                     {DsgLayers::BUILDINGS, false}};
    bool merge_update_dynamic = true;
    double places_merge_pos_threshold_m = 0.4;
    double places_merge_distance_tolerance_m = 0.3;
    bool enable_merge_undos = false;
    bool use_active_flag_for_updates = true;
    size_t num_neighbors_to_find_for_merge = 1;
    std::string zmq_send_url = "tcp://127.0.0.1:8001";
    std::string zmq_recv_url = "tcp://127.0.0.1:8002";
    bool use_zmq_interface = false;
    size_t zmq_num_threads = 2;
    size_t zmq_poll_time_ms = 10;
    bool zmq_send_mesh = true;
    bool use_2d_places = false;
    Update2dPlacesFunctor::Config places2d_config;
    UpdateFrontiersFunctor::Config frontier_config;
    std::vector<Sink::Factory> sinks;
  } const config;

  BackendModule(const Config& config,
                const SharedDsgInfo::Ptr& dsg,
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

  void addSink(const Sink::Ptr& sink);

  void setUpdateFunctor(LayerId layer, const UpdateFunctor::Ptr& functor);

 protected:
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

  virtual void updateAgentNodeMeasurements(const pose_graph_tools::PoseGraph& meas);

  virtual void optimize(size_t timestamp_ns);

  virtual void updateDsgMesh(size_t timestamp_ns, bool force_mesh_update = false);

  virtual void resetBackendDsg(size_t timestamp_ns);

  virtual void callUpdateFunctions(size_t timestamp_ns,
                                   const gtsam::Values& places_values = gtsam::Values(),
                                   const gtsam::Values& pgmo_values = gtsam::Values(),
                                   bool new_loop_closure = false,
                                   const UpdateInfo::LayerMerges& given_merges = {});

  void runZmqUpdates();

  void updateMergedNodes(const std::map<NodeId, NodeId>& new_merges);

  void logStatus(bool init = false) const;

  void logIncrementalLoopClosures(const pose_graph_tools::PoseGraph& graph);

  void labelRooms(const UpdateInfo& info, SharedDsgInfo* dsg);

 protected:
  void stopImpl();

  std::unique_ptr<std::thread> spin_thread_;
  std::atomic<bool> should_shutdown_{false};
  bool have_loopclosures_{false};
  bool have_new_loopclosures_{false};
  bool have_new_mesh_{false};
  size_t prev_num_archived_vertices_{0};
  size_t num_archived_vertices_{0};
  bool reset_backend_dsg_{false};

  std::unordered_map<NodeId, Eigen::Vector3d> place_pos_cache_;

  SharedDsgInfo::Ptr private_dsg_;
  DynamicSceneGraph::Ptr unmerged_graph_;
  SharedModuleState::Ptr state_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr original_vertices_;
  std::vector<uint64_t> vertex_stamps_;

  MergeTracker merge_tracker;
  std::map<LayerId, UpdateFunctor::Ptr> layer_functors_;
  UpdateFunctor::Ptr agent_functor_;

  BackendModuleStatus status_;
  SceneGraphLogger backend_graph_logger_;
  LogSetup::Ptr logs_;
  std::list<LoopClosureLog> loop_closures_;

  kimera_pgmo::Path trajectory_;
  std::vector<size_t> timestamps_;
  std::queue<size_t> unconnected_nodes_;

  Sink::List sinks_;

  std::map<NodeId, std::string> room_name_map_;
  std::unique_ptr<std::thread> zmq_thread_;
  std::unique_ptr<spark_dsg::ZmqReceiver> zmq_receiver_;
  std::unique_ptr<spark_dsg::ZmqSender> zmq_sender_;

  // TODO(lschmid): This mutex currently simply locks all data for manipulation.
  std::mutex mutex_;

  inline static const auto registration_ =
      config::RegistrationWithConfig<BackendModule,
                                     BackendModule,
                                     Config,
                                     SharedDsgInfo::Ptr,
                                     SharedModuleState::Ptr,
                                     LogSetup::Ptr>("BackendModule");
};

void declare_config(BackendModule::Config& conf);

}  // namespace hydra
