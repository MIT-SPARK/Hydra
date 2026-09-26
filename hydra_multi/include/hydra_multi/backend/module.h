#pragma once
#include <config_utilities/factory.h>
#include <config_utilities/virtual_config.h>
#include <hydra/backend/dsg_updater.h>
#include <hydra/backend/external_loop_closure_receiver.h>
#include <hydra/backend/merge_tracker.h>
#include <hydra/backend/update_functions.h>
#include <hydra/common/output_sink.h>
#include <kimera_pgmo/kimera_pgmo_interface.h>
#include <spark_dsg/scene_graph_logger.h>

#include <map>
#include <memory>
#include <mutex>
#include <thread>
#include <unordered_map>

#include "hydra_multi/backend/initial_align.h"
#include "hydra_multi/common/multi_dsg_info.h"
#include "hydra_multi/common/types.h"
#include "hydra_multi/interface/interface_container.h"

namespace hydra_multi {

struct LoopClosureLog {
  gtsam::Symbol src;        // factor edge "source"
  gtsam::Symbol dest;       // factor edge "dest"
  gtsam::Pose3 src_T_dest;  // src_frame.between(dest_frame)
  bool dsg;
  int64_t level;
  bool added_to_dgraph = false;  // track if already added to deformation graph
};

struct MultiBackendModuleStatus {
  size_t new_loop_closures = 0;
  size_t total_loop_closures = 0;
  size_t total_interrobot_loop_closures = 0;
  size_t inlier_loop_closures = 0;
  size_t inlier_interrobot_loop_closures = 0;
  size_t total_factors = 0;
  size_t total_values = 0;
  size_t new_factors = 0;
  size_t new_graph_factors = 0;
  size_t trajectory_len = 0;
  size_t num_merges_undone = 0;
  std::optional<double> spin_s;
  std::optional<double> opt_s;
  std::optional<double> mesh_s;
  void reset();
};

std::ostream& operator<<(std::ostream& os, const MultiBackendModuleStatus& status);

class MultiBackendModule : public kimera_pgmo::KimeraPgmoInterface {
 public:
  using Ptr = std::shared_ptr<MultiBackendModule>;
  using Sink = hydra::OutputSink<uint64_t,
                                 const DynamicSceneGraph&,
                                 const kimera_pgmo::DeformationGraph&,
                                 const MultiBackendModuleStatus&>;
  using NodeToRobotMap = std::unordered_map<NodeId, size_t>;

  struct Config : hydra::DsgUpdater::Config {
    //! Actually perform PGO on every detected loop closure
    bool optimize_on_lc = true;
    //! Specialized PGMO configuration that includes scene graph factor covariances
    kimera_pgmo::KimeraPgmoConfig pgmo;
    //! Add prior to robot ID (negative disables anchor)
    int anchor_robot_id = 0;
    //! PGO versus PGMO
    bool include_mesh_factors = true;
    //! Configuration for associating external loop closures
    hydra::ExternalLoopClosureReceiver::Config external_loop_closures;
    //! Add object pose to deformation graph
    bool add_objects_to_deformation_graph = false;
    //! Object agent association max time diff (s) when adding to dgraph
    double object_association_max_diff_s = 10.0;
    //! Object agent association covariance
    double object_variance = 1.0;
    //! Scene graph loop closure variance
    double sg_loop_closure_variance = 0.1;
    //! Output sinks that process that latest backed scene graph and state
    std::vector<Sink::Factory> sinks;
    //! Initial alginment module
    config::VirtualConfig<InitialAlignModule> initial_align;
    //! Partial optimizer
    config::VirtualConfig<kimera_pgmo::Optimizer> partial_optimizer{
        kimera_pgmo::KimeraRpgoOptimizer::Config()};
  } const config;

  MultiBackendModule(const Config& config,
                     const InterfaceContainer::StatesPtr& shared_states);

  virtual ~MultiBackendModule();

  MultiBackendModule(const MultiBackendModule& other) = delete;

  MultiBackendModule& operator=(const MultiBackendModule& other) = delete;

  void start();

  void stop();

  void save(const DataDirectory& output);

  std::string printInfo() const;

  void addSink(const Sink::Ptr& sink);

  void addLoopClosures(const pose_graph_tools::PoseGraph& loop_closures);

  bool spinOnce(Timestamp timestamp_ns);

  bool optimizeOnce(Timestamp timestamp_ns);

 protected:
  void spin();

  void optimizeSpin();

  void stopImpl();

  bool pollSharedStates(Timestamp& stamp, bool mark_updated = true) const;

  bool updateFromStates(Timestamp timestamp_ns);

  void combineAndDeformMesh(Timestamp timestamp_ns, bool force_mesh_update = false);

  bool processLoopClosures(Timestamp timestamp_ns);

  void addLoopClosures();

  void initialAlignment();

  void addObjectsToDeformationGraph();

  void optimizeStartup() override;

  void optimizeCleanup(const gtsam::Values& results) override;

  void optimize(size_t timestamp_ns);

  void partialOptimization(const Timestamp& timestamp_ns);

  gtsam::Pose3 getRobotWorldFrame(RobotId robot_id, gtsam::Values& optimized);

  void rebaseStates(size_t timestamps_ns);

  void logStatus();

 protected:
  std::unique_ptr<std::thread> spin_thread_;
  std::unique_ptr<std::thread> optimization_thread_;

  std::atomic<bool> should_shutdown_{false};
  std::atomic<bool> should_deform_{false};
  bool have_loopclosures_ = false;
  bool have_new_mesh_ = false;
  uint64_t last_sequence_number_ = 0;

  struct GuardedValues {
    std::mutex mutex;
    gtsam::Values values;
  } optimized_;

  std::mutex status_mutex_;
  MultiBackendModuleStatus status_;
  std::vector<MultiBackendModuleStatus> status_log_;
  SceneGraphLogger backend_graph_logger_;

  std::list<LoopClosureLog> loop_closures_;
  hydra::MessageQueue<pose_graph_tools::PoseGraph> lc_queue_;
  hydra::MessageQueue<LoopClosureLog> interrobot_lc_queue_;
  hydra::ExternalLoopClosureReceiver lc_receiver_;

  std::map<pose_graph_tools::PoseGraphEdge::Type, double> edge_variance_map_;

  Sink::List sinks_;

  MultiDsgInfo::Ptr unmerged_dsg_;
  MultiDsgInfo::Ptr merged_dsg_;
  InterfaceContainer::StatesPtr shared_states_;
  std::map<size_t, MeshData::Ptr> staged_mesh_data_;
  std::map<size_t, gtsam::Pose3> curr_W_T_robot_;

  InitialAlignModule::Ptr initial_align_;
  hydra::DsgUpdater::Ptr dsg_updater_;
  kimera_pgmo::Optimizer::Ptr partial_pgo_;
};

void declare_config(MultiBackendModule::Config& config);

}  // namespace hydra_multi
