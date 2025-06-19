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
#include <config_utilities/virtual_config.h>
#include <kimera_pgmo/kimera_pgmo_interface.h>
#include <spark_dsg/scene_graph_logger.h>

#include <filesystem>
#include <map>
#include <memory>
#include <mutex>
#include <thread>

#include "hydra/backend/backend_input.h"
#include "hydra/backend/dsg_updater.h"
#include "hydra/backend/external_loop_closure_receiver.h"
#include "hydra/backend/merge_tracker.h"
#include "hydra/backend/pgmo_configs.h"
#include "hydra/common/module.h"
#include "hydra/common/output_sink.h"
#include "hydra/common/shared_dsg_info.h"
#include "hydra/common/shared_module_state.h"

namespace hydra {

struct LoopClosureLog {
  gtsam::Symbol src;        // factor edge "source"
  gtsam::Symbol dest;       // factor edge "dest"
  gtsam::Pose3 src_T_dest;  // src_frame.between(dest_frame)
  bool dsg;
  int64_t level;
};

struct BackendModuleStatus {
  size_t total_loop_closures = 0;
  size_t new_loop_closures = 0;
  size_t total_factors = 0;
  size_t total_values = 0;
  size_t new_factors = 0;
  size_t new_graph_factors = 0;
  size_t trajectory_len = 0;
  size_t num_merges_undone = 0;
  std::optional<double> last_spin_s = 0.0;
  std::optional<double> last_opt_s = 0.0;
  std::optional<double> last_mesh_update_s = 0.0;
};

class BackendModule : public kimera_pgmo::KimeraPgmoInterface, public Module {
 public:
  using Ptr = std::shared_ptr<BackendModule>;
  using Sink = OutputSink<uint64_t,
                          const DynamicSceneGraph&,
                          const kimera_pgmo::DeformationGraph&>;

  struct Config : DsgUpdater::Config {
    //! Specialized PGMO configuration that includes scene graph factor covariances
    HydraPgmoConfig pgmo;
    //! Add places layer to factor graph via MST approach
    bool add_places_to_deformation_graph = true;
    //! Optimize
    bool optimize_on_lc = true;
    ExternalLoopClosureReceiver::Config external_loop_closures;
    //! Output sinks that process that latest backed scene graph and state
    std::vector<Sink::Factory> sinks;
  } const config;

  BackendModule(const Config& config,
                const SharedDsgInfo::Ptr& dsg,
                const SharedModuleState::Ptr& state);

  virtual ~BackendModule();

  BackendModule(const BackendModule& other) = delete;

  BackendModule& operator=(const BackendModule& other) = delete;

  void start() override;

  void stop() override;

  void save(const DataDirectory& output) override;

  std::string printInfo() const override;

  void spin();

  bool step(bool force_optimize = false);

  void loadState(const std::filesystem::path& mesh_path,
                 const std::filesystem::path& dgrf_path,
                 bool force_loopclosures = true);

  void addSink(const Sink::Ptr& sink);

 protected:
  virtual bool spinOnce(bool force_update = true);

  void addLoopClosure(const gtsam::Key& src,
                      const gtsam::Key& dest,
                      const gtsam::Pose3& src_T_dest,
                      double variance);

  void updateFactorGraph(const BackendInput& input);

  bool updateFromLcdQueue();

  void copyMeshDelta(const BackendInput& input);

  bool updatePrivateDsg(size_t timestamp_ns, bool force_update = true);

  void updateAgentNodeMeasurements(const pose_graph_tools::PoseGraph& meas);

  void optimize(size_t timestamp_ns, bool force_find_merge = false);

  void updateDsgMesh(size_t timestamp_ns, bool force_mesh_update = false);

  void logIncrementalLoopClosures(const pose_graph_tools::PoseGraph& graph);

  void logStatus();

 protected:
  void stopImpl();

  std::unique_ptr<std::thread> spin_thread_;
  std::atomic<bool> should_shutdown_{false};
  bool force_optimize_ = false;
  bool have_loopclosures_ = false;
  bool have_new_loopclosures_ = false;
  bool have_new_mesh_ = false;
  uint64_t last_sequence_number_ = 0;

  SharedDsgInfo::Ptr private_dsg_;
  DynamicSceneGraph::Ptr unmerged_graph_;
  SharedModuleState::Ptr state_;

  DsgUpdater::Ptr dsg_updater_;

  kimera_pgmo::Path trajectory_;
  std::vector<size_t> timestamps_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr original_vertices_;
  std::shared_ptr<std::vector<uint64_t>> vertex_stamps_;
  size_t prev_num_archived_vertices_ = 0;
  size_t num_archived_vertices_ = 0;

  std::vector<BackendModuleStatus> status_log_;
  SceneGraphLogger backend_graph_logger_;
  std::list<LoopClosureLog> loop_closures_;
  ExternalLoopClosureReceiver external_lc_receiver_;

  Sink::List sinks_;

  // TODO(lschmid): This mutex currently simply locks all data for manipulation.
  std::mutex mutex_;
};

void declare_config(BackendModule::Config& conf);

}  // namespace hydra
