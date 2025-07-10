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
#include "hydra/backend/backend_module.h"

#include <config_utilities/config.h>
#include <config_utilities/printing.h>
#include <config_utilities/validation.h>
#include <glog/logging.h>
#include <glog/stl_logging.h>
#include <kimera_pgmo/utils/mesh_io.h>

#include "hydra/backend/backend_utilities.h"
#include "hydra/backend/mst_factors.h"
#include "hydra/common/global_info.h"
#include "hydra/common/launch_callbacks.h"
#include "hydra/common/pipeline_queues.h"
#include "hydra/utils/minimum_spanning_tree.h"
#include "hydra/utils/pgmo_mesh_traits.h"
#include "hydra/utils/timing_utilities.h"

namespace hydra {

using hydra::timing::ScopedTimer;
using kimera_pgmo::KimeraPgmoInterface;
using pose_graph_tools::PoseGraph;

namespace {

static const auto registration =
    config::RegistrationWithConfig<BackendModule,
                                   BackendModule,
                                   BackendModule::Config,
                                   SharedDsgInfo::Ptr,
                                   SharedModuleState::Ptr>("BackendModule");

void writeBackendStatus(const std::vector<BackendModuleStatus>& entries,
                        const std::filesystem::path& filename) {
  std::ofstream file(filename, std::ofstream::out);
  // file format
  file << "total_lc,new_lc,total_factors,total_values,new_factors,new_graph_"
          "factors,trajectory_len,run_time,optimize_time,mesh_update_time,num_"
          "merges_"
          "undone\n";

  const auto nan = std::numeric_limits<double>::quiet_NaN();
  for (const auto& entry : entries) {
    file << entry.total_loop_closures << "," << entry.new_loop_closures << ","
         << entry.total_factors << "," << entry.total_values << "," << entry.new_factors
         << "," << entry.new_graph_factors << "," << entry.trajectory_len << ","
         << entry.last_spin_s.value_or(nan) << "," << entry.last_opt_s.value_or(nan)
         << "," << entry.last_mesh_update_s.value_or(nan) << entry.num_merges_undone
         << "\n";
  }

  file.close();
}

inline bool hasLoopClosure(const PoseGraph& graph) {
  for (const auto& edge : graph.edges) {
    if (edge.type == pose_graph_tools::PoseGraphEdge::Type::LOOPCLOSE) {
      return true;
    }
  }
  return false;
}

}  // namespace

void declare_config(BackendModule::Config& config) {
  using namespace config;
  base<DsgUpdater::Config>(config);
  name("BackendConfig");
  field(config.pgmo, "pgmo");
  field(config.add_places_to_deformation_graph, "add_places_to_deformation_graph");
  field(config.optimize_on_lc, "optimize_on_lc");
  field(config.external_loop_closures, "external_loop_closures");
  field(config.sinks, "sinks");
}

BackendModule::BackendModule(const Config& config,
                             const SharedDsgInfo::Ptr& dsg,
                             const SharedModuleState::Ptr& state)
    : KimeraPgmoInterface(config.pgmo),
      config(config::checkValid(config)),
      private_dsg_(dsg),
      state_(state),
      external_lc_receiver_(config.external_loop_closures,
                            &PipelineQueues::instance().external_loop_closure_queue) {
  // set up frontend graph copy
  unmerged_graph_ = private_dsg_->graph->clone();
  // set up mesh infrastructure
  private_dsg_->graph->setMesh(std::make_shared<spark_dsg::Mesh>());
  unmerged_graph_->setMesh(private_dsg_->graph->mesh());
  original_vertices_.reset(
      new pcl::PointCloud<pcl::PointXYZ>());  // set up frontend graph copy
  vertex_stamps_.reset(new std::vector<uint64_t>());

  dsg_updater_.reset(new DsgUpdater(config, unmerged_graph_, private_dsg_));
}

BackendModule::~BackendModule() { stopImpl(); }

void BackendModule::start() {
  spin_thread_.reset(new std::thread(&BackendModule::spin, this));
  LOG(INFO) << "[Hydra Backend] started!";
}

void BackendModule::stopImpl() {
  should_shutdown_ = true;

  if (spin_thread_) {
    VLOG(2) << "[Hydra Backend] joining optimizer thread and stopping";
    spin_thread_->join();
    spin_thread_.reset();
    VLOG(2) << "[Hydra Backend] stopped!";
  }
}

void BackendModule::stop() { stopImpl(); }

void BackendModule::save(const DataDirectory& output) {
  std::lock_guard<std::mutex> lock(mutex_);
  dsg_updater_->save(output, "backend");

  const auto backend_path = output.path("backend");
  backend_graph_logger_.save(backend_path);

  const auto filename = output.path("backend/pgmo") / "dsg_pgmo_status.csv";
  writeBackendStatus(status_log_, filename);

  deformation_graph_->save(backend_path / "deformation_graph.dgrf");
  const auto& prefix = GlobalInfo::instance().getRobotPrefix();
  if (deformation_graph_->hasPrefixPoses(prefix.key)) {
    const auto optimized_path = getOptimizedTrajectory(prefix.id);
    std::string csv_name = backend_path / "trajectory.csv";
    saveTrajectory(optimized_path, timestamps_, csv_name);
  }

  const std::string output_csv = backend_path / "loop_closures.csv";
  std::ofstream output_file;
  output_file.open(output_csv);

  output_file << "time_from_ns,time_to_ns,x,y,z,qw,qx,qy,qz,type,level" << std::endl;
  for (const auto& loop_closure : loop_closures_) {
    // pose = src.between(dest) or to_T_from
    auto time_from = utils::getTimeNs(*private_dsg_->graph, loop_closure.dest);
    auto time_to = utils::getTimeNs(*private_dsg_->graph, loop_closure.src);
    if (!time_from || !time_to) {
      continue;
    }

    const gtsam::Point3 pos = loop_closure.src_T_dest.translation();
    const gtsam::Quaternion quat = loop_closure.src_T_dest.rotation().toQuaternion();

    output_file << *time_from << "," << *time_to << ",";
    output_file << pos.x() << "," << pos.y() << "," << pos.z() << ",";
    output_file << quat.w() << ", " << quat.x() << "," << quat.y() << "," << quat.z()
                << ",";
    output_file << (loop_closure.dsg ? 1 : 0) << "," << loop_closure.level;
    output_file << std::endl;
  }
}

std::string BackendModule::printInfo() const {
  return config::toString(config) + "\n" + Sink::printSinks(sinks_);
}

void BackendModule::spin() {
  bool should_shutdown = false;
  while (!should_shutdown) {
    auto& queue = PipelineQueues::instance().backend_queue;
    bool has_data = queue.poll();
    if (GlobalInfo::instance().force_shutdown() || !has_data) {
      // copy over shutdown request
      should_shutdown = should_shutdown_;
    }

    if (!has_data) {
      continue;
    }

    spinOnce(false);
  }
}

bool BackendModule::step(bool force_optimize) {
  const bool prev_force_optimize = force_optimize_;
  force_optimize_ |= force_optimize;
  const auto updated = spinOnce(true);
  force_optimize_ = prev_force_optimize;
  return updated;
}

bool BackendModule::spinOnce(bool force_update) {
  auto& queue = PipelineQueues::instance().backend_queue;
  bool has_data = queue.poll();
  if (!has_data && !force_update) {
    return false;
  }

  uint64_t timestamp_ns = 0;
  status_log_.emplace_back(BackendModuleStatus{});

  std::lock_guard<std::mutex> lock(mutex_);
  if (has_data) {
    const auto packet = queue.front();
    queue.pop();
    const auto& input = *packet;
    timestamp_ns = input.timestamp_ns;
    last_sequence_number_ = input.sequence_number;

    updateFactorGraph(input);
    copyMeshDelta(input);
  }

  ScopedTimer timer("backend/update", timestamp_ns);
  updateFromLcdQueue();
  status_log_.back().total_loop_closures = num_loop_closures_;

  if (!updatePrivateDsg(timestamp_ns, force_update)) {
    VLOG(2) << "Backend skipping input @ " << timestamp_ns << " [ns]";
    // we only read from the frontend dsg if we've processed all the
    // factor graph update packets (as long as force_update is false)
    // we still log the status for each received frontend packet
    logStatus();
    return true;
  }

  timer.reset("backend/spin");
  if ((config.optimize_on_lc && have_loopclosures_) || force_optimize_) {
    optimize(timestamp_ns);
  } else {
    updateDsgMesh(timestamp_ns);
    UpdateInfo::ConstPtr info(new UpdateInfo{timestamp_ns});
    dsg_updater_->callUpdateFunctions(timestamp_ns, info);
  }

  logStatus();

  ScopedTimer sink_timer("backend/sinks", timestamp_ns);
  Sink::callAll(sinks_, timestamp_ns, *private_dsg_->graph, *deformation_graph_);
  return true;
}

void BackendModule::loadState(const std::filesystem::path& mesh_path,
                              const std::filesystem::path& dgrf_path,
                              bool force_loopclosures) {
  spark_dsg::Mesh::Ptr mesh;
  if (mesh_path.extension() == ".sparkdsg" || mesh_path.extension() == ".json") {
    auto graph = DynamicSceneGraph::load(mesh_path);
    if (!graph) {
      LOG(ERROR) << "Invalid graph path: " << mesh_path;
      return;
    }

    if (!graph->hasMesh()) {
      LOG(WARNING) << "Invalid mesh path: " << mesh_path << ", graph has no mesh!";
    }

    mesh = graph->mesh();
  } else {
    kimera_pgmo::ReadMesh(mesh_path, *mesh);
  }

  if (mesh) {
    LOG(ERROR) << "Loaded mesh with " << mesh->numVertices() << " vertices and "
               << mesh->numFaces() << " faces";
  } else {
    LOG(ERROR) << "Failed to load mesh...";
  }

  private_dsg_->graph->setMesh(mesh);
  unmerged_graph_->setMesh(mesh);
  have_new_mesh_ = true;
  have_loopclosures_ = force_loopclosures;

  loadDeformationGraphFromFile(dgrf_path);
  LOG(WARNING) << "Loaded " << deformation_graph_->getNumVertices()
               << " vertices for deformation graph";
}

void BackendModule::addSink(const Sink::Ptr& sink) {
  if (sink) {
    sinks_.push_back(sink);
  }
}

void BackendModule::updateFactorGraph(const BackendInput& input) {
  ScopedTimer timer("backend/process_factors", input.timestamp_ns);
  const size_t prev_loop_closures = num_loop_closures_;
  status_log_.back().new_graph_factors = input.deformation_graph.edges.size();
  status_log_.back().new_factors += input.deformation_graph.edges.size();

  std::vector<size_t> inc_mesh_indices;
  std::vector<uint64_t> inc_mesh_index_stamps;

  if (!input.deformation_graph.nodes.empty() &&
      !input.deformation_graph.edges.empty()) {
    try {
      processIncrementalMeshGraph(input.deformation_graph,
                                  timestamps_,
                                  inc_mesh_indices,
                                  inc_mesh_index_stamps);
    } catch (const gtsam::ValuesKeyDoesNotExist& e) {
      LOG(ERROR) << input.deformation_graph;
      throw std::logic_error(e.what());
    }
  } else {
    VLOG(10) << "[Hydra Backend] Dropping empty deformation graph @ "
             << input.timestamp_ns << " [ns]";
  }

  for (const auto& msg : input.agent_updates.pose_graphs) {
    status_log_.back().new_factors += msg.edges.size();

    VLOG(5) << "[Hydra Backend] Adding pose graph message: " << msg;
    if (hasLoopClosure(msg)) {
      LOG(INFO) << "[Hydra Backend] Input pose graph has loop closure @ "
                << msg.stamp_ns << " [ns]";
    }
    processIncrementalPoseGraph(
        msg, inc_mesh_indices, inc_mesh_index_stamps, trajectory_, timestamps_);
    logIncrementalLoopClosures(msg);
  }

  if (input.agent_updates.external_priors) {
    updateAgentNodeMeasurements(*input.agent_updates.external_priors);
    // Think of it as "implicit" loop closures
    have_loopclosures_ = true;
    have_new_loopclosures_ = true;
  }

  external_lc_receiver_.update(
      *unmerged_graph_,
      [this](NodeId to_node, NodeId from_node, const gtsam::Pose3 to_T_from) {
        LoopClosureLog lc{to_node, from_node, to_T_from, true, 1};
        addLoopClosure(
            lc.src, lc.dest, lc.src_T_dest, (KimeraPgmoInterface::config_.lc_variance));
        loop_closures_.push_back(lc);
        ++num_loop_closures_;
      });

  if (num_loop_closures_ > prev_loop_closures) {
    LOG(WARNING) << "New loop closures detected!";
    have_new_loopclosures_ = true;
  }

  if (num_loop_closures_ > 0) {
    status_log_.back().new_loop_closures = num_loop_closures_ - prev_loop_closures;
    have_loopclosures_ = true;
  }

  status_log_.back().trajectory_len = trajectory_.size();
  status_log_.back().total_factors = deformation_graph_->getFactors()->size();
  status_log_.back().total_values = deformation_graph_->getValues()->size();
}

bool BackendModule::updateFromLcdQueue() {
  bool added_new_loop_closure = false;
  auto& queue = PipelineQueues::instance().backend_lcd_queue;
  while (!queue.empty()) {
    const auto result = queue.pop();
    // note that pose graph convention is pose = src.between(dest) where the edge
    // connects frames "to -> from" (i.e. src = to, dest = from, pose = to_T_from)
    const gtsam::Pose3 to_T_from(gtsam::Rot3(result.to_R_from), result.to_p_from);
    LoopClosureLog lc{result.to_node, result.from_node, to_T_from, true, result.level};
    addLoopClosure(lc.src,
                   lc.dest,
                   lc.src_T_dest,
                   (result.level ? KimeraPgmoInterface::config_.lc_variance
                                 : config.pgmo.sg_loop_closure_variance));

    loop_closures_.push_back(lc);

    added_new_loop_closure = true;
    have_loopclosures_ = true;
    have_new_loopclosures_ = true;
    num_loop_closures_++;
    status_log_.back().new_loop_closures++;
  }

  return added_new_loop_closure;
}

void BackendModule::copyMeshDelta(const BackendInput& input) {
  ScopedTimer timer("backend/copy_mesh_delta", input.timestamp_ns);
  if (!input.mesh_update) {
    LOG(WARNING) << "[Hydra Backend] invalid mesh update!";
    return;
  }

  // TODO(nathan) this is ugly, but no good way to know at backend init whether
  // we're tracking first-seen stamps or not (because that's private to the active
  // window map)
  if (input.mesh_stamp_update && !private_dsg_->graph->mesh()->numVertices()) {
    const_cast<bool&>(private_dsg_->graph->mesh()->has_first_seen_stamps) = true;
  }

  {
    std::lock_guard<std::mutex> graph_lock(private_dsg_->mutex);

    input.mesh_update->updateMesh(*private_dsg_->graph->mesh());
    if (input.mesh_stamp_update) {
      input.mesh_stamp_update->updateMesh(*private_dsg_->graph->mesh(),
                                          input.mesh_update->vertex_start);
    }

    kimera_pgmo::StampedCloud<pcl::PointXYZ> cloud_out{*original_vertices_,
                                                       *vertex_stamps_};
    input.mesh_update->updateVertices(cloud_out);
    // we use this to make sure that deformation only happens for vertices that are
    // still active
    num_archived_vertices_ = input.mesh_update->getTotalArchivedVertices();
    utils::updatePlaces2d(private_dsg_, *input.mesh_update, num_archived_vertices_);
  }
  have_new_mesh_ = true;
}

bool BackendModule::updatePrivateDsg(size_t timestamp_ns, bool force_update) {
  std::lock_guard<std::mutex> graph_lock(private_dsg_->mutex);
  {  // start joint critical section
    ScopedTimer timer("backend/read_graph", timestamp_ns);

    const auto& shared_dsg = *state_->backend_graph;
    std::lock_guard<std::mutex> shared_graph_lock(shared_dsg.mutex);
    if (!force_update && shared_dsg.sequence_number != last_sequence_number_) {
      return false;
    }

    unmerged_graph_->mergeGraph(*shared_dsg.graph);
  }  // end joint critical section

  backend_graph_logger_.logGraph(*private_dsg_->graph);
  return true;
}

void BackendModule::addLoopClosure(const gtsam::Key& src,
                                   const gtsam::Key& dest,
                                   const gtsam::Pose3& src_T_dest,
                                   double variance) {
  deformation_graph_->processNewBetween(src, dest, src_T_dest, variance);
}

void BackendModule::updateDsgMesh(size_t timestamp_ns, bool force_mesh_update) {
  if (!force_mesh_update && !have_new_mesh_) {
    return;
  }

  have_new_mesh_ = false;

  auto mesh = private_dsg_->graph->mesh();
  if (!mesh || mesh->empty()) {
    return;
  }

  if (!force_mesh_update && !have_loopclosures_) {
    // we don't need to deform the mesh if we haven't found any loop closures yet
    // note that the first time we get a loop closure, we will deform the entire mesh
    // and cache the number of archived vertices and their values then
    return;
  }

  ScopedTimer timer("backend/mesh_deformation", timestamp_ns);

  VLOG(2) << "Deforming mesh with " << mesh->numVertices() << " vertices";

  kimera_pgmo::ConstStampedCloud<pcl::PointXYZ> cloud_in{*original_vertices_,
                                                         *vertex_stamps_};
  deformation_graph_->deformPoints(*private_dsg_->graph->mesh(),
                                   cloud_in,
                                   GlobalInfo::instance().getRobotPrefix().vertex_key,
                                   *deformation_graph_->getValues(),
                                   KimeraPgmoInterface::config_.num_interp_pts,
                                   KimeraPgmoInterface::config_.interp_horizon,
                                   nullptr,
                                   prev_num_archived_vertices_);
  prev_num_archived_vertices_ = num_archived_vertices_;
}

void BackendModule::updateAgentNodeMeasurements(const PoseGraph& meas) {
  deformation_graph_->removePriorsWithPrefix(
      GlobalInfo::instance().getRobotPrefix().key);
  std::vector<std::pair<gtsam::Key, gtsam::Pose3>> agent_measurements;
  for (const auto& node : meas.nodes) {
    agent_measurements.push_back(
        {gtsam::Symbol(GlobalInfo::instance().getRobotPrefix().key, node.key),
         gtsam::Pose3(node.pose.matrix())});
  }
  deformation_graph_->processNodeMeasurements(agent_measurements);
}

void BackendModule::optimize(size_t timestamp_ns, bool force_find_merge) {
  if (config.add_places_to_deformation_graph) {
    const auto vertex_key = GlobalInfo::instance().getRobotPrefix().vertex_key;
    addPlacesToDeformationGraph(*unmerged_graph_,
                                timestamp_ns,
                                *deformation_graph_,
                                config.pgmo.place_edge_variance,
                                config.pgmo.place_mesh_variance,
                                [vertex_key](auto) { return vertex_key; });
  }

  {  // timer scope
    ScopedTimer timer("dsg_updater/optimization", timestamp_ns, true, 0, false);
    KimeraPgmoInterface::optimize();
  }  // timer scope

  updateDsgMesh(timestamp_ns);

  UpdateInfo::ConstPtr info(new UpdateInfo{timestamp_ns,
                                           config.add_places_to_deformation_graph
                                               ? deformation_graph_->getTempValues()
                                               : nullptr,
                                           deformation_graph_->getValues(),
                                           have_new_loopclosures_ || force_find_merge,
                                           {},
                                           deformation_graph_.get(),
                                           nullptr,
                                           num_archived_vertices_,
                                           prev_num_archived_vertices_});
  dsg_updater_->callUpdateFunctions(timestamp_ns, info);
  have_new_loopclosures_ = false;
}

void BackendModule::logIncrementalLoopClosures(const PoseGraph& msg) {
  for (const auto& edge : msg.edges) {
    if (edge.type != pose_graph_tools::PoseGraphEdge::LOOPCLOSE) {
      continue;
    }

    const auto& prefix = GlobalInfo::instance().getRobotPrefix();
    const gtsam::Pose3 pose(edge.pose.matrix());
    const gtsam::Symbol src_key(prefix.key, edge.key_from);
    const gtsam::Symbol dest_key(prefix.key, edge.key_to);
    // note that pose graph convention is pose = src.between(dest) where the edge
    // connects frames "to -> from" (i.e. src = to, dest = from, pose = to_T_from)
    loop_closures_.push_back({src_key, dest_key, pose, false, 0});
  }
}

void BackendModule::logStatus() {
  if (status_log_.empty()) {
    return;
  }

  auto& status = status_log_.back();
  const auto& timer = hydra::timing::ElapsedTimeRecorder::instance();
  status.last_spin_s = timer.getLastElapsed("backend/spin");
  status.last_opt_s = timer.getLastElapsed("backend/optimization");
  status.last_mesh_update_s = timer.getLastElapsed("backend/mesh_update");
}

}  // namespace hydra
