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
namespace {

static const auto registration =
    config::RegistrationWithConfig<BackendModule,
                                   BackendModule,
                                   BackendModule::Config,
                                   SharedDsgInfo::Ptr,
                                   SharedModuleState::Ptr,
                                   LogSetup::Ptr>("BackendModule");

}

using hydra::timing::ScopedTimer;
using kimera_pgmo::KimeraPgmoInterface;
using pose_graph_tools::PoseGraph;

void BackendModuleStatus::reset() {
  total_loop_closures = 0;
  new_loop_closures = 0;
  total_factors = 0;
  total_values = 0;
  new_factors = 0;
  new_graph_factors = 0;
  trajectory_len = 0;
  num_merges_undone = 0;
}

inline bool hasLoopClosure(const PoseGraph& graph) {
  for (const auto& edge : graph.edges) {
    if (edge.type == pose_graph_tools::PoseGraphEdge::Type::LOOPCLOSE) {
      return true;
    }
  }
  return false;
}

void declare_config(BackendModule::Config& config) {
  using namespace config;
  name("BackendConfig");
  field(config.pgmo, "pgmo");
  field(config.add_places_to_deformation_graph, "add_places_to_deformation_graph");
  field(config.optimize_on_lc, "optimize_on_lc");
  field(config.enable_node_merging, "enable_node_merging");
  field(config.enable_exhaustive_merging, "enable_exhaustive_merging");
  field(config.max_external_loop_closure_time_difference,
        "max_external_loop_closure_time_difference",
        "s");
  field(config.update_functors, "update_functors");
  field(config.sinks, "sinks");

  check(config.max_external_loop_closure_time_difference,
        GE,
        0.0,
        "max_external_loop_closure_time_difference");
}

BackendModule::BackendModule(const Config& config,
                             const SharedDsgInfo::Ptr& dsg,
                             const SharedModuleState::Ptr& state,
                             const LogSetup::Ptr& logs)
    : KimeraPgmoInterface(),
      config(config::checkValid(config)),
      private_dsg_(dsg),
      state_(state) {
  if (!KimeraPgmoInterface::initialize(config.pgmo)) {
    throw std::runtime_error("invalid pgmo config");
  }

  setSolverParams();

  // set up frontend graph copy
  unmerged_graph_ = private_dsg_->graph->clone();
  // set up mesh infrastructure
  private_dsg_->graph->setMesh(std::make_shared<spark_dsg::Mesh>());
  unmerged_graph_->setMesh(private_dsg_->graph->mesh());
  original_vertices_.reset(new pcl::PointCloud<pcl::PointXYZ>());
  deformation_graph_->setForceRecalculate(!config.pgmo.gnc_fix_prev_inliers);
  setSolverParams();

  for (const auto& [name, functor] : config.update_functors) {
    update_functors_.push_back(functor.create());
  }

  if (logs && logs->valid()) {
    logs_ = logs;
    const auto log_path = logs->getLogDir("backend");
    backend_graph_logger_.setOutputPath(log_path);
    VLOG(1) << "[Hydra Backend] logging to " << log_path;
    backend_graph_logger_.setLayerName(DsgLayers::OBJECTS, "objects");
    backend_graph_logger_.setLayerName(DsgLayers::PLACES, "places");
    backend_graph_logger_.setLayerName(DsgLayers::ROOMS, "rooms");
    backend_graph_logger_.setLayerName(DsgLayers::BUILDINGS, "buildings");
  } else {
    VLOG(1) << "[Hydra Backend] logging disabled.";
  }
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

void BackendModule::save(const LogSetup& log_setup) {
  std::lock_guard<std::mutex> lock(mutex_);
  const auto backend_path = log_setup.getLogDir("backend");
  const auto pgmo_path = log_setup.getLogDir("backend/pgmo");
  private_dsg_->graph->save(backend_path + "/dsg.json", false);
  private_dsg_->graph->save(backend_path + "/dsg_with_mesh.json");
  savePoseGraphSparseMapping(pgmo_path + "/sparsification_mapping.txt");

  const auto& prefix = GlobalInfo::instance().getRobotPrefix();
  if (deformation_graph_->hasPrefixPoses(prefix.key)) {
    const auto optimized_path = getOptimizedTrajectory(prefix.id);
    std::string csv_name = pgmo_path + "/traj_pgmo.csv";
    saveTrajectory(optimized_path, timestamps_, csv_name);
  }

  const auto mesh = private_dsg_->graph->mesh();
  if (mesh && !mesh->empty()) {
    // mesh implements vertex and face traits
    kimera_pgmo::WriteMesh(backend_path + "/mesh.ply", *mesh, *mesh);
  }

  deformation_graph_->update();  // Update before saving
  deformation_graph_->save(pgmo_path + "/deformation_graph.dgrf");

  const std::string output_csv = backend_path + "/loop_closures.csv";
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

  status_.reset();
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
  status_.total_loop_closures = num_loop_closures_;

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
    callUpdateFunctions(timestamp_ns);
  }

  if (logs_) {
    logStatus();
  }

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

void BackendModule::setSolverParams() {
  KimeraRPGO::RobustSolverParams params = deformation_graph_->getParams();
  params.verbosity = config.pgmo.rpgo_verbosity;
  params.solver = config.pgmo.rpgo_solver;
  if (logs_) {
    params.logOutput(logs_->getLogDir("backend/pgmo"));
    logStatus(true);
  }
  deformation_graph_->setParams(params);
  setVerboseFlag(false);
}

void BackendModule::updateFactorGraph(const BackendInput& input) {
  ScopedTimer timer("backend/process_factors", input.timestamp_ns);
  const size_t prev_loop_closures = num_loop_closures_;
  status_.new_graph_factors = input.deformation_graph.edges.size();
  status_.new_factors += input.deformation_graph.edges.size();

  if (!input.deformation_graph.nodes.empty() &&
      !input.deformation_graph.edges.empty()) {
    try {
      processIncrementalMeshGraph(
          input.deformation_graph, timestamps_, unconnected_nodes_);
    } catch (const gtsam::ValuesKeyDoesNotExist& e) {
      LOG(ERROR) << input.deformation_graph;
      throw std::logic_error(e.what());
    }
  } else {
    VLOG(10) << "[Hydra Backend] Dropping empty deformation graph @ "
             << input.timestamp_ns << " [ns]";
  }

  for (const auto& msg : input.agent_updates.pose_graphs) {
    status_.new_factors += msg.edges.size();

    VLOG(5) << "[Hydra Backend] Adding pose graph message: " << msg;
    if (hasLoopClosure(msg)) {
      LOG(INFO) << "[Hydra Backend] Input pose graph has loop closure @ "
                << msg.stamp_ns << " [ns]";
    }
    processIncrementalPoseGraph(msg, trajectory_, timestamps_, unconnected_nodes_);
    logIncrementalLoopClosures(msg);
  }

  if (input.agent_updates.external_priors) {
    updateAgentNodeMeasurements(*input.agent_updates.external_priors);
    // Think of it as "implicit" loop closures
    have_loopclosures_ = true;
    have_new_loopclosures_ = true;
  }

  updateExternalLoopClosures();

  if (num_loop_closures_ > prev_loop_closures) {
    LOG(WARNING) << "New loop closures detected!";
    have_new_loopclosures_ = true;
  }

  if (num_loop_closures_ > 0) {
    status_.new_loop_closures = num_loop_closures_ - prev_loop_closures;
    have_loopclosures_ = true;
  }

  status_.trajectory_len = trajectory_.size();
  status_.total_factors = deformation_graph_->getGtsamFactors().size();
  status_.total_values = deformation_graph_->getGtsamValues().size();
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
    status_.new_loop_closures++;
  }

  return added_new_loop_closure;
}

void BackendModule::copyMeshDelta(const BackendInput& input) {
  ScopedTimer timer("backend/copy_mesh_delta", input.timestamp_ns);
  if (!input.mesh_update) {
    LOG(WARNING) << "[Hydra Backend] invalid mesh update!";
    return;
  }

  input.mesh_update->updateMesh(*private_dsg_->graph->mesh());
  kimera_pgmo::StampedCloud<pcl::PointXYZ> cloud_out{*original_vertices_,
                                                     vertex_stamps_};
  input.mesh_update->updateVertices(cloud_out);
  // we use this to make sure that deformation only happens for vertices that are
  // still active
  num_archived_vertices_ = input.mesh_update->getTotalArchivedVertices();
  utils::updatePlaces2d(private_dsg_, *input.mesh_update, num_archived_vertices_);

  have_new_mesh_ = true;
}

bool BackendModule::updatePrivateDsg(size_t timestamp_ns, bool force_update) {
  std::unique_lock<std::mutex> graph_lock(private_dsg_->mutex);
  {  // start joint critical section
    ScopedTimer timer("backend/read_graph", timestamp_ns);

    const auto& shared_dsg = *state_->backend_graph;
    std::unique_lock<std::mutex> shared_graph_lock(shared_dsg.mutex);
    if (!force_update && shared_dsg.sequence_number != last_sequence_number_) {
      return false;
    }

    unmerged_graph_->mergeGraph(*shared_dsg.graph);
  }  // end joint critical section

  if (logs_) {
    backend_graph_logger_.logGraph(*private_dsg_->graph);
  }

  return true;
}

void BackendModule::addLoopClosure(const gtsam::Key& src,
                                   const gtsam::Key& dest,
                                   const gtsam::Pose3& src_T_dest,
                                   double variance) {
  if (full_sparse_frame_map_.size() == 0 ||
      !KimeraPgmoInterface::config_.b_enable_sparsify) {
    deformation_graph_->processNewBetween(src, dest, src_T_dest, variance);
  } else {
    if (!full_sparse_frame_map_.count(src) || !full_sparse_frame_map_.count(dest)) {
      // TODO(yun) this happened a few times when loop closure found for node that has
      // not yet been received
      LOG(ERROR)
          << "Attempted to add loop closure with node not yet processed by PGMO.\n";
      return;
    }
    gtsam::Key sparse_src = full_sparse_frame_map_.at(src);
    gtsam::Key sparse_dest = full_sparse_frame_map_.at(dest);
    gtsam::Pose3 sparse_src_T_sparse_dest =
        sparse_frames_.at(sparse_src).keyed_transforms.at(src) * src_T_dest *
        sparse_frames_.at(sparse_dest).keyed_transforms.at(dest).inverse();
    deformation_graph_->processNewBetween(
        sparse_src, sparse_dest, sparse_src_T_sparse_dest, variance);
  }
}

void BackendModule::updateDsgMesh(size_t timestamp_ns, bool force_mesh_update) {
  deformation_graph_->update();
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
                                                         vertex_stamps_};
  deformation_graph_->deformPoints(*private_dsg_->graph->mesh(),
                                   cloud_in,
                                   GlobalInfo::instance().getRobotPrefix().vertex_key,
                                   deformation_graph_->getGtsamValues(),
                                   KimeraPgmoInterface::config_.num_interp_pts,
                                   KimeraPgmoInterface::config_.interp_horizon,
                                   nullptr,
                                   prev_num_archived_vertices_);
  prev_num_archived_vertices_ = num_archived_vertices_;
}

std::optional<NodeId> BackendModule::findClosestAgentId(uint64_t stamp_ns,
                                                        std::optional<int> robot_id,
                                                        double max_diff_s) const {
  const auto prefix = robot_id ? RobotPrefixConfig(robot_id.value()).key
                               : GlobalInfo::instance().getRobotPrefix().key;
  const auto& layer = unmerged_graph_->getLayer(DsgLayers::AGENTS, prefix);
  const auto stamp = std::chrono::nanoseconds(stamp_ns);
  auto closest =
      std::min_element(layer.nodes().begin(),
                       layer.nodes().end(),
                       [stamp](const auto& lhs, const auto& rhs) {
                         const auto diff_lhs = lhs->timestamp.value() - stamp;
                         const auto diff_rhs = rhs->timestamp.value() - stamp;
                         return std::abs(diff_lhs.count()) < std::abs(diff_rhs.count());
                       });
  if (closest == layer.nodes().end()) {
    LOG(ERROR) << "[Hydra Backend] No nodes exist for agent layer '" << prefix
               << "' when looking up timestamp " << stamp_ns << " [ns]";
    return std::nullopt;
  }

  const auto& best_node = *closest;
  // NOTE(hlim) It's heuristic, but occasionally,
  // during Hydra's mesh generation process, a delay can occur,
  // causing the most recently stored node in `timestamps_` to be mistakenly
  // identified as the closest node. This needs to be rejected.
  const auto diff_s = std::chrono::duration_cast<std::chrono::duration<double>>(
                          best_node->timestamp.value() - stamp)
                          .count();
  VLOG(5) << "[Hydra Backend] Found node " << NodeSymbol(best_node->id).getLabel()
          << " with difference of " << diff_s << " [s] for timestamp " << stamp_ns
          << " [ns]";
  if (max_diff_s && std::abs(diff_s) >= max_diff_s) {
    LOG(WARNING) << "[Hydra Backend] Nearest node "
                 << NodeSymbol(best_node->id).getLabel()
                 << " has too large of a time difference: " << diff_s
                 << " [s] >= " << max_diff_s << " [s]";
    return std::nullopt;
  }

  return best_node->id;
}

void BackendModule::updateExternalLoopClosures() {
  auto& queue = PipelineQueues::instance().external_loop_closure_queue;
  while (!queue.empty()) {
    const auto lcd_message = queue.pop();
    for (const auto& edge : lcd_message.edges) {
      // NOTE(hlim) the pose graph edge IDs contain the loop closure timestamps, not the
      // keyframe IDs
      const auto max_diff_s = config.max_external_loop_closure_time_difference;
      const auto from_node =
          findClosestAgentId(edge.key_from, std::nullopt, max_diff_s);
      const auto to_node = findClosestAgentId(edge.key_to, std::nullopt, max_diff_s);
      if (!from_node || !to_node) {
        LOG(WARNING) << "[Hydra Backend] dropped external loop closure: " << edge;
        continue;
      }

      const gtsam::Pose3 to_T_from(edge.pose.matrix());
      LoopClosureLog lc = LoopClosureLog{*to_node, *from_node, to_T_from, true, 1};

      addLoopClosure(
          lc.src, lc.dest, lc.src_T_dest, (KimeraPgmoInterface::config_.lc_variance));
      loop_closures_.push_back(lc);

      have_loopclosures_ = true;
      have_new_loopclosures_ = true;
      num_loop_closures_++;
    }
  }
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

void BackendModule::optimize(size_t timestamp_ns) {
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
    ScopedTimer timer("backend/optimization", timestamp_ns, true, 0, false);
    deformation_graph_->optimize();
  }  // timer scope

  updateDsgMesh(timestamp_ns, true);

  callUpdateFunctions(timestamp_ns,
                      deformation_graph_->getGtsamTempValues(),
                      deformation_graph_->getGtsamValues(),
                      have_new_loopclosures_);
  have_new_loopclosures_ = false;
}

void BackendModule::resetBackendDsg(size_t timestamp_ns) {
  ScopedTimer timer("backend/reset_dsg", timestamp_ns, true, 0, false);
  {
    std::unique_lock<std::mutex> graph_lock(private_dsg_->mutex);
    // First reset private graph
    private_dsg_->graph->clear();
  }

  // TODO(nathan) this might break mesh stuff
  private_dsg_->graph->mergeGraph(*unmerged_graph_);
  private_dsg_->merges.clear();
  merge_tracker.clear();
  deformation_graph_->setRecalculateVertices();
  reset_backend_dsg_ = false;
}

void BackendModule::callUpdateFunctions(size_t timestamp_ns,
                                        const gtsam::Values& places_values,
                                        const gtsam::Values& pgmo_values,
                                        bool new_loop_closure,
                                        const UpdateInfo::LayerMerges& given_merges,
                                        const NodeToRobotMap* node_to_robot) {
  ScopedTimer spin_timer("backend/update_layers", timestamp_ns);

  // TODO(nathan) chance that this causes weirdness when we have multiple nodes but no
  // accepted reconciliation merges
  const bool enable_merging = given_merges.empty() ? config.enable_node_merging : false;

  const auto complete_agent_values =
      utils::getDenseFrames(full_sparse_frame_map_, sparse_frames_, pgmo_values);
  // TODO(nathan) given_merges probably doesn't need to be passed
  UpdateInfo::ConstPtr info(
      new UpdateInfo{config.add_places_to_deformation_graph ? &places_values : nullptr,
                     &pgmo_values,
                     new_loop_closure,
                     timestamp_ns,
                     given_merges,
                     &complete_agent_values,
                     deformation_graph_.get(),
                     node_to_robot});

  // merge topological changes to private dsg, respecting merges
  // attributes may be overwritten, but ideally we don't bother
  GraphMergeConfig merge_config;
  merge_config.previous_merges = &private_dsg_->merges;
  merge_config.update_dynamic_attributes = false;
  private_dsg_->graph->mergeGraph(*unmerged_graph_, merge_config);

  std::list<LayerCleanupFunc> cleanup_hooks;
  for (const auto& functor : update_functors_) {
    // TODO(nathan) keep track of names and push timing here
    if (!functor) {
      continue;
    }

    const auto hooks = functor->hooks();
    if (hooks.cleanup) {
      cleanup_hooks.push_back(hooks.cleanup);
    }

    functor->call(*unmerged_graph_, *private_dsg_, info);
    if (enable_merging) {
      // TODO(nathan) handle given merges
      const auto merges = functor->findMerges(*unmerged_graph_, info);
      const auto applied = merge_tracker.applyMerges(
          *unmerged_graph_, merges, *private_dsg_, hooks.merge);
      VLOG(1) << "Found " << merges.size() << " merges (applied " << applied << ")";

      if (config.enable_exhaustive_merging) {
        size_t merge_iter = 0;
        size_t num_applied = 0;
        do {
          const auto new_merges = functor->findMerges(*private_dsg_->graph, info);
          num_applied = merge_tracker.applyMerges(
              *private_dsg_->graph, new_merges, *private_dsg_, hooks.merge);
          VLOG(1) << "Found " << new_merges.size() << " merges at pass " << merge_iter
                  << " (" << num_applied << " applied)";
          ++merge_iter;
        } while (num_applied > 0);
      }
    }
  }

  launchCallbacks(cleanup_hooks, info, private_dsg_.get());
}

void BackendModule::logStatus(bool init) const {
  if (!logs_) {
    return;
  }

  const auto filename = logs_->getLogDir("backend/pgmo") + "/dsg_pgmo_status.csv";

  std::ofstream file;
  if (init) {
    LOG(INFO) << "[Hydra Backend] logging PGMO status output to " << filename;
    file.open(filename);
    // file format
    file << "total_lc,new_lc,total_factors,total_values,new_factors,new_graph_"
            "factors,trajectory_len,run_time,optimize_time,mesh_update_time,num_"
            "merges_"
            "undone\n";
    file.close();
    return;
  }

  const auto& timer = hydra::timing::ElapsedTimeRecorder::instance();
  const double nan = std::numeric_limits<double>::quiet_NaN();
  file.open(filename, std::ofstream::out | std::ofstream::app);
  file << status_.total_loop_closures << "," << status_.new_loop_closures << ","
       << status_.total_factors << "," << status_.total_values << ","
       << status_.new_factors << "," << status_.new_graph_factors << ","
       << status_.trajectory_len << ","
       << timer.getLastElapsed("backend/spin").value_or(nan) << ","
       << timer.getLastElapsed("backend/optimization").value_or(nan) << ","
       << timer.getLastElapsed("backend/mesh_update").value_or(nan) << ","
       << status_.num_merges_undone << std::endl;
  file.close();
  return;
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

}  // namespace hydra
