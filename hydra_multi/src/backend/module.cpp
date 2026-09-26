#include "hydra_multi/backend/module.h"

#include <config_utilities/config.h>
#include <config_utilities/printing.h>
#include <config_utilities/validation.h>
#include <glog/logging.h>
#include <hydra/backend/backend_utilities.h>
#include <hydra/backend/mst_factors.h>
#include <hydra/common/launch_callbacks.h>
#include <hydra/common/pipeline_queues.h>
#include <hydra/utils/pgmo_mesh_traits.h>
#include <hydra/utils/timing_utilities.h>
#include <kimera_pgmo/utils/mesh_io.h>
#include <kimera_rpgo/utils/g2o.h>

#include <ranges>

#include "hydra_multi/backend/loop_closure_initial_align.h"
#include "hydra_multi/backend/utils.h"
#include "hydra_multi/common/multi_global_info.h"

namespace hydra_multi {
using hydra::DsgUpdater;
using hydra::timing::ScopedTimer;

namespace {

inline gtsam::Pose3 projectToClosestSE3(const Eigen::Matrix4d& T) {
  Eigen::Matrix3d R = T.block<3, 3>(0, 0);
  Eigen::JacobiSVD<Eigen::Matrix3d> svd(R, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Matrix3d R_proj = svd.matrixU() * svd.matrixV().transpose();

  if (R_proj.determinant() < 0) {
    Eigen::Matrix3d U = svd.matrixU();
    U.col(2) *= -1;
    R_proj = U * svd.matrixV().transpose();
  }

  Eigen::Vector3d t = T.block<3, 1>(0, 3);

  return gtsam::Pose3(gtsam::Rot3(R_proj), gtsam::Point3(t));
}

inline static const auto registration_ =
    config::RegistrationWithConfig<MultiBackendModule,
                                   MultiBackendModule,
                                   MultiBackendModule::Config,
                                   InterfaceContainer::StatesPtr>("MultiBackendModule");

void writeBackendStatus(const std::vector<MultiBackendModuleStatus>& entries,
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
         << entry.spin_s.value_or(nan) << "," << entry.opt_s.value_or(nan) << ","
         << entry.mesh_s.value_or(nan) << "," << entry.num_merges_undone << "\n";
  }

  file.close();
}

void saveTrajectory(const SceneGraphLayer& layer,
                    const std::filesystem::path& output_path) {
  std::ofstream fout(output_path);
  fout << "#timestamp_kf,x,y,z,qw,qx,qy,qz\n";
  for (const auto& [node_id, node] : layer.nodes()) {
    const auto attrs = node->tryAttributes<AgentNodeAttributes>();
    if (!attrs) {
      LOG(ERROR) << "Invalid agent layer, node " << NodeSymbol(node_id).str()
                 << " is not agent node";
      continue;
    }

    fout << attrs->timestamp.count() << "," << attrs->position.x() << ","
         << attrs->position.y() << "," << attrs->position.z() << ","
         << attrs->world_R_body.w() << "," << attrs->world_R_body.x() << ","
         << attrs->world_R_body.y() << "," << attrs->world_R_body.z() << "\n";
  }

  fout.flush();
  fout.close();
}

}  // namespace

std::ostream& operator<<(std::ostream& os, const MultiBackendModuleStatus& status) {
  os << "new_loop_closures: " << status.new_loop_closures << ", "
     << "total_loop_closures: " << status.total_loop_closures << ", "
     << "total_interrobot_loop_closures: " << status.total_interrobot_loop_closures
     << ", " << "inlier_loop_closures: " << status.inlier_loop_closures << ", "
     << "inlier_interrobot_loop_closures: " << status.inlier_interrobot_loop_closures
     << ", " << "total_factors: " << status.total_factors << ", "
     << "total_values: " << status.total_values << ", "
     << "new_factors: " << status.new_factors << ", "
     << "new_graph_factors: " << status.new_graph_factors << ", "
     << "trajectory_len: " << status.trajectory_len << ", "
     << "num_merges_undone: " << status.num_merges_undone << ", " << "spin_s: "
     << (status.spin_s ? std::to_string(status.spin_s.value()) + " [s]" : "n/a") << ", "
     << "opt_s: "
     << (status.opt_s ? std::to_string(status.opt_s.value()) + " [s]" : "n/a") << ", "
     << "mesh_s: "
     << (status.mesh_s ? std::to_string(status.mesh_s.value()) + " [s]" : "n/a");
  return os;
}

void MultiBackendModuleStatus::reset() { *this = MultiBackendModuleStatus(); }

void declare_config(MultiBackendModule::Config& config) {
  using namespace config;
  base<DsgUpdater::Config>(config);
  name("MultiBackendModule::Config");
  field(config.pgmo, "pgmo");
  field(config.optimize_on_lc, "optimize_on_lc");
  field(config.anchor_robot_id, "anchor_robot_id");
  field(config.include_mesh_factors, "include_mesh_factors");
  field(config.external_loop_closures, "external_loop_closures");
  field(config.add_objects_to_deformation_graph, "add_objects_to_deformation_graph");
  field(config.object_variance, "object_variance");
  field(config.sg_loop_closure_variance, "sg_loop_closure_variance");
  field(config.object_association_max_diff_s, "object_association_max_diff_s", "s");
  config.initial_align.setOptional();
  field(config.initial_align, "initial_align");
  config.partial_optimizer.setOptional();
  field(config.partial_optimizer, "partial_optimizer");

  check(config.object_association_max_diff_s, GE, 0.0, "object_association_max_diff_s");
}

MultiBackendModule::MultiBackendModule(
    const Config& config, const InterfaceContainer::StatesPtr& shared_states)
    : KimeraPgmoInterface(config.pgmo),
      config(config::checkValid(config)),
      lc_receiver_(config.external_loop_closures, &lc_queue_),
      unmerged_dsg_(MultiGlobalInfo::instance().createMultiDsg()),
      merged_dsg_(MultiGlobalInfo::instance().createMultiDsg()),
      shared_states_(shared_states),
      initial_align_(config.initial_align.create()),
      dsg_updater_(new hydra::DsgUpdater(config, unmerged_dsg_->graph, merged_dsg_)),
      partial_pgo_(config.partial_optimizer.create()) {
  using EdgeType = pose_graph_tools::PoseGraphEdge::Type;
  edge_variance_map_[EdgeType::ODOM] = KimeraPgmoInterface::config_.odom_variance;
  edge_variance_map_[EdgeType::LOOPCLOSE] = KimeraPgmoInterface::config_.lc_variance;
  edge_variance_map_[EdgeType::MESH] = KimeraPgmoInterface::config_.mesh_edge_variance;
  edge_variance_map_[EdgeType::POSE_MESH] =
      KimeraPgmoInterface::config_.pose_mesh_variance;
  edge_variance_map_[EdgeType::MESH_POSE] =
      KimeraPgmoInterface::config_.pose_mesh_variance;
  edge_variance_map_[EdgeType::PRIOR] = KimeraPgmoInterface::config_.prior_variance;

  // make sure there's always a valid status measurement in the log
  status_.reset();
  status_log_.push_back(status_);
  if (!config.include_mesh_factors) {
    LOG(WARNING) << "Mesh factors are OFF! Mesh will not be updated!";
  }
}

MultiBackendModule::~MultiBackendModule() { stopImpl(); }

void MultiBackendModule::start() {
  spin_thread_.reset(new std::thread(&MultiBackendModule::spin, this));
  optimization_thread_.reset(new std::thread(&MultiBackendModule::optimizeSpin, this));
  LOG(INFO) << " [Hydra-Multi Backend] started!";
}

void MultiBackendModule::stop() { stopImpl(); }

void MultiBackendModule::save(const DataDirectory& output) {
  const auto backend_path = output.path("backend");
  const auto pgmo_path = output.path("backend/pgmo");
  merged_dsg_->graph->save(backend_path / "dsg.json", false);
  merged_dsg_->graph->save(backend_path / "dsg_with_mesh.json");

  const auto& graph = *merged_dsg_->graph;
  const auto desired_layer = graph.getLayerKey(DsgLayers::AGENTS)->layer;
  for (const auto& [prefix, layer] : graph.layer_partition(desired_layer)) {
    const auto robot_id = kimera_pgmo::robot_prefix_to_id.at(prefix);
    std::string filename = "robot_" + std::to_string(robot_id) + "_trajectory.csv";
    const std::filesystem::path trajectory_path = backend_path / filename;
    ::hydra_multi::saveTrajectory(*layer, trajectory_path);
  }

  const auto mesh = merged_dsg_->graph->mesh();
  if (mesh && !mesh->empty()) {
    // mesh implements vertex and face traits
    kimera_pgmo::WriteMesh(backend_path / "mesh.ply", *mesh, *mesh);
  }

  backend_graph_logger_.save(backend_path);
  writeBackendStatus(status_log_, pgmo_path / "dsg_pgmo_status.csv");
  deformation_graph_->save(pgmo_path / "deformation_graph.dgrf");

  const std::string output_csv = backend_path / "loop_closures.csv";
  std::ofstream output_file;
  output_file.open(output_csv);

  output_file << "time_from_ns,time_to_ns,x,y,z,qw,qx,qy,qz,type,level" << std::endl;
  for (const auto& loop_closure : loop_closures_) {
    // pose = src.between(dest) or to_T_from
    auto time_from = hydra::utils::getTimeNs(*merged_dsg_->graph, loop_closure.dest);
    auto time_to = hydra::utils::getTimeNs(*merged_dsg_->graph, loop_closure.src);
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

std::string MultiBackendModule::printInfo() const {
  std::stringstream ss;
  ss << config::toString(config_);
  return ss.str();
}

void MultiBackendModule::addSink(const Sink::Ptr& sink) {
  if (sink) {
    sinks_.push_back(sink);
  }
}

void MultiBackendModule::addLoopClosures(const pose_graph_tools::PoseGraph& info) {
  lc_queue_.push(info);
}

bool MultiBackendModule::optimizeOnce(Timestamp stamp) {
  if (processLoopClosures(stamp)) {
    optimize(stamp);
  }

  return true;
}

void MultiBackendModule::spin() {
  bool should_shutdown = false;
  while (!should_shutdown) {
    Timestamp stamp;
    bool has_update = pollSharedStates(stamp);
    if (MultiGlobalInfo::instance().force_shutdown() || !has_update) {
      // copy over shutdown request
      should_shutdown = should_shutdown_;
    }

    if (!has_update) {
      using namespace std::chrono_literals;
      std::this_thread::sleep_for(10ms);
      continue;
    }

    spinOnce(stamp);
  }
}

void MultiBackendModule::optimizeSpin() {
  bool should_shutdown = false;
  while (!should_shutdown) {
    Timestamp stamp;
    pollSharedStates(stamp, false);

    bool should_optimize = processLoopClosures(stamp) && config.optimize_on_lc;
    if (!should_optimize || MultiGlobalInfo::instance().force_shutdown()) {
      // copy over shutdown request
      should_shutdown = should_shutdown_;
    }

    if (should_optimize && !should_shutdown) {
      optimize(stamp);
    } else {
      using namespace std::chrono_literals;
      std::this_thread::sleep_for(10ms);
    }
  }
}

void MultiBackendModule::stopImpl() {
  VLOG(1) << "Backend stopping...";
  should_shutdown_ = true;

  if (optimization_thread_) {
    VLOG(3) << "[Hydra-Multi Backend] joining optimizer thread and stopping";
    optimization_thread_->join();
    optimization_thread_.reset();
    VLOG(3) << "[Hydra-Multi Backend] optimizer thread stopped!";
  }

  if (spin_thread_) {
    VLOG(3) << "[Hydra-Multi Backend] joining spin thread and stopping";
    spin_thread_->join();
    spin_thread_.reset();
    VLOG(3) << "[Hydra-Multi Backend] spin stopped!";
  }
}

bool MultiBackendModule::pollSharedStates(Timestamp& stamp, bool mark_updated) const {
  bool updated = false;
  stamp = 0;
  for (const auto& [robot_id, state] : *shared_states_) {
    std::lock_guard<std::mutex> graph_lock(state->mutex);
    if (state->updated) {
      updated = true;
      if (mark_updated) {
        state->updated = false;  // Toggle back to false
      }

      if (state->stamp > stamp) {
        stamp = state->stamp;
      }
    }
  }

  return updated;
}

bool MultiBackendModule::processLoopClosures(Timestamp timestamp_ns) {
  ScopedTimer timer("backend/process_external_lcs", timestamp_ns);

  size_t processed = 0;
  std::lock_guard<std::mutex> unmerged_lock(unmerged_dsg_->mutex);
  lc_receiver_.update(
      *unmerged_dsg_->graph,
      [this, &processed](
          NodeId to_node, NodeId from_node, const gtsam::Pose3 to_T_from) {
        LoopClosureLog lc{to_node, from_node, to_T_from, true, 1, false};
        loop_closures_.push_back(lc);

        // TODO(Yun) currently interrobot non-external lcs not added to initial_align
        const auto to_prefix = NodeSymbol(to_node).category();
        const auto from_prefix = NodeSymbol(from_node).category();
        if (initial_align_ && to_prefix != from_prefix) {
          interrobot_lc_queue_.push(lc);
        }

        ++num_loop_closures_;
        ++processed;
      });

  have_loopclosures_ = (loop_closures_.size() > 0);
  return processed > 0;
}

void MultiBackendModule::partialOptimization(const Timestamp& timestamp_ns) {
  gtsam::Values optimized;
  {  // critical section for values
    std::lock_guard<std::mutex> lock(optimized_.mutex);
    optimized = optimized_.values;
  }

  if (optimized.empty()) {
    return;
  }

  if (!partial_pgo_) {
    deformation_graph_->updateValues(optimized);
    return;
  }

  ScopedTimer timer("backend/partial_optimization", timestamp_ns);
  auto initial = deformation_graph_->getValuesCopy();

  bool has_all_keys = true;
  for (const auto& key : initial.keys()) {
    if (!optimized.exists(key)) {
      has_all_keys = false;
      break;
    }
  }

  if (has_all_keys) {
    LOG(WARNING) << "No work to do in partial optimization!";
    return;
  }

  auto factors = deformation_graph_->getFactorsCopy();
  auto temp_factors = deformation_graph_->getTempFactorsCopy();
  auto temp_initial = deformation_graph_->getTempValuesCopy();

  // TODO(Yun): remove hard coded value?
  static const gtsam::SharedNoiseModel& prior_noise =
      gtsam::noiseModel::Isotropic::Variance(6, 1.0e-3);

  for (const auto& key : optimized.keys()) {
    auto value = optimized.at<gtsam::Pose3>(key);
    factors.add(gtsam::PriorFactor<gtsam::Pose3>(key, value, prior_noise));
    initial.update(key, value);
  }

  // Set all factors as inliers
  auto known_indices = std::views::iota(size_t(0), factors.size());
  std::set<size_t> known_inliers(known_indices.begin(), known_indices.end());
  auto temp_known_indices = std::views::iota(size_t(0), temp_factors.size());
  std::set<size_t> temp_known_inliers(temp_known_indices.begin(),
                                      temp_known_indices.end());
  LOG(INFO) << "Starting partial optimization";
  try {
    partial_pgo_->update(factors,
                         initial,
                         known_inliers,
                         temp_factors,
                         temp_initial,
                         temp_known_inliers);
    LOG(INFO) << "Finished partial optimization";
  } catch (const gtsam::IndeterminantLinearSystemException& e) {
    LOG(ERROR) << "Failed optimization: '" << gtsam::Symbol(e.nearbyVariable());
    std::vector<gtsam::NonlinearFactorGraph::sharedFactor> nearby_factors;
    for (const auto& factor : factors) {
      for (const auto& key : factor->keys()) {
        if (key == e.nearbyVariable()) {
          nearby_factors.push_back(factor);
          break;
        }
      }
    }

    for (size_t i = 0; i < nearby_factors.size(); ++i) {
      const auto& factor = nearby_factors[i];
      std::stringstream ss;
      ss << "[";
      auto iter = factor->keys().begin();
      while (iter != factor->keys().end()) {
        ss << gtsam::Symbol(*iter).string();
        ++iter;
        if (iter != factor->keys().end()) {
          ss << ", ";
        }
      }
      ss << "]";
      LOG(ERROR) << "factor " << i << ": " << ss.str();
    }
  }

  auto estimates = partial_pgo_->getEstimates();
  auto temp_estimates = partial_pgo_->getTempEstimates();
  // Only update values
  deformation_graph_->updateValues(estimates);
  deformation_graph_->updateTempValues(temp_estimates);
}

bool MultiBackendModule::spinOnce(Timestamp timestamp_ns) {
  ScopedTimer timer("backend/spin", timestamp_ns);
  {
    auto lock = deformation_graph_->acquireLock();
    std::lock_guard<std::mutex> graph_lock(unmerged_dsg_->mutex);

    VLOG(2) << "Checking for updates @ " << timestamp_ns << " [ns]";
    if (!updateFromStates(timestamp_ns)) {
      VLOG(2) << "Waiting for states...";
      return true;
    }

    VLOG(1) << "Spinning once for update @ " << timestamp_ns << " [ns]";
    partialOptimization(timestamp_ns);

    combineAndDeformMesh(timestamp_ns, should_deform_);

    hydra::UpdateInfo::ConstPtr info(
        new hydra::UpdateInfo{timestamp_ns,
                              nullptr,
                              deformation_graph_->getValues(),
                              should_deform_,
                              {},
                              deformation_graph_.get(),
                              &unmerged_dsg_->node_robot_map});
    dsg_updater_->callUpdateFunctions(timestamp_ns, info);

    // should_deform_ = false;
    rebaseStates(timestamp_ns);
  }

  VLOG(2) << "Calling backend sinks @ " << timestamp_ns << " [ns]";
  ScopedTimer sink_timer("backend/sinks", timestamp_ns);
  MultiBackendModuleStatus status;
  {  // status critical section
    std::lock_guard<std::mutex> lock(status_mutex_);
    status = status_log_.back();
  }  // status critical section

  Sink::callAll(sinks_, timestamp_ns, *merged_dsg_->graph, *deformation_graph_, status);
  return true;
}

void MultiBackendModule::addLoopClosures() {
  // Add Loop Closures (only those not yet added to deformation graph)
  for (auto& lc : loop_closures_) {
    if (lc.added_to_dgraph) {
      continue;
    }
    deformation_graph_->processNewBetween(
        lc.src,
        lc.dest,
        lc.src_T_dest,
        (lc.level ? KimeraPgmoInterface::config_.lc_variance
                  : config.sg_loop_closure_variance));
    lc.added_to_dgraph = true;
  }
}

void MultiBackendModule::addObjectsToDeformationGraph() {
  if (!unmerged_dsg_->graph->hasLayer(DsgLayers::OBJECTS)) {
    return;
  }

  const auto& objects = unmerged_dsg_->graph->getLayer(DsgLayers::OBJECTS);
  LayerView view = LayerView(objects);

  for (const auto& obj : view) {
    auto attrs = obj.tryAttributes<ObjectNodeAttributes>();
    if (!attrs) {
      continue;  // not an object
    }

    // TODO(nathan) fix this to associate to agent nodes some other way
    const std::optional<NodeId> agent_node;
    /*    const auto agent_node = findClosestAgentId(attrs->last_update_time_ns,*/
    /*unmerged_dsg_->node_robot_map.at(obj.id),*/
    /*config.object_association_max_diff_s);*/
    if (!agent_node) {
      LOG(ERROR) << "Failed to associated object " << NodeSymbol(obj.id)
                 << " to agent node.";
      continue;
    }

    const auto& agent_attrs =
        unmerged_dsg_->graph->getNode(*agent_node).attributes<AgentNodeAttributes>();

    // Extract transforms
    gtsam::Pose3 W_T_obj =
        gtsam::Pose3(gtsam::Rot3(attrs->world_R_object), attrs->position);
    gtsam::Pose3 W_T_agent =
        gtsam::Pose3(gtsam::Rot3(agent_attrs.world_R_body), agent_attrs.position);
    auto agent_T_obj = W_T_agent.between(W_T_obj);

    // Add to deformation graph
    // TODO(Yun) this is needed right now bc scene graph transformed to world frame
    // while dgraph not. Fix in next PR.
    gtsam::Pose3 Wrobot_T_obj =
        curr_W_T_robot_.at(unmerged_dsg_->node_robot_map.at(obj.id)).between(W_T_obj);
    deformation_graph_->processNewNode(obj.id, Wrobot_T_obj, false);
    deformation_graph_->processNewBetween(
        agent_attrs.external_key, obj.id, agent_T_obj, config.object_variance);
  }
}

void MultiBackendModule::optimizeStartup() {
  addLoopClosures();
  initialAlignment();
}

void MultiBackendModule::optimizeCleanup(const gtsam::Values& results) {
  std::lock_guard<std::mutex> lock(optimized_.mutex);
  optimized_.values = gtsam::Values(results);
}

void MultiBackendModule::optimize(size_t timestamp_ns) {
  ScopedTimer timer("backend/optimize", timestamp_ns);
  LOG(INFO) << "Starting full optimization";
  kimera_pgmo::OptimizeStats stats;
  {  // timer scope
    ScopedTimer timer("backend/optimization", timestamp_ns, true, 0, false);
    stats = KimeraPgmoInterface::optimize();
  }  // timer scope
  LOG(INFO) << "Finish full optimization";

  {  // status scope
    std::lock_guard<std::mutex> lock(status_mutex_);
    status_.total_loop_closures = stats.total_loop_closures;
    status_.total_interrobot_loop_closures = stats.total_interrobot_loop_closures;
    status_.inlier_loop_closures = stats.inlier_loop_closures;
    status_.inlier_interrobot_loop_closures = stats.inlier_interrobot_loop_closures;
    status_.total_factors = stats.total_factors;
    status_.total_values = stats.total_values;
    status_.opt_s = stats.elapsed_s;
  }

  logStatus();
  should_deform_ = true;
}

gtsam::Pose3 MultiBackendModule::getRobotWorldFrame(RobotId robot_id,
                                                    gtsam::Values& optimized) {
  const char robot_prefix = kimera_pgmo::robot_id_to_prefix.at(robot_id);
  if (!deformation_graph_->hasPrefixPoses(robot_prefix)) {
    return gtsam::Pose3();
  }

  auto trajectory = deformation_graph_->getTrajectory(robot_prefix);
  auto last_key = trajectory.size() - 1;

  auto Wglobal_T_pose = trajectory.back();

  // TODO(nathan) think about this more?
  // Try to use instead the optimized value
  auto it = optimized.upper_bound(gtsam::Symbol(robot_prefix, last_key));
  if (it != optimized.begin()) {
    --it;
    if (gtsam::Symbol(it->key).chr() == robot_prefix) {
      last_key = gtsam::Symbol(it->key).index();
      Wglobal_T_pose = optimized.at<gtsam::Pose3>(it->key);
    }
  }

  const auto curr_W_T_pose = deformation_graph_->getInitialPose(robot_prefix, last_key);
  const auto Wrobot_T_pose = curr_W_T_robot_.at(robot_id).between(curr_W_T_pose);
  const auto Wglobal_T_Wrobot = Wglobal_T_pose.compose(Wrobot_T_pose.inverse());
  // NOTE(Yun): Weirdly numerical pile up and quickly diverges if we do not keep
  // projecting back...
  return projectToClosestSE3(Wglobal_T_Wrobot.matrix());
}

void MultiBackendModule::rebaseStates(size_t timestamp_ns) {
  ScopedTimer timer("backend/rebase_states", timestamp_ns);
  // TODO(Yun): Account for how to propagate merged nodes or map to robot / unit
  // interface.
  gtsam::Values optimized;
  {
    std::lock_guard<std::mutex> lock(optimized_.mutex);
    optimized = optimized_.values;
  }

  for (const auto& [id, state] : *shared_states_) {
    std::lock_guard<std::mutex> state_lock(state->mutex);
    state->world_T_robot = getRobotWorldFrame(id, optimized);
  }
}

bool MultiBackendModule::updateFromStates(Timestamp timestamp_ns) {
  ScopedTimer timer("backend/update_from_states", timestamp_ns);
  bool updates = false;
  unmerged_dsg_->clear();

  // Accumulate factor graphs from each robot into one
  deformation_graph_->clear();

  // Reset loop closure flags since the deformation graph was cleared
  // They will be re-added by addLoopClosures() at the end of updateFromStates
  for (auto& lc : loop_closures_) {
    lc.added_to_dgraph = false;
  }

  for (const auto& [id, state] : *shared_states_) {
    VLOG(3) << "Saw robot " << id << " in update @ " << timestamp_ns << " [ns]";
    std::lock_guard<std::mutex> state_lock(state->mutex);

    // Append transform
    Eigen::Isometry3d transform(state->world_T_robot.matrix());
    curr_W_T_robot_[id] = gtsam::Pose3(transform.matrix());
    // Unfortunately, "transforming" the deformation graph when adding makes initial
    // alignment a bit more complicated. Hence requiring us to store the transform to
    // account for it in initial alignment.

    //// Process pose graph and mesh graph
    // Check and construct remapping ids then process pose graph and mesh graph
    // Implicit assumption that each state here only have a single id
    if (state->pose_graph_->nodes.size() > 0) {
      std::map<size_t, size_t> id_mapping;
      id_mapping[state->pose_graph_->nodes[0].robot_id] = id;
      VLOG(3) << "Remapping " << state->pose_graph_->nodes[0].robot_id << " -> " << id
              << " for pg with " << state->pose_graph_->nodes.size() << " nodes";
      deformation_graph_->processPoseGraph(
          *state->pose_graph_, edge_variance_map_, id_mapping, &transform);
      if (config.anchor_robot_id >= 0 &&
          id == static_cast<size_t>(config.anchor_robot_id)) {
        auto prefix = kimera_pgmo::GetRobotPrefix(id);
        deformation_graph_->processNewNode(
            gtsam::Symbol(prefix, 0), gtsam::Pose3(), true, config.pgmo.prior_variance);
      }
    }

    if (state->mesh_graph_->nodes.size() > 0 && config.include_mesh_factors) {
      std::map<size_t, size_t> id_mapping;
      id_mapping[state->mesh_graph_->nodes[0].robot_id] = id;
      deformation_graph_->processMeshGraph(
          *state->mesh_graph_, edge_variance_map_, id_mapping, &transform);
    }

    if (!state->dsg_) {
      continue;
    }

    // TODO(nathan) this could be handled better
    updates = true;
    unmerged_dsg_->addRobotGraph(id, *state->dsg_, &transform);

    if (!state->mesh_data_ || !state->mesh_data_->mesh ||
        state->mesh_data_->mesh->empty()) {
      continue;
    }

    staged_mesh_data_[id] = state->mesh_data_->clone();
    staged_mesh_data_[id]->transform(transform);
  }

  // Add objects or places to deformation graph
  deformation_graph_->clearTemporaryStructures();
  if (config.add_objects_to_deformation_graph) {
    addObjectsToDeformationGraph();
  }

  // Add loop closures so they appear in the published pose graph
  addLoopClosures();

  return updates;
}

void MultiBackendModule::combineAndDeformMesh(Timestamp timestamp_ns,
                                              bool force_mesh_update) {
  ScopedTimer timer("backend/deform_mesh", timestamp_ns);

  Mesh::Ptr combined_mesh;
  for (const auto& [id, mesh_data] : staged_mesh_data_) {
    if (!mesh_data) {
      LOG(WARNING) << "No mesh for robot " << id;
      continue;
    }

    if (have_loopclosures_ || force_mesh_update) {
      VLOG(3) << "Deforming mesh for robot " << id;
      // Note that here the mesh in the state is actually getting modified!
      char vertex_prefix = kimera_pgmo::robot_id_to_vertex_prefix.at(id);
      const kimera_pgmo::ConstStampedCloud<pcl::PointXYZ> cloud_in{
          *mesh_data->original_vertices, *mesh_data->vertex_stamps};
      deformation_graph_->deformPoints(
          *mesh_data->mesh,
          cloud_in,
          vertex_prefix,
          *deformation_graph_->getValues(),
          KimeraPgmoInterface::config_.num_interp_pts,
          KimeraPgmoInterface::config_.interp_horizon,
          nullptr,
          0);  // TODO(Yun): cannot use pre archived vertices as 'start_hint' because
               // there could be a change in initial alignment. but this could be pretty
               // slow
    }

    VLOG(5) << "Copying mesh to combined mesh for robot " << id;
    if (!combined_mesh) {
      combined_mesh = mesh_data->mesh->clone();
      unmerged_dsg_->robot_vertex_offset[id] = 0;
    } else {
      unmerged_dsg_->robot_vertex_offset[id] = combined_mesh->numVertices();
      *combined_mesh += *mesh_data->mesh;
    }

    unmerged_dsg_->robot_num_vertices[id] = mesh_data->mesh->numVertices();
  }

  if (combined_mesh) {
    unmerged_dsg_->graph->setMesh(combined_mesh);
    reindexMeshConnections(*unmerged_dsg_->graph,
                           unmerged_dsg_->node_robot_map,
                           unmerged_dsg_->robot_vertex_offset);
  } else {
    VLOG(3) << "No mesh for any robot!";
  }

  // TODO(nathan) fix this!!!!!!!!!!!!!!!!
  // Merged DSG is actually reset twice here (but resetBackendDsg necessary to reset
  // MergeTracker)
  dsg_updater_->resetBackendDsg(timestamp_ns);
  std::lock_guard<std::mutex> merged_graph_lock(merged_dsg_->mutex);
  merged_dsg_->update(*unmerged_dsg_);
}

void MultiBackendModule::initialAlignment() {
  if (!initial_align_) {
    LOG(WARNING) << "Initial alignment not set.";
    return;
  }

  // TODO(nathan) figure out how to handle inheritance
  auto info = std::make_shared<LoopClosureInitialAlignModule::Info>();
  // W_T_robot only updated during initial
  info->prev_W_T_robot = curr_W_T_robot_;
  info->dgraph = deformation_graph_;
  while (!interrobot_lc_queue_.empty()) {
    const auto lc = interrobot_lc_queue_.pop();
    info->loop_closures.push_back({lc.src, lc.dest, lc.src_T_dest});
  }

  VLOG(10) << "Running initial alignment";
  initial_align_->update(info);

  // TODO(Yun): temp values not aligned. Might be fine for now
  auto pgmo_values = deformation_graph_->getValuesCopy();
  const auto initial_values = initial_align_->computeInitialGuess(
      info, pgmo_values, merged_dsg_->node_robot_map);
  deformation_graph_->updateValues(initial_values);
}

void MultiBackendModule::logStatus() {
  std::lock_guard<std::mutex> lock(status_mutex_);

  const auto& timer = hydra::timing::ElapsedTimeRecorder::instance();
  status_.spin_s = timer.getLastElapsed("backend/spin");
  status_.mesh_s = timer.getLastElapsed("backend/mesh_update");
  status_log_.push_back(status_);
  status_.reset();
}

}  // namespace hydra_multi
