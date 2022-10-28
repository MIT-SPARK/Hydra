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
#include "hydra_dsg_builder/incremental_dsg_backend.h"
#include "hydra_dsg_builder/minimum_spanning_tree.h"

#include <glog/logging.h>
#include <hydra_utils/timing_utilities.h>
#include <pcl/search/kdtree.h>
#include <voxblox/core/block_hash.h>

namespace hydra {
namespace incremental {

using hydra::dsg_updates::NodeMergeLog;
using hydra::timing::ScopedTimer;
using kimera_pgmo::DeformationGraph;
using kimera_pgmo::DeformationGraphPtr;
using kimera_pgmo::KimeraPgmoInterface;
using kimera_pgmo::KimeraPgmoMesh;
using pose_graph_tools::PoseGraph;
using LayerMerges = std::map<LayerId, std::map<NodeId, NodeId>>;

std::optional<uint64_t> getTimeNs(const DynamicSceneGraph& graph, gtsam::Symbol key) {
  NodeSymbol node(key.chr(), key.index());
  if (!graph.hasNode(node)) {
    LOG(ERROR) << "Missing node << " << node.getLabel() << "when logging loop closure";
    LOG(ERROR) << "Num dynamic nodes: " << graph.numDynamicNodes();
    return std::nullopt;
  }

  return graph.getDynamicNode(node).value().get().timestamp.count();
}

DsgBackend::DsgBackend(const RobotPrefixConfig& prefix,
                       const DsgBackendConfig& config,
                       const kimera_pgmo::KimeraPgmoConfig& pgmo_config,
                       const SharedDsgInfo::Ptr& dsg,
                       const SharedDsgInfo::Ptr& backend_dsg,
                       const SharedModuleState::Ptr& state)
    : KimeraPgmoInterface(),
      prefix_(prefix),
      config_(config),
      shared_dsg_(dsg),
      private_dsg_(backend_dsg),
      shared_places_copy_(DsgLayers::PLACES),
      state_(state) {
  KimeraPgmoInterface::config_ = pgmo_config;

  if (!KimeraPgmoInterface::initializeFromConfig()) {
    throw std::runtime_error("invalid pgmo config");
  }

  optimized_mesh_.reset(new pcl::PolygonMesh());
  setDefaultUpdateFunctions();
  deformation_graph_->setForceRecalculate(!config_.pgmo.gnc_fix_prev_inliers);
  deformation_graph_->storeOnlyNoOptimization();

  if (config_.should_log) {
    const std::string log_path = config_.log_path + "/backend";
    backend_graph_logger_.setOutputPath(log_path);
    LOG(INFO) << "[Hydra Backend] logging to " << log_path;
    backend_graph_logger_.setLayerName(DsgLayers::OBJECTS, "objects");
    backend_graph_logger_.setLayerName(DsgLayers::PLACES, "places");
    backend_graph_logger_.setLayerName(DsgLayers::ROOMS, "rooms");
    backend_graph_logger_.setLayerName(DsgLayers::BUILDINGS, "buildings");
  } else {
    LOG(INFO) << "[Hydra Backend] logging disabled.";
  }

  if (config_.use_zmq_interface) {
    zmq_receiver_.reset(
        new spark_dsg::ZmqReceiver(config_.zmq_recv_url, config_.zmq_num_threads));
  }
}

DsgBackend::~DsgBackend() {
  LOG(INFO) << "[Hydra Backend] destructor called!";
  stop();
}

void DsgBackend::start() {
  spin_thread_.reset(new std::thread(&DsgBackend::spin, this));

  if (config_.use_zmq_interface) {
    zmq_thread_.reset(new std::thread(&DsgBackend::runZmqUpdates, this));
  }
  LOG(INFO) << "[Hydra Backend] started!";
}

void DsgBackend::stop() {
  should_shutdown_ = true;

  if (spin_thread_) {
    VLOG(2) << "[Hydra Backend] joining optimizer thread and stopping";
    spin_thread_->join();
    spin_thread_.reset();
    VLOG(2) << "[Hydra Backend] stopped!";
  }

  if (zmq_thread_) {
    VLOG(2) << "[Hydra Backend] joining zmq thread and stopping";
    zmq_thread_->join();
    zmq_thread_.reset();
    VLOG(2) << "[Hydra Backend] stopped!";
  }

  VLOG(2) << "[Hydra Backend]: " << state_->backend_queue.size() << " messages left";
}

void DsgBackend::save(const std::string& output_path) {
  private_dsg_->graph->save(output_path + "/dsg.json", false);
  private_dsg_->graph->save(output_path + "/dsg_with_mesh.json");
  // TODO(Yun) Technically not strictly a g2o
  deformation_graph_->save(config_.pgmo.log_path + "/deformation_graph.dgrf");
  savePoseGraphSparseMapping(config_.pgmo.log_path + "/sparsification_mapping.txt");

  if (deformation_graph_->hasPrefixPoses(prefix_.key)) {
    const auto optimized_path = getOptimizedTrajectory(prefix_.id);
    std::string csv_name = config_.pgmo.log_path + std::string("/traj_pgmo.csv");
    saveTrajectory(optimized_path, timestamps_, csv_name);
  }

  if (!private_dsg_->graph->isMeshEmpty()) {
    kimera_pgmo::WriteMeshWithStampsToPly(output_path + "/mesh.ply",
                                          private_dsg_->graph->getMesh(),
                                          *mesh_vertex_stamps_);
  }

  const std::string output_csv = output_path + "/loop_closures.csv";
  std::ofstream output_file;
  output_file.open(output_csv);

  output_file << "time_from_ns,time_to_ns,x,y,z,qw,qx,qy,qz,type,level" << std::endl;
  for (const auto& loop_closure : loop_closures_) {
    // pose = src.between(dest) or to_T_from
    auto time_from = getTimeNs(*private_dsg_->graph, loop_closure.dest);
    auto time_to = getTimeNs(*private_dsg_->graph, loop_closure.src);
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


void DsgBackend::spin() {
  while (!should_shutdown_) {
    bool has_data = state_->backend_queue.poll();
    if (!has_data) {
      continue;
    }

    spinOnce(*state_->backend_queue.front(), false);
    state_->backend_queue.pop();

    // TODO(nathan) don't exit if there are still messages in the queue
  }
}

bool DsgBackend::spinOnce(bool force_update) {
  bool has_data = state_->backend_queue.poll();
  if (!has_data) {
    return false;
  }

  spinOnce(*state_->backend_queue.front(), force_update);
  state_->backend_queue.pop();
  return true;
}

void DsgBackend::spinOnce(const BackendInput& input, bool force_update) {
  ScopedTimer spin_timer("backend/spin", input.timestamp_ns);
  status_.reset();

  updateFactorGraph(input);
  updateFromLcdQueue();
  status_.total_loop_closures_ = num_loop_closures_;

  if (!updatePrivateDsg(input.timestamp_ns, force_update)) {
    // we only read from the frontend dsg if we've processed all the
    // factor graph update packets (as long as force_update is false)
    // we still log the status for each received frontend packet
    logStatus();
    return;
  }

  if (!config_.use_mesh_subscribers) {
    // copy latest mesh if we getting it directly from the frontend
    updateFromSharedState(input.timestamp_ns);
  }

  if (config_.optimize_on_lc && have_loopclosures_) {
    optimize(input.timestamp_ns);
  } else if (config_.call_update_periodically) {
    updateDsgMesh(input.timestamp_ns);
    callUpdateFunctions(input.timestamp_ns);
  }

  if (config_.pgmo.should_log) {
    logStatus();
  }

  for (const auto& cb_func : output_callbacks_) {
    cb_func(*private_dsg_->graph,
            *optimized_mesh_,
            *deformation_graph_,
            input.timestamp_ns);
  }
}

void DsgBackend::loadState(const std::string& state_path,
                           const std::string& dgrf_path) {
  latest_mesh_.reset(new pcl::PolygonMesh());
  mesh_vertex_stamps_.reset(new std::vector<ros::Time>());
  kimera_pgmo::ReadMeshWithStampsFromPly(
      state_path + "/mesh.ply", *latest_mesh_, mesh_vertex_stamps_.get());
  have_new_mesh_ = true;

  loadDeformationGraphFromFile(dgrf_path);
  LOG(WARNING) << "Loaded " << deformation_graph_->getNumVertices()
               << " vertices for deformation graph";
}

void DsgBackend::setUpdateFuncs(const std::list<LayerUpdateFunc>& update_funcs) {
  dsg_update_funcs_ = update_funcs;
}

void DsgBackend::setSolverParams() {
  KimeraRPGO::RobustSolverParams params = deformation_graph_->getParams();
  params.verbosity = config_.pgmo.rpgo_verbosity;
  params.solver = config_.pgmo.rpgo_solver;
  if (config_.pgmo.should_log) {
    params.logOutput(config_.pgmo.log_path);
    logStatus(true);
  }
  deformation_graph_->setParams(params);
  setVerboseFlag(false);
}

void DsgBackend::setDefaultUpdateFunctions() {
  update_objects_functor_.reset(new dsg_updates::UpdateObjectsFunctor());
  update_places_functor_.reset(new dsg_updates::UpdatePlacesFunctor(
      config_.places_merge_pos_threshold_m, config_.places_merge_distance_tolerance_m));
  update_buildings_functor_.reset(new dsg_updates::UpdateBuildingsFunctor(
      config_.building_color, config_.building_semantic_label));

  merge_handler_.reset(new MergeHandler(
      update_objects_functor_, update_places_functor_, config_.enable_merge_undos));

  dsg_update_funcs_.push_back(&dsg_updates::updateAgents);
  dsg_update_funcs_.push_back(std::bind(&dsg_updates::UpdateObjectsFunctor::call,
                                        update_objects_functor_.get(),
                                        std::placeholders::_1,
                                        std::placeholders::_2));
  dsg_update_funcs_.push_back(std::bind(&dsg_updates::UpdatePlacesFunctor::call,
                                        update_places_functor_.get(),
                                        std::placeholders::_1,
                                        std::placeholders::_2));

  if (config_.enable_rooms) {
    update_rooms_functor_.reset(
        new dsg_updates::UpdateRoomsFunctor(config_.room_finder));
    dsg_update_funcs_.push_back(std::bind(&dsg_updates::UpdateRoomsFunctor::call,
                                          update_rooms_functor_.get(),
                                          std::placeholders::_1,
                                          std::placeholders::_2));
  }

  dsg_update_funcs_.push_back(std::bind(&dsg_updates::UpdateBuildingsFunctor::call,
                                        update_buildings_functor_.get(),
                                        std::placeholders::_1,
                                        std::placeholders::_2));
}


void DsgBackend::updateFactorGraph(const BackendInput& input) {
  ScopedTimer timer("backend/process_factors", input.timestamp_ns);
  const size_t prev_loop_closures = num_loop_closures_;

  status_.new_graph_factors_ = input.deformation_graph->edges.size();
  status_.new_factors_ += input.deformation_graph->edges.size();

  try {
    processIncrementalMeshGraph(
        input.deformation_graph, timestamps_, &unconnected_nodes_);
  } catch (const gtsam::ValuesKeyDoesNotExist& e) {
    LOG(ERROR) << *input.deformation_graph;
    throw std::logic_error(e.what());
  }

  for (const auto& msg : input.pose_graphs) {
    status_.new_factors_ += msg->edges.size();
    processIncrementalPoseGraph(msg, &trajectory_, &unconnected_nodes_, &timestamps_);
    logIncrementalLoopClosures(*msg);
  }

  if (num_loop_closures_ > prev_loop_closures) {
    LOG(WARNING) << "New loop closures detected!";
    have_new_loopclosures_ = true;
  }

  if (num_loop_closures_ > 0) {
    status_.new_loop_closures_ = num_loop_closures_ - prev_loop_closures;
    have_loopclosures_ = true;
  }
  status_.trajectory_len_ = trajectory_.size();
  status_.total_factors_ = deformation_graph_->getGtsamFactors().size();
  status_.total_values_ = deformation_graph_->getGtsamValues().size();
}

bool DsgBackend::updateFromLcdQueue() {
  bool added_new_loop_closure = false;
  while (!state_->backend_lcd_queue.empty()) {
    const auto result = state_->backend_lcd_queue.pop();

    // note that pose graph convention is pose = src.between(dest) where the edge
    // connects frames "to -> from" (i.e. src = to, dest = from, pose = to_T_from)
    LoopClosureLog lc{
        result.to_node, result.from_node, result.to_T_from, true, result.level};
    addLoopClosure(lc.src, lc.dest, lc.src_T_dest);

    loop_closures_.push_back(lc);

    added_new_loop_closure = true;
    have_loopclosures_ = true;
    have_new_loopclosures_ = true;
    num_loop_closures_++;
    status_.new_loop_closures_++;
  }

  return added_new_loop_closure;
}

bool DsgBackend::updatePrivateDsg(size_t timestamp_ns, bool force_update) {
  std::unique_lock<std::mutex> graph_lock(private_dsg_->mutex);
  {  // start joint critical section
    std::unique_lock<std::mutex> shared_graph_lock(shared_dsg_->mutex);
    if (!force_update && shared_dsg_->last_update_time > timestamp_ns) {
      return false;
    }

    private_dsg_->graph->mergeGraph(*shared_dsg_->graph,
                                    merge_handler_->mergedNodes(),
                                    true,
                                    false,
                                    true,
                                    &config_.merge_update_map,
                                    config_.merge_update_dynamic);

    // update merge book-keeping and optionally update merged node
    // connections and attributes
    merge_handler_->updateFromUnmergedGraph(*shared_dsg_->graph);

    if (shared_dsg_->graph->hasLayer(DsgLayers::PLACES)) {
      // TODO(nathan) simplify
      const auto& places = shared_dsg_->graph->getLayer(DsgLayers::PLACES);
      shared_places_copy_.mergeLayer(places, {});
      std::vector<NodeId> removed_place_nodes;
      places.getRemovedNodes(removed_place_nodes);
      for (const auto& place_id : removed_place_nodes) {
        shared_places_copy_.removeNode(place_id);
      }
    }
  }  // end joint critical section

  if (config_.should_log) {
    backend_graph_logger_.logGraph(private_dsg_->graph);
  }

  return true;
}

void DsgBackend::updateFromSharedState(size_t timestamp_ns) {
  std::unique_lock<std::mutex> lock(state_->mesh_mutex);
  ScopedTimer timer("backend/update_from_shared", timestamp_ns);
  if (state_->have_new_mesh) {
    // latch the pointer so that frontend can copy a new one without
    // getting tied up waiting for the mesh interpolation
    latest_mesh_ = state_->latest_mesh;
    mesh_vertex_stamps_ = state_->mesh_vertex_stamps;
    mesh_vertex_graph_inds_ = state_->mesh_vertex_graph_indices;
    update_objects_functor_->invalid_indices.reset(new std::set<size_t>(
        state_->invalid_indices->begin(), state_->invalid_indices->end()));
    state_->have_new_mesh = false;
    have_new_mesh_ = true;
  }
}

void DsgBackend::addPlacesToDeformationGraph(size_t timestamp_ns) {
  if (shared_places_copy_.nodes().empty()) {
    LOG(WARNING) << "Attempting to add places to deformation graph without places";
    return;
  }

  ScopedTimer timer("backend/add_places", timestamp_ns);

  deformation_graph_->clearTemporaryStructures();

  MinimumSpanningTreeInfo mst_info;
  {  // start timing scope
    ScopedTimer mst_timer("backend/places_mst", timestamp_ns);
    mst_info = getMinimumSpanningEdges(shared_places_copy_);
  }  // end timing scope

  {  // start timing scope
    ScopedTimer add_timer("backend/add_places_nodes", timestamp_ns);

    std::vector<gtsam::Key> place_nodes;
    std::vector<gtsam::Pose3> place_node_poses;
    std::vector<std::vector<size_t>> place_node_valences;

    for (const auto& id_node_pair : shared_places_copy_.nodes()) {
      const auto& node = *id_node_pair.second;
      const auto& attrs = node.attributes<PlaceNodeAttributes>();

      if (!node.hasSiblings()) {
        continue;
      }

      place_nodes.push_back(node.id);
      place_node_poses.push_back(gtsam::Pose3(gtsam::Rot3(), attrs.position));

      if (mst_info.leaves.count(node.id)) {
        place_node_valences.push_back(attrs.deformation_connections);
      } else {
        place_node_valences.push_back(std::vector<size_t>{});
      }
    }

    deformation_graph_->addNewTempNodesValences(place_nodes,
                                                place_node_poses,
                                                place_node_valences,
                                                prefix_.vertex_key,
                                                false,
                                                config_.pgmo.place_mesh_variance);
  }  // end timing scope

  {  // start timing scope
    ScopedTimer between_timer("backend/add_places_between", timestamp_ns);
    PoseGraph mst_edges;
    for (const auto& edge : mst_info.edges) {
      gtsam::Pose3 source(gtsam::Rot3(), shared_places_copy_.getPosition(edge.source));
      gtsam::Pose3 target(gtsam::Rot3(), shared_places_copy_.getPosition(edge.target));
      pose_graph_tools::PoseGraphEdge mst_e;
      mst_e.key_from = edge.source;
      mst_e.key_to = edge.target;
      mst_e.pose = kimera_pgmo::GtsamToRos(source.between(target));
      mst_edges.edges.push_back(mst_e);
    }
    deformation_graph_->addNewTempEdges(mst_edges, config_.pgmo.place_edge_variance);
  }  // end timing scope
}

const pcl::PolygonMesh* DsgBackend::getLatestMesh() { return latest_mesh_.get(); }

void DsgBackend::addLoopClosure(const gtsam::Key& src,
                                const gtsam::Key& dest,
                                const gtsam::Pose3& src_T_dest) {
  if (full_sparse_frame_map_.size() == 0 ||
      !KimeraPgmoInterface::config_.b_enable_sparsify) {
    deformation_graph_->addNewBetween(src,
                                      dest,
                                      src_T_dest,
                                      gtsam::Pose3(),
                                      KimeraPgmoInterface::config_.lc_variance);
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
    deformation_graph_->addNewBetween(sparse_src,
                                      sparse_dest,
                                      sparse_src_T_sparse_dest,
                                      gtsam::Pose3(),
                                      KimeraPgmoInterface::config_.lc_variance);
  }
}

void DsgBackend::runZmqUpdates() {
  while (!should_shutdown_) {
    if (!zmq_receiver_->recv(config_.poll_time_ms)) {
      continue;
    }

    std::unique_lock<std::mutex> lock(private_dsg_->mutex);
    auto update_graph = zmq_receiver_->graph();
    if (!update_graph) {
      LOG(ERROR) << "zmq receiver graph is invalid";
      continue;
    }

    const auto& rooms = update_graph->getLayer(DsgLayers::ROOMS);
    for (const auto& id_node_pair : rooms.nodes()) {
      const auto new_name =
          id_node_pair.second->attributes<SemanticNodeAttributes>().name;
      room_name_map_[id_node_pair.first] = new_name;

      auto node_opt = private_dsg_->graph->getNode(id_node_pair.first);
      if (!node_opt) {
        VLOG(1) << "received update for node "
                << NodeSymbol(id_node_pair.first).getLabel()
                << " but node no longer exists";
        continue;
      }

      VLOG(2) << "assiging name " << new_name << " to "
              << NodeSymbol(id_node_pair.first).getLabel();
      node_opt->get().attributes<SemanticNodeAttributes>().name = new_name;
    }
  }
}

void DsgBackend::updateDsgMesh(size_t timestamp_ns, bool force_mesh_update) {
  if (!force_mesh_update && !have_new_mesh_) {
    return;
  }

  auto mesh = getLatestMesh();
  if (!mesh || mesh->cloud.height * mesh->cloud.width == 0) {
    return;
  }
  have_new_mesh_ = false;

  ScopedTimer timer("backend/mesh_update", timestamp_ns);
  VLOG(3) << "Deforming mesh with " << mesh_vertex_stamps_->size() << " vertices";

  // TODO(nathan) check if copying happening
  optimized_mesh_.reset(new pcl::PolygonMesh(
      deformation_graph_->deformMesh(*latest_mesh_,
                                     *mesh_vertex_stamps_,
                                     *mesh_vertex_graph_inds_,
                                     prefix_.vertex_key,
                                     KimeraPgmoInterface::config_.num_interp_pts,
                                     KimeraPgmoInterface::config_.interp_horizon)));
  {
    // start private dsg critical section
    std::unique_lock<std::mutex> graph_lock(private_dsg_->mutex);
    private_dsg_->graph->setMeshDirectly(*optimized_mesh_);
  }
}

void DsgBackend::optimize(size_t timestamp_ns) {
  if (config_.add_places_to_deformation_graph) {
    addPlacesToDeformationGraph(timestamp_ns);
  }

  {  // timer scope
    ScopedTimer timer("backend/optimization", timestamp_ns, true, 0, false);
    deformation_graph_->optimize();
  }  // timer scope

  updateDsgMesh(timestamp_ns, true);

  gtsam::Values pgmo_values = deformation_graph_->getGtsamValues();
  gtsam::Values places_values = deformation_graph_->getGtsamTempValues();

  callUpdateFunctions(timestamp_ns, places_values, pgmo_values, have_new_loopclosures_);
  have_new_loopclosures_ = false;
}

void DsgBackend::callUpdateFunctions(size_t timestamp_ns,
                                     const gtsam::Values& places_values,
                                     const gtsam::Values& pgmo_values,
                                     bool new_loop_closure,
                                     const LayerMerges& given_merges) {
  bool enable_node_merging = config_.enable_node_merging;
  if (given_merges.size() > 0) {
    enable_node_merging = false;
  }

  gtsam::Values complete_agent_values;
  if (full_sparse_frame_map_.size() == 0) {
    complete_agent_values = pgmo_values;
  } else {
    for (const auto& agent_sparse_key : full_sparse_frame_map_) {
      const auto& dense_key = agent_sparse_key.first;
      const auto& sparse_key = agent_sparse_key.second;
      if (!pgmo_values.exists(sparse_key)) {
        continue;
      }

      const auto& sparse_T_dense = sparse_frames_.at(sparse_key).keyed_transforms.at(dense_key);
      gtsam::Pose3 agent_pose = pgmo_values.at<gtsam::Pose3>(sparse_key).compose(sparse_T_dense);
      complete_agent_values.insert(dense_key, agent_pose);
    }
  }

  const UpdateInfo info{&places_values,
                        &pgmo_values,
                        new_loop_closure,
                        timestamp_ns,
                        enable_node_merging,
                        &complete_agent_values};

  if (config_.enable_merge_undos) {
    status_.num_merges_undone_ =
        merge_handler_->checkAndUndo(*private_dsg_->graph, info);
  }

  ScopedTimer spin_timer("backend/update_layers", timestamp_ns);
  for (const auto& update_func : dsg_update_funcs_) {
    auto merged_nodes = update_func(*private_dsg_, info);
    merge_handler_->updateMerges(merged_nodes, *private_dsg_->graph);
  }

  for (const auto& layer_merges : given_merges) {
    merge_handler_->updateMerges(layer_merges.second, *private_dsg_->graph);
  }

  std::unique_lock<std::mutex> lock(private_dsg_->mutex);
  const auto& rooms = private_dsg_->graph->getLayer(DsgLayers::ROOMS);
  for (auto& id_node_pair : rooms.nodes()) {
    const auto iter = room_name_map_.find(id_node_pair.first);
    if (iter == room_name_map_.end()) {
      continue;
    }

    id_node_pair.second->attributes<SemanticNodeAttributes>().name = iter->second;
  }
}

void DsgBackend::logStatus(bool init) const {
  std::ofstream file;
  std::string filename = config_.pgmo.log_path + std::string("/dsg_pgmo_status.csv");
  if (init) {
    LOG(INFO) << "[Hydra Backend] logging PGMO status output to " << filename;
    file.open(filename);
    // file format
    file << "total_lc,new_lc,total_factors,total_values,new_factors,new_graph_"
            "factors,trajectory_len,run_time,optimize_time,mesh_update_time,num_merges_"
            "undone\n";
    file.close();
    return;
  }

  const auto& timer = hydra::timing::ElapsedTimeRecorder::instance();
  const double nan = std::numeric_limits<double>::quiet_NaN();
  file.open(filename, std::ofstream::out | std::ofstream::app);
  file << status_.total_loop_closures_ << "," << status_.new_loop_closures_ << ","
       << status_.total_factors_ << "," << status_.total_values_ << ","
       << status_.new_factors_ << "," << status_.new_graph_factors_ << ","
       << status_.trajectory_len_ << ","
       << timer.getLastElapsed("backend/spin").value_or(nan) << ","
       << timer.getLastElapsed("backend/optimization").value_or(nan) << ","
       << timer.getLastElapsed("backend/mesh_update").value_or(nan) << ","
       << status_.num_merges_undone_ << std::endl;
  file.close();
  return;
}

void DsgBackend::logIncrementalLoopClosures(const PoseGraph& msg) {
  for (const auto& edge : msg.edges) {
    if (edge.type != pose_graph_tools::PoseGraphEdge::LOOPCLOSE) {
      continue;
    }

    gtsam::Pose3 pose = kimera_pgmo::RosToGtsam(edge.pose);
    const gtsam::Symbol src_key(prefix_.key, edge.key_from);
    const gtsam::Symbol dest_key(prefix_.key, edge.key_to);
    // note that pose graph convention is pose = src.between(dest) where the edge
    // connects frames "to -> from" (i.e. src = to, dest = from, pose = to_T_from)
    loop_closures_.push_back({src_key, dest_key, pose, false, 0});
  }
}

}  // namespace incremental
}  // namespace hydra
