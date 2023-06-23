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

#include <glog/logging.h>
#include <pcl/search/kdtree.h>
#include <voxblox/core/block_hash.h>

#include "hydra/common/hydra_config.h"
#include "hydra/rooms/room_finder.h"
#include "hydra/utils/minimum_spanning_tree.h"
#include "hydra/utils/timing_utilities.h"

namespace hydra {

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

BackendModule::BackendModule(const RobotPrefixConfig& prefix,
                             const BackendConfig& config,
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

  private_dsg_->graph->initMesh();
  original_vertices_.reset(new pcl::PointCloud<pcl::PointXYZRGBA>());
  setDefaultUpdateFunctions();
  deformation_graph_->setForceRecalculate(!config_.pgmo.gnc_fix_prev_inliers);

  if (config_.should_log) {
    const std::string log_path = config_.log_path + "/backend";
    backend_graph_logger_.setOutputPath(log_path);
    VLOG(1) << "[Hydra Backend] logging to " << log_path;
    backend_graph_logger_.setLayerName(DsgLayers::OBJECTS, "objects");
    backend_graph_logger_.setLayerName(DsgLayers::PLACES, "places");
    backend_graph_logger_.setLayerName(DsgLayers::ROOMS, "rooms");
    backend_graph_logger_.setLayerName(DsgLayers::BUILDINGS, "buildings");
  } else {
    VLOG(1) << "[Hydra Backend] logging disabled.";
  }

  if (config_.use_zmq_interface) {
    zmq_receiver_.reset(
        new spark_dsg::ZmqReceiver(config_.zmq_recv_url, config_.zmq_num_threads));
  }
}

BackendModule::~BackendModule() {
  VLOG(1) << "[Hydra Backend] destructor called!";
  stop();
}

void BackendModule::start() {
  spin_thread_.reset(new std::thread(&BackendModule::spin, this));

  if (config_.use_zmq_interface) {
    zmq_thread_.reset(new std::thread(&BackendModule::runZmqUpdates, this));
  }
  LOG(INFO) << "[Hydra Backend] started!";
}

void BackendModule::stop() {
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

void BackendModule::save(const std::string& output_path) {
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
    kimera_pgmo::WriteMeshWithStampsToPly(
        output_path + "/mesh.ply", private_dsg_->graph->getMesh(), mesh_timestamps_);
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

void BackendModule::spin() {
  bool should_shutdown = false;
  while (!should_shutdown) {
    bool has_data = state_->backend_queue.poll();
    if (HydraConfig::instance().force_shutdown() || !has_data) {
      // copy over shutdown request
      should_shutdown = should_shutdown_;
    }

    if (!has_data) {
      continue;
    }

    spinOnce(*state_->backend_queue.front(), false);
    state_->backend_queue.pop();
  }
}

bool BackendModule::spinOnce(bool force_update) {
  bool has_data = state_->backend_queue.poll();
  if (!has_data) {
    return false;
  }

  spinOnce(*state_->backend_queue.front(), force_update);
  state_->backend_queue.pop();
  return true;
}

void BackendModule::spinOnce(const BackendInput& input, bool force_update) {
  ScopedTimer spin_timer("backend/spin", input.timestamp_ns);
  status_.reset();

  updateFactorGraph(input);
  updateFromLcdQueue();
  status_.total_loop_closures_ = num_loop_closures_;

  if (!config_.use_mesh_subscribers) {
    copyMeshDelta(input);
  }

  if (!updatePrivateDsg(input.timestamp_ns, force_update)) {
    // we only read from the frontend dsg if we've processed all the
    // factor graph update packets (as long as force_update is false)
    // we still log the status for each received frontend packet
    logStatus();
    return;
  }

  if (have_loopclosures_) {
    double avg_distance = 0.0;
    size_t num_valid = 0;
    const auto& place_values = deformation_graph_->getGtsamTempValues();
    for (const auto& id_node_pair :
         private_dsg_->graph->getLayer(DsgLayers::PLACES).nodes()) {
      const auto node_id = id_node_pair.first;
      if (!place_values.exists(node_id)) {
        continue;
      }

      const auto& attrs = id_node_pair.second->attributes();
      const double dist =
          (attrs.position - place_values.at<gtsam::Pose3>(node_id).translation())
              .norm();
      avg_distance += dist;
      num_valid++;
    }
    if (num_valid) {
      avg_distance /= num_valid;
    }

    LOG(ERROR) << "Average distance: " << avg_distance << ", Num valid: " << num_valid
               << " / " << private_dsg_->graph->getLayer(DsgLayers::PLACES).numNodes();
  }

  if (config_.optimize_on_lc && have_loopclosures_) {
    optimize(input.timestamp_ns);
  } else {
    updateDsgMesh(input.timestamp_ns);
    callUpdateFunctions(input.timestamp_ns);
  }

  if (config_.pgmo.should_log) {
    logStatus();
  }

  for (const auto& cb_func : output_callbacks_) {
    cb_func(*private_dsg_->graph, *deformation_graph_, input.timestamp_ns);
  }
}

void BackendModule::loadState(const std::string& state_path,
                              const std::string& dgrf_path) {
  const std::string mesh_path = state_path + "/mesh.ply";

  pcl::PolygonMesh mesh;
  kimera_pgmo::ReadMeshWithStampsFromPly(mesh_path, mesh, &mesh_timestamps_);
  *private_dsg_->graph->getMeshFaces() = mesh.polygons;
  pcl::fromPCLPointCloud2(mesh.cloud, *private_dsg_->graph->getMeshVertices());

  have_new_mesh_ = true;

  loadDeformationGraphFromFile(dgrf_path);
  LOG(WARNING) << "Loaded " << deformation_graph_->getNumVertices()
               << " vertices for deformation graph";
}

void BackendModule::setUpdateFuncs(const std::list<LayerUpdateFunc>& update_funcs) {
  dsg_update_funcs_ = update_funcs;
}

void BackendModule::setSolverParams() {
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

void BackendModule::setDefaultUpdateFunctions() {
  update_objects_functor_.reset(new dsg_updates::UpdateObjectsFunctor());
  update_objects_functor_->angle_step = config_.angle_step;
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
    update_rooms_functor_->room_finder->enableLogging(config_.log_path +
                                                      "/room_filtrations");
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

std::string logPoseGraphConnections(const pose_graph_tools::PoseGraph& msg) {
  std::stringstream ss;
  ss << "nodes: [";
  auto iter = msg.nodes.begin();
  while (iter != msg.nodes.end()) {
    ss << "{r=" << iter->robot_id << ", k=" << iter->key << "}";
    ++iter;
    if (iter != msg.nodes.end()) {
      ss << ", ";
    }
  }

  ss << "], edges: [";
  auto eiter = msg.edges.begin();
  while (eiter != msg.edges.end()) {
    ss << eiter->robot_from << "(" << eiter->key_from << ") -> " << eiter->robot_to
       << "(" << eiter->key_to << ")";
    ++eiter;
    if (eiter != msg.edges.end()) {
      ss << ", ";
    }
  }
  ss << "]";

  return ss.str();
}

void BackendModule::updateFactorGraph(const BackendInput& input) {
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

    VLOG(5) << "[Hydra Backend] Adding pose graph message: "
            << logPoseGraphConnections(*msg);
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

bool BackendModule::updateFromLcdQueue() {
  bool added_new_loop_closure = false;
  while (!state_->backend_lcd_queue.empty()) {
    const auto result = state_->backend_lcd_queue.pop();

    // note that pose graph convention is pose = src.between(dest) where the edge
    // connects frames "to -> from" (i.e. src = to, dest = from, pose = to_T_from)
    LoopClosureLog lc{
        result.to_node, result.from_node, result.to_T_from, true, result.level};
    addLoopClosure(lc.src,
                   lc.dest,
                   lc.src_T_dest,
                   (result.level ? KimeraPgmoInterface::config_.lc_variance
                                 : config_.pgmo.sg_loop_closure_variance));

    loop_closures_.push_back(lc);

    added_new_loop_closure = true;
    have_loopclosures_ = true;
    have_new_loopclosures_ = true;
    num_loop_closures_++;
    status_.new_loop_closures_++;
  }

  return added_new_loop_closure;
}

void BackendModule::copyMeshDelta(const BackendInput& input) {
  ScopedTimer timer("backend/copy_mesh_delta", input.timestamp_ns);
  input.mesh_update->updateMesh(*private_dsg_->graph->getMeshVertices(),
                                mesh_timestamps_,
                                *private_dsg_->graph->getMeshFaces());
  input.mesh_update->updateVertices(*original_vertices_);
  // we use this to make sure that deformation only happens for vertices that are
  // still active
  num_archived_vertices_ = input.mesh_update->getTotalArchivedVertices();
  have_new_mesh_ = true;
}

bool BackendModule::updatePrivateDsg(size_t timestamp_ns, bool force_update) {
  std::unique_lock<std::mutex> graph_lock(private_dsg_->mutex);
  {                   // start joint critical section
    cachePlacePos();  // save place positions before grabbing new attributes from
                      // frontend

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

    const auto& objects = shared_dsg_->graph->getLayer(DsgLayers::OBJECTS);
    for (const auto& id_node_pair : objects.nodes()) {
      const auto node_opt = private_dsg_->graph->getNode(id_node_pair.first);
      if (!node_opt) {
        continue;
      }

      // TODO(nathan) we might need to think about checking the is_active flag here, but
      // no guarantee that mesh vertice aren't remapped under an inactive object node

      const auto& attrs = id_node_pair.second->attributes<ObjectNodeAttributes>();
      auto& private_attrs = node_opt->get().attributes<ObjectNodeAttributes>();
      private_attrs.mesh_connections = attrs.mesh_connections;
      private_attrs.is_active = attrs.is_active;
    }

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

    updatePlacePosFromCache();  // copy optimized positions back
  }                             // end joint critical section

  if (config_.should_log) {
    backend_graph_logger_.logGraph(private_dsg_->graph);
  }

  return true;
}

void BackendModule::cachePlacePos() {
  place_pos_cache_.clear();
  const auto& places = private_dsg_->graph->getLayer(DsgLayers::PLACES);
  for (const auto& id_node_pair : places.nodes()) {
    const auto& attributes = id_node_pair.second->attributes();
    if (attributes.is_active) {
      continue;
    }

    place_pos_cache_.emplace(id_node_pair.first, attributes.position);
  }
}

void BackendModule::updatePlacePosFromCache() {
  const auto& places = private_dsg_->graph->getLayer(DsgLayers::PLACES);
  for (const auto& id_node_pair : places.nodes()) {
    auto iter = place_pos_cache_.find(id_node_pair.first);
    if (iter == place_pos_cache_.end()) {
      continue;
    }

    id_node_pair.second->attributes().position = iter->second;
  }
}

void BackendModule::addPlacesToDeformationGraph(size_t timestamp_ns) {
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

void BackendModule::addLoopClosure(const gtsam::Key& src,
                                   const gtsam::Key& dest,
                                   const gtsam::Pose3& src_T_dest,
                                   double variance) {
  if (full_sparse_frame_map_.size() == 0 ||
      !KimeraPgmoInterface::config_.b_enable_sparsify) {
    deformation_graph_->addNewBetween(src, dest, src_T_dest, gtsam::Pose3(), variance);
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
    deformation_graph_->addNewBetween(
        sparse_src, sparse_dest, sparse_src_T_sparse_dest, gtsam::Pose3(), variance);
  }
}

void BackendModule::runZmqUpdates() {
  while (!should_shutdown_) {
    if (!zmq_receiver_->recv(config_.zmq_poll_time_ms)) {
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

void BackendModule::updateDsgMesh(size_t timestamp_ns, bool force_mesh_update) {
  if (!force_mesh_update && !have_new_mesh_) {
    return;
  }

  have_new_mesh_ = false;
  if (private_dsg_->graph->isMeshEmpty()) {
    return;
  }

  if (!force_mesh_update && !have_loopclosures_) {
    // we don't need to deform the mesh if we haven't found any loop closures yet
    // note that the first time we get a loop closure, we will deform the entire mesh
    // and cache the number of archived vertices and their values then
    return;
  }

  ScopedTimer timer("backend/mesh_deformation", timestamp_ns);
  VLOG(3) << "Deforming mesh with " << mesh_timestamps_.size() << " vertices";
  deformation_graph_->deformPoints(*private_dsg_->graph->getMeshVertices(),
                                   *original_vertices_,
                                   mesh_timestamps_,
                                   prefix_.vertex_key,
                                   deformation_graph_->getGtsamValues(),
                                   KimeraPgmoInterface::config_.num_interp_pts,
                                   KimeraPgmoInterface::config_.interp_horizon,
                                   nullptr,
                                   prev_num_archived_vertices_);
  prev_num_archived_vertices_ = num_archived_vertices_;
}

void BackendModule::optimize(size_t timestamp_ns) {
  if (config_.add_places_to_deformation_graph) {
    addPlacesToDeformationGraph(timestamp_ns);
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
  merge_handler_->reset();
  proposed_node_merges_.clear();
  {
    std::unique_lock<std::mutex> graph_lock(private_dsg_->mutex);
    // First reset private graph
    private_dsg_->graph->clear();
  }
  updatePrivateDsg(true);
  deformation_graph_->setRecalculateVertices();
  reset_backend_dsg_ = false;
}

void BackendModule::callUpdateFunctions(size_t timestamp_ns,
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

      const auto& sparse_T_dense =
          sparse_frames_.at(sparse_key).keyed_transforms.at(dense_key);
      gtsam::Pose3 agent_pose =
          pgmo_values.at<gtsam::Pose3>(sparse_key).compose(sparse_T_dense);
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

void BackendModule::logStatus(bool init) const {
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

void BackendModule::logIncrementalLoopClosures(const PoseGraph& msg) {
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

}  // namespace hydra
