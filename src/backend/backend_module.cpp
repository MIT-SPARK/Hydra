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
#include <config_utilities/types/eigen_matrix.h>
#include <config_utilities/types/enum.h>
#include <config_utilities/validation.h>
#include <glog/logging.h>
#include <kimera_pgmo/mesh_delta.h>
#include <kimera_pgmo/utils/mesh_io.h>
#include <spark_dsg/scene_graph_types.h>
#include <spark_dsg/zmq_interface.h>

#include "hydra/backend/backend_utilities.h"
#include "hydra/backend/update_agents_functor.h"
#include "hydra/backend/update_frontiers_functor.h"
#include "hydra/backend/update_objects_functor.h"
#include "hydra/backend/update_places_functor.h"
#include "hydra/backend/update_rooms_buildings_functor.h"
#include "hydra/common/config_utilities.h"
#include "hydra/common/global_info.h"
#include "hydra/rooms/room_finder.h"
#include "hydra/utils/minimum_spanning_tree.h"
#include "hydra/utils/pgmo_mesh_traits.h"
#include "hydra/utils/timing_utilities.h"

namespace hydra {

using hydra::timing::ScopedTimer;
using kimera_pgmo::DeformationGraph;
using kimera_pgmo::DeformationGraphPtr;
using kimera_pgmo::KimeraPgmoInterface;
using pose_graph_tools::PoseGraph;

void declare_config(BackendModule::Config& config) {
  using namespace config;
  name("BackendConfig");
  field(config.visualize_place_factors, "visualize_place_factors");
  field(config.enable_rooms, "enable_rooms");
  field(config.room_finder, "room_finder");
  field(config.enable_buildings, "enable_buildings");
  field(config.building_color, "building_color");
  field(config.building_semantic_label, "building_semantic_label");
  field(config.pgmo, "pgmo");
  field(config.use_2d_places, "use_2d_places");
  field(config.places2d_config, "places2d_config");

  enter_namespace("dsg");
  field(config.add_places_to_deformation_graph, "add_places_to_deformation_graph");
  field(config.optimize_on_lc, "optimize_on_lc");
  field(config.enable_node_merging, "enable_node_merging");
  field<LayerMapConversion<bool>>(config.merge_update_map, "merge_update_map");
  field(config.merge_update_dynamic, "merge_update_dynamic");
  field(config.places_merge_pos_threshold_m, "places_merge_pos_threshold_m");
  field(config.places_merge_distance_tolerance_m, "places_merge_distance_tolerance_m");
  field(config.use_mesh_subscribers, "use_mesh_subscribers");
  field(config.enable_merge_undos, "enable_merge_undos");
  field(config.use_active_flag_for_updates, "use_active_flag_for_updates");
  field(config.num_neighbors_to_find_for_merge, "num_neighbors_to_find_for_merge");
  field(config.zmq_send_url, "zmq_send_url");
  field(config.zmq_recv_url, "zmq_recv_url");
  field(config.use_zmq_interface, "use_zmq_interface");
  field(config.zmq_num_threads, "zmq_num_threads");
  field(config.zmq_poll_time_ms, "zmq_poll_time_ms");
  field(config.zmq_send_mesh, "zmq_send_mesh");
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

  setupDefaultFunctors();

  if (config.use_zmq_interface) {
    zmq_receiver_.reset(
        new spark_dsg::ZmqReceiver(config.zmq_recv_url, config.zmq_num_threads));
    zmq_sender_.reset(
        new spark_dsg::ZmqSender(config.zmq_send_url, config.zmq_num_threads));
  }
}

BackendModule::~BackendModule() { stopImpl(); }

void BackendModule::start() {
  spin_thread_.reset(new std::thread(&BackendModule::spin, this));

  if (config.use_zmq_interface) {
    zmq_thread_.reset(new std::thread(&BackendModule::runZmqUpdates, this));
  }
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

  if (zmq_thread_) {
    VLOG(2) << "[Hydra Backend] joining zmq thread and stopping";
    zmq_thread_->join();
    zmq_thread_.reset();
    VLOG(2) << "[Hydra Backend] stopped!";
  }

  VLOG(2) << "[Hydra Backend]: " << state_->backend_queue.size() << " messages left";
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
  std::stringstream ss;
  ss << config::toString(config);
  return ss.str();
}

void BackendModule::spin() {
  bool should_shutdown = false;
  while (!should_shutdown) {
    bool has_data = state_->backend_queue.poll();
    if (GlobalInfo::instance().force_shutdown() || !has_data) {
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
  status_.reset();
  std::lock_guard<std::mutex> lock(mutex_);

  ScopedTimer timer("backend/update", input.timestamp_ns);
  updateFactorGraph(input);
  updateFromLcdQueue();
  status_.total_loop_closures = num_loop_closures_;

  if (!config.use_mesh_subscribers) {
    copyMeshDelta(input);
  }

  if (!updatePrivateDsg(input.timestamp_ns, force_update)) {
    VLOG(2) << "Backend skipping input @ " << input.timestamp_ns << " [ns]";
    // we only read from the frontend dsg if we've processed all the
    // factor graph update packets (as long as force_update is false)
    // we still log the status for each received frontend packet
    logStatus();
    return;
  }

  timer.reset("backend/spin"); 
  if (config.optimize_on_lc && have_loopclosures_) {
    optimize(input.timestamp_ns);
  } else {
    updateDsgMesh(input.timestamp_ns);
    callUpdateFunctions(input.timestamp_ns);
  }

  if (logs_) {
    logStatus();
  }

  if (zmq_sender_) {
    zmq_sender_->send(*private_dsg_->graph, config.zmq_send_mesh);
  }
  ScopedTimer sink_timer("backend/sinks", input.timestamp_ns);
  Sink::callAll(sinks_, input.timestamp_ns, *private_dsg_->graph, *deformation_graph_);
}

void BackendModule::loadState(const std::string& state_path,
                              const std::string& dgrf_path) {
  const std::string mesh_path = state_path + "/mesh.ply";

  auto mesh = std::make_shared<spark_dsg::Mesh>();
  kimera_pgmo::ReadMesh(mesh_path, *mesh);
  private_dsg_->graph->setMesh(mesh);
  have_new_mesh_ = true;

  loadDeformationGraphFromFile(dgrf_path);
  LOG(WARNING) << "Loaded " << deformation_graph_->getNumVertices()
               << " vertices for deformation graph";
}

void BackendModule::addSink(const Sink::Ptr& sink) {
  if (sink) {
    sinks_.push_back(sink);
  }
}

void BackendModule::setUpdateFunctor(LayerId layer, const UpdateFunctor::Ptr& functor) {
  layer_functors_[layer] = functor;
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

void BackendModule::setupDefaultFunctors() {
  layer_functors_[DsgLayers::OBJECTS] = std::make_shared<UpdateObjectsFunctor>();

  layer_functors_[DsgLayers::PLACES] = std::make_shared<UpdatePlacesFunctor>(
      config.places_merge_pos_threshold_m, config.places_merge_distance_tolerance_m);

  if (config.enable_rooms) {
    auto room_functor = std::make_shared<UpdateRoomsFunctor>(config.room_finder);
    if (logs_) {
      const auto log_path = logs_->getLogDir("backend/room_filtrations");
      room_functor->room_finder->enableLogging(log_path);
    }

    layer_functors_[DsgLayers::ROOMS] = room_functor;
  }

  if (config.enable_buildings) {
    layer_functors_[DsgLayers::BUILDINGS] = std::make_shared<UpdateBuildingsFunctor>(
        config.building_color, config.building_semantic_label);
  }
}

inline bool hasLoopClosure(const PoseGraph& graph) {
  for (const auto& edge : graph.edges) {
    if (edge.type == pose_graph_tools::PoseGraphEdge::Type::LOOPCLOSE) {
      return true;
    }
  }
  return false;
}

void BackendModule::updateFactorGraph(const BackendInput& input) {
  ScopedTimer timer("backend/process_factors", input.timestamp_ns);
  const size_t prev_loop_closures = num_loop_closures_;

  if (!input.deformation_graph) {
    LOG(WARNING) << "[Hydra Backend] Received invalid deformation graph";
    return;
  }

  status_.new_graph_factors = input.deformation_graph->edges.size();
  status_.new_factors += input.deformation_graph->edges.size();

  try {
    processIncrementalMeshGraph(
        *input.deformation_graph, timestamps_, unconnected_nodes_);
  } catch (const gtsam::ValuesKeyDoesNotExist& e) {
    LOG(ERROR) << *input.deformation_graph;
    throw std::logic_error(e.what());
  }

  for (const auto& msg : input.agent_updates.pose_graphs) {
    status_.new_factors += msg->edges.size();

    VLOG(5) << "[Hydra Backend] Adding pose graph message: " << *msg;
    if (hasLoopClosure(*msg)) {
      LOG(INFO) << "[Hydra Backend] Input pose graph has loop closure @ "
                << msg->stamp_ns << " [ns]";
    }

    processIncrementalPoseGraph(*msg, trajectory_, timestamps_, unconnected_nodes_);
    logIncrementalLoopClosures(*msg);
  }

  if (input.agent_updates.external_priors) {
    updateAgentNodeMeasurements(*input.agent_updates.external_priors);
    // Think of it as "implicit" loop closures
    have_loopclosures_ = true;
    have_new_loopclosures_ = true;
  }

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
  while (!state_->backend_lcd_queue.empty()) {
    const auto result = state_->backend_lcd_queue.pop();

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
    if (!force_update && shared_dsg.last_update_time != timestamp_ns) {
      return false;
    }

    unmerged_graph_->mergeGraph(*shared_dsg.graph);
  }  // end joint critical section

  if (logs_) {
    backend_graph_logger_.logGraph(private_dsg_->graph);
  }

  return true;
}

void BackendModule::addPlacesToDeformationGraph(size_t timestamp_ns) {
  const auto& places = unmerged_graph_->getLayer(DsgLayers::PLACES);
  if (places.nodes().empty()) {
    LOG(WARNING) << "Attempting to add places to deformation graph without places";
    return;
  }

  ScopedTimer timer("backend/add_places", timestamp_ns);
  const auto& prefix = GlobalInfo::instance().getRobotPrefix();

  deformation_graph_->clearTemporaryStructures();

  MinimumSpanningTreeInfo mst_info;
  {  // start timing scope
    ScopedTimer mst_timer("backend/places_mst", timestamp_ns);
    mst_info = getMinimumSpanningEdges(places);
  }  // end timing scope

  {  // start timing scope
    ScopedTimer add_timer("backend/add_places_nodes", timestamp_ns);

    std::vector<gtsam::Key> place_nodes;
    std::vector<gtsam::Pose3> place_node_poses;
    std::vector<std::vector<size_t>> place_node_valences;

    for (const auto& id_node_pair : places.nodes()) {
      const auto& node = *id_node_pair.second;
      const auto& attrs = node.attributes<PlaceNodeAttributes>();

      if (!node.hasSiblings()) {
        continue;
      }

      place_nodes.push_back(node.id);
      place_node_poses.push_back(gtsam::Pose3(gtsam::Rot3(), attrs.position));

      if (mst_info.leaves.count(node.id)) {
        std::vector<size_t> valid_connections;
        for (const auto& idx : attrs.deformation_connections) {
          if (idx == std::numeric_limits<size_t>::max()) {
            continue;
          }
          valid_connections.push_back(idx);
        }

        place_node_valences.push_back(valid_connections);
      } else {
        place_node_valences.push_back(std::vector<size_t>{});
      }
    }

    deformation_graph_->addNewTempNodesValences(place_nodes,
                                                place_node_poses,
                                                place_node_valences,
                                                prefix.vertex_key,
                                                false,
                                                config.pgmo.place_mesh_variance);
  }  // end timing scope

  {  // start timing scope
    ScopedTimer between_timer("backend/add_places_between", timestamp_ns);
    PoseGraph mst_edges;
    for (const auto& edge : mst_info.edges) {
      gtsam::Pose3 source(gtsam::Rot3(), places.getPosition(edge.source));
      gtsam::Pose3 target(gtsam::Rot3(), places.getPosition(edge.target));
      pose_graph_tools::PoseGraphEdge mst_e;
      mst_e.key_from = edge.source;
      mst_e.key_to = edge.target;
      mst_e.pose = source.between(target).matrix();
      mst_edges.edges.push_back(mst_e);
    }
    deformation_graph_->addNewTempEdges(mst_edges, config.pgmo.place_edge_variance);
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
    if (!zmq_receiver_->recv(config.zmq_poll_time_ms)) {
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

      auto node_opt = private_dsg_->graph->findNode(id_node_pair.first);
      if (!node_opt) {
        VLOG(2) << "received update for node "
                << NodeSymbol(id_node_pair.first).getLabel()
                << " but node no longer exists";
        continue;
      }

      VLOG(2) << "assiging name " << new_name << " to "
              << NodeSymbol(id_node_pair.first).getLabel();
      node_opt->attributes<SemanticNodeAttributes>().name = new_name;
    }
  }
}

void BackendModule::updateDsgMesh(size_t timestamp_ns, bool force_mesh_update) {
  // deformation_graph_->update();
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

void BackendModule::updateAgentNodeMeasurements(
    const pose_graph_tools::PoseGraph& meas) {
  deformation_graph_->removePriorsWithPrefix(
      GlobalInfo::instance().getRobotPrefix().key);
  std::vector<std::pair<gtsam::Key, gtsam::Pose3>> agent_measurements;
  for (const auto& node : meas.nodes) {
    agent_measurements.push_back(
        {gtsam::Symbol(GlobalInfo::instance().getRobotPrefix().key, node.key),
         gtsam::Pose3(node.pose.matrix())});
  }
  deformation_graph_->addNodeMeasurements(agent_measurements);
}

void BackendModule::optimize(size_t timestamp_ns) {
  if (config.add_places_to_deformation_graph) {
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
                                        const UpdateInfo::LayerMerges& given_merges) {
  ScopedTimer spin_timer("backend/update_layers", timestamp_ns);

  // TODO(nathan) chance that this causes weirdness when we have multiple nodes but no
  // accepted reconciliation merges
  const bool enable_merging = given_merges.empty() ? config.enable_node_merging : false;

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

  UpdateInfo::ConstPtr info(new UpdateInfo{&places_values,
                                           &pgmo_values,
                                           new_loop_closure,
                                           timestamp_ns,
                                           enable_merging,
                                           given_merges,
                                           &complete_agent_values});

  // merge topological changes to private dsg, respecting merges
  // attributes may be overwritten, but ideally we don't bother
  GraphMergeConfig config;
  config.previous_merges = &private_dsg_->merges;
  config.update_dynamic_attributes = false;
  private_dsg_->graph->mergeGraph(*unmerged_graph_, config);

  if (agent_functor_) {
    agent_functor_->call(*unmerged_graph_, *private_dsg_, info);
  }

  std::list<LayerCleanupFunc> cleanup_hooks;
  for (const auto& [layer, functor] : layer_functors_) {
    if (!functor) {
      continue;
    }

    const auto merges = functor->call(*unmerged_graph_, *private_dsg_, info);
    const auto hooks = functor->hooks();
    merge_tracker.applyMerges(*unmerged_graph_, merges, *private_dsg_, hooks.merge);
    if (hooks.cleanup) {
      cleanup_hooks.push_back(hooks.cleanup);
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

void BackendModule::labelRooms(const UpdateInfo&, SharedDsgInfo* dsg) {
  if (!dsg) {
    return;
  }

  std::unique_lock<std::mutex> lock(dsg->mutex);
  const auto& rooms = dsg->graph->getLayer(DsgLayers::ROOMS);
  for (auto& id_node_pair : rooms.nodes()) {
    const auto iter = room_name_map_.find(id_node_pair.first);
    if (iter == room_name_map_.end()) {
      continue;
    }

    id_node_pair.second->attributes<SemanticNodeAttributes>().name = iter->second;
  }
}

}  // namespace hydra
