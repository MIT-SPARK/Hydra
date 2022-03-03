#include "kimera_dsg_builder/incremental_dsg_backend.h"
#include "kimera_dsg_builder/configs.h"
#include "kimera_dsg_builder/minimum_spanning_tree.h"

#include <hydra_utils/timing_utilities.h>
#include <kimera_dsg/node_attributes.h>
#include <pcl/search/kdtree.h>
#include <voxblox/core/block_hash.h>

#include <glog/logging.h>

namespace kimera {
namespace incremental {

using hydra::timing::ScopedTimer;
using kimera_pgmo::DeformationGraph;
using kimera_pgmo::DeformationGraphPtr;
using kimera_pgmo::KimeraPgmoInterface;
using kimera_pgmo::KimeraPgmoMesh;
using kimera_pgmo::Path;
using pose_graph_tools::PoseGraph;

void DsgBackend::setSolverParams() {
  KimeraRPGO::RobustSolverParams params = deformation_graph_->getParams();
  params.verbosity = config_.pgmo.rpgo_verbosity;
  params.solver = config_.pgmo.rpgo_solver;
  ;
  if (config_.pgmo.should_log) {
    params.logOutput(config_.pgmo.log_path);
    logStatus(true);
  }
  deformation_graph_->setParams(params);
  setVerboseFlag(false);
}

DsgBackend::DsgBackend(const ros::NodeHandle nh,
                       const SharedDsgInfo::Ptr& dsg,
                       const SharedDsgInfo::Ptr& backend_dsg)
    : KimeraPgmoInterface(),
      nh_(nh),
      shared_dsg_(dsg),
      private_dsg_(backend_dsg),
      shared_places_copy_(KimeraDsgLayers::PLACES),
      robot_id_(0) {
  config_ = load_config<DsgBackendConfig>(nh_);

  nh_.getParam("robot_id", robot_id_);
  if (!loadParameters(ros::NodeHandle(nh_, "pgmo"))) {
    ROS_FATAL("Failed to initialize pgmo parameters!");
    throw std::runtime_error("pgmo parameter parsing failed");
  }

  robot_prefix_ = kimera_pgmo::robot_id_to_prefix.at(robot_id_);
  robot_vertex_prefix_ = kimera_pgmo::robot_id_to_vertex_prefix.at(robot_id_);

  if (config_.enable_rooms) {
    room_finder_.reset(new RoomFinder(config_.room_finder));
  }

  dsg_update_funcs_.push_back(&dsg_updates::updateAgents);
  dsg_update_funcs_.push_back(&dsg_updates::updateObjects);
  dsg_update_funcs_.push_back([&](auto& graph,
                                  const auto& place_values,
                                  const auto& pgmo_values,
                                  bool allow_merging) {
    dsg_updates::updatePlaces(graph,
                              place_values,
                              pgmo_values,
                              allow_merging,
                              config_.places_merge_pos_threshold_m,
                              config_.places_merge_distance_tolerance_m);
  });

  deformation_graph_->setForceRecalculate(!config_.pgmo.gnc_fix_prev_inliers);
  deformation_graph_->storeOnlyNoOptimization();

  if (config_.should_log) {
    backend_graph_logger_.setOutputPath(config_.log_path + "/backend");
    ROS_INFO("Logging backend graph to %s", (config_.log_path + "/backend").c_str());
    backend_graph_logger_.setLayerName(KimeraDsgLayers::OBJECTS, "objects");
    backend_graph_logger_.setLayerName(KimeraDsgLayers::PLACES, "places");
    backend_graph_logger_.setLayerName(KimeraDsgLayers::ROOMS, "rooms");
    backend_graph_logger_.setLayerName(KimeraDsgLayers::BUILDINGS, "buildings");
  } else {
    ROS_ERROR("DSG Backend logging disabled. ");
  }

  last_timestamp_ = 0;
}

void DsgBackend::stop() {
  LOG(INFO) << "[DSG Backend] stopping!";
  should_opt_shutdown_ = true;

  VLOG(2) << " [DSG Backend] joining optimizer thread";
  if (optimizer_thread_) {
    optimizer_thread_->join();
    optimizer_thread_.reset();
  }
  VLOG(2) << " [DSG Backend] joined optimizer thread";

  should_viz_shutdown_ = true;

  VLOG(2) << "[DSG Backend] joining visualizer thread";
  if (visualizer_thread_) {
    visualizer_thread_->join();
    visualizer_thread_.reset();
  }
  VLOG(2) << " [DSG Backend] joined visualized thread";

  visualizer_.reset();
}

DsgBackend::~DsgBackend() {
  LOG(INFO) << " [DSG Backend] destructor called!";
  stop();
}

void DsgBackend::start() {
  startPgmo();
  startVisualizer();
  LOG(INFO) << " [DSG Backend] started!";
}

void DsgBackend::startVisualizer() {
  std::string visualizer_ns;
  nh_.param<std::string>("visualizer_ns", visualizer_ns, "/kimera_dsg_visualizer");

  visualizer_queue_.reset(new ros::CallbackQueue());

  ros::NodeHandle nh(visualizer_ns);
  nh.setCallbackQueue(visualizer_queue_.get());

  visualizer_.reset(new DynamicSceneGraphVisualizer(nh, getDefaultLayerIds()));

  bool use_voxblox_mesh_plugin;
  nh_.param<bool>("use_voxblox_mesh_plugin", use_voxblox_mesh_plugin, false);
  if (use_voxblox_mesh_plugin) {
    // TODO(nathan) voxblox mesh plugin in rviz doesn't handle large graphs well
    visualizer_->addPlugin(std::make_shared<VoxbloxMeshPlugin>(nh, "dsg_mesh"));
  } else {
    visualizer_->addPlugin(std::make_shared<PgmoMeshPlugin>(nh, "dsg_mesh"));
  }

  bool use_parent_plugin;
  nh_.param<bool>("use_parent_plugin", use_parent_plugin, false);
  if (use_parent_plugin) {
    visualizer_->addPlugin(std::make_shared<PlaceParentsPlugin>(nh, "parents"));
  }

  bool use_mst_plugin;
  nh_.param<bool>("use_mst_plugin", use_mst_plugin, false);
  if (use_mst_plugin) {
    // note that this only makes sense for the frontend graph
    visualizer_->addPlugin(std::make_shared<MeshPlaceConnectionsPlugin>(nh, "mst"));
  }

  visualizer_should_reset_ = true;

  bool show_frontend_dsg;
  nh_.param<bool>("show_frontend_dsg", show_frontend_dsg, false);
  visualizer_show_frontend_ = show_frontend_dsg;

  frontend_viz_srv_ = nh.advertiseService(
      "visualize_frontend_dsg", &DsgBackend::setVisualizeFrontend, this);
  backend_viz_srv_ = nh.advertiseService(
      "visualize_backend_dsg", &DsgBackend::setVisualizeBackend, this);

  visualizer_thread_.reset(new std::thread(&DsgBackend::runVisualizer, this));
}

bool DsgBackend::setVisualizeFrontend(std_srvs::Empty::Request&,
                                      std_srvs::Empty::Response&) {
  if (!visualizer_show_frontend_) {
    visualizer_should_reset_ = true;
  }

  visualizer_show_frontend_ = true;
  return true;
}

bool DsgBackend::setVisualizeBackend(std_srvs::Empty::Request&,
                                     std_srvs::Empty::Response&) {
  if (visualizer_show_frontend_) {
    visualizer_should_reset_ = true;
  }

  visualizer_show_frontend_ = false;
  return true;
}

void DsgBackend::runVisualizer() {
  ros::WallRate r(5);
  while (ros::ok() && !should_viz_shutdown_) {
    // process any config changes
    visualizer_queue_->callAvailable(ros::WallDuration(0));

    if (visualizer_should_reset_) {
      if (visualizer_show_frontend_) {
        std::unique_lock<std::mutex> lock(shared_dsg_->mutex);
        visualizer_->setGraph(shared_dsg_->graph);
      } else {
        std::unique_lock<std::mutex> lock(private_dsg_->mutex);
        visualizer_->setGraph(private_dsg_->graph);
      }
      visualizer_should_reset_ = false;
    }

    // TODO(nathan) this is janky, avoid weird updated / redraw split
    if (private_dsg_->updated) {
      // the frontend dsg update flag propagates to the backend flag, so we always check
      // the backend flag to see if we need to redraw
      visualizer_->setGraphUpdated();
      private_dsg_->updated = false;
      VLOG(5) << "Visualizer caught latched update!";
    }

    if (visualizer_show_frontend_) {
      std::unique_lock<std::mutex> graph_lock(shared_dsg_->mutex);
      visualizer_->redraw();
    } else {
      std::unique_lock<std::mutex> graph_lock(private_dsg_->mutex);
      visualizer_->redraw();
    }

    r.sleep();
  }
}

bool DsgBackend::updatePrivateDsg() {
  std::unique_lock<std::mutex> graph_lock(private_dsg_->mutex);
  bool have_frontend_updates = shared_dsg_->updated;
  if (have_frontend_updates) {
    {  // start joint critical section
      std::unique_lock<std::mutex> shared_graph_lock(shared_dsg_->mutex);
      // private_dsg_->updated = false;
      private_dsg_->graph->mergeGraph(*shared_dsg_->graph,
                                      false,
                                      true,
                                      &config_.merge_update_map,
                                      config_.merge_update_dynamic);
      *private_dsg_->latest_places = *shared_dsg_->latest_places;

      if (shared_dsg_->graph->hasLayer(KimeraDsgLayers::PLACES)) {
        const auto& places = shared_dsg_->graph->getLayer(KimeraDsgLayers::PLACES);
        shared_places_copy_.mergeLayer(places);
        std::vector<NodeId> removed_place_nodes;
        places.getRemovedNodes(&removed_place_nodes);
        for (const auto& place_id : removed_place_nodes) {
          shared_places_copy_.removeNode(place_id);
        }
      }
      shared_dsg_->updated = false;
    }  // end joint critical section

    if (config_.should_log) {
      backend_graph_logger_.logGraph(private_dsg_->graph);
    }
  }

  return have_frontend_updates;
}

void DsgBackend::startPgmo() {
  full_mesh_sub_ =
      nh_.subscribe("pgmo/full_mesh", 1, &DsgBackend::fullMeshCallback, this);
  deformation_graph_sub_ = nh_.subscribe(
      "pgmo/mesh_graph_incremental", 1000, &DsgBackend::deformationGraphCallback, this);
  pose_graph_sub_ = nh_.subscribe(
      "pose_graph_incremental", 10000, &DsgBackend::poseGraphCallback, this);
  // TODO(yun) with lower queue size we drop msgs, esp with permissive dsg lcd

  viz_mesh_mesh_edges_pub_ = nh_.advertise<visualization_msgs::Marker>(
      "pgmo/deformation_graph_mesh_mesh", 10, false);
  viz_pose_mesh_edges_pub_ = nh_.advertise<visualization_msgs::Marker>(
      "pgmo/deformation_graph_pose_mesh", 10, false);
  pose_graph_pub_ = nh_.advertise<PoseGraph>("pgmo/pose_graph", 10, false);

  if (config_.visualize_place_factors) {
    places_factors_visualizer_ = std::make_shared<PlacesFactorGraphViz>(nh_);
  }

  save_mesh_srv_ =
      nh_.advertiseService("save_mesh", &DsgBackend::saveMeshCallback, this);

  // Initialize save trajectory service
  save_traj_srv_ = nh_.advertiseService(
      "save_trajectory", &DsgBackend::saveTrajectoryCallback, this);

  optimizer_thread_.reset(new std::thread(&DsgBackend::runPgmo, this));
}

void DsgBackend::logIncrementalLoopClosures(const PoseGraph& msg) {
  for (const auto& edge : msg.edges) {
    if (edge.type != pose_graph_tools::PoseGraphEdge::LOOPCLOSE) {
      continue;
    }

    gtsam::Pose3 pose = kimera_pgmo::RosToGtsam(edge.pose);
    const gtsam::Symbol src_key(robot_prefix_, edge.key_from);
    const gtsam::Symbol dest_key(robot_prefix_, edge.key_to);
    // note that pose graph convention is pose = src.between(dest) where the edge
    // connects frames "to -> from" (i.e. src = to, dest = from, pose = to_T_from)
    loop_closures_.push_back({src_key, dest_key, pose, false, 0});
  }
}

PoseGraph::ConstPtr DsgBackend::popDeformationGraphQueue() {
  std::unique_lock<std::mutex> lock(pgmo_mutex_);
  if (deformation_graph_updates_.empty()) {
    return nullptr;
  }

  const auto msg = deformation_graph_updates_.front();
  deformation_graph_updates_.pop();
  return msg;
}

PoseGraph::ConstPtr DsgBackend::popAgentGraphQueue() {
  std::unique_lock<std::mutex> lock(pgmo_mutex_);
  if (pose_graph_updates_.empty()) {
    return nullptr;
  }

  const auto msg = pose_graph_updates_.front();
  pose_graph_updates_.pop();
  return msg;
}

bool DsgBackend::readPgmoUpdates() {
  bool have_updates = false;
  ScopedTimer spin_timer("backend/process_factors", last_timestamp_);

  status_.new_graph_factors_ = 0;

  PoseGraph::ConstPtr msg;
  while ((msg = popDeformationGraphQueue()) != nullptr) {
    status_.new_graph_factors_ += msg->edges.size();
    status_.new_factors_ += msg->edges.size();
    processIncrementalMeshGraph(msg, timestamps_, &unconnected_nodes_);
    have_updates = true;
  }

  while ((msg = popAgentGraphQueue()) != nullptr) {
    status_.new_factors_ += msg->edges.size();
    processIncrementalPoseGraph(msg, &trajectory_, &unconnected_nodes_, &timestamps_);
    logIncrementalLoopClosures(*msg);
    have_updates = true;
  }

  have_updates |= addInternalLCDToDeformationGraph();
  return have_updates;
}

void DsgBackend::runPgmo() {
  ros::WallRate r(10);
  while (ros::ok()) {
    status_.reset();
    ScopedTimer spin_timer("backend/spin", last_timestamp_);
    const size_t prev_loop_closures = num_loop_closures_;

    bool have_graph_updates = readPgmoUpdates();

    if (num_loop_closures_ > prev_loop_closures) {
      LOG(WARNING) << "New loop closures detected!";
    }

    if (num_loop_closures_ > 0) {
      status_.total_loop_closures_ = num_loop_closures_;
      status_.new_loop_closures_ = num_loop_closures_ - prev_loop_closures;
      have_loopclosures_ = true;
    }
    status_.trajectory_len_ = trajectory_.size();
    status_.total_factors_ = deformation_graph_->getGtsamFactors().size();
    status_.total_values_ = deformation_graph_->getGtsamValues().size();

    bool have_dsg_updates = false;
    bool was_updated = false;
    {  // start pgmo mesh critical section
      std::unique_lock<std::mutex> pgmo_lock(pgmo_mutex_);
      have_dsg_updates = updatePrivateDsg();

      // TODO(nathan) move to helper
      if (config_.visualize_place_factors && (have_dsg_updates || have_graph_updates)) {
        MinimumSpanningTreeInfo mst_info = getMinimumSpanningEdges(shared_places_copy_);
        places_factors_visualizer_->draw(
            robot_vertex_prefix_, shared_places_copy_, mst_info, *deformation_graph_);
      }

      if (config_.optimize_on_lc && have_graph_updates && have_loopclosures_) {
        optimize();
        was_updated = true;
      } else if (config_.call_update_periodically && have_dsg_updates) {
        updateDsgMesh();
        callUpdateFunctions();
        was_updated = true;
      }
    }  // end pgmo mesh critical section

    if (have_graph_updates && config_.pgmo.should_log) {
      logStatus();
    }

    if (was_updated) {
      VLOG(5) << "Backend flipped flag!";
      private_dsg_->updated = true;
    } else {
      r.sleep();
    }

    if (should_opt_shutdown_ && !have_graph_updates && !have_dsg_updates) {
      break;
    }
  }

  // TODO(nathan) figure this out instead of forcing an update before exiting
  updateDsgMesh();
  callUpdateFunctions();
  // TODO(Yun) Technically not strictly a g2o
  deformation_graph_->save(config_.pgmo.log_path + "/deformation_graph.dgrf");
}

void DsgBackend::fullMeshCallback(const KimeraPgmoMesh::ConstPtr& msg) {
  std::unique_lock<std::mutex> lock(pgmo_mutex_);
  latest_mesh_ = msg;
  have_new_mesh_ = true;
}

void DsgBackend::deformationGraphCallback(const PoseGraph::ConstPtr& msg) {
  std::unique_lock<std::mutex> lock(pgmo_mutex_);
  deformation_graph_updates_.push(msg);
  last_timestamp_ = msg->header.stamp.toNSec();
}

void DsgBackend::poseGraphCallback(const PoseGraph::ConstPtr& msg) {
  std::unique_lock<std::mutex> lock(pgmo_mutex_);
  pose_graph_updates_.push(msg);
}

bool DsgBackend::saveMeshCallback(std_srvs::Empty::Request&,
                                  std_srvs::Empty::Response&) {
  pcl::PolygonMesh opt_mesh;
  {
    std::unique_lock<std::mutex> graph_lock(private_dsg_->mutex);
    opt_mesh = private_dsg_->graph->getMesh();
  }
  // Save mesh
  std::string ply_name = config_.pgmo.log_path + std::string("/mesh_pgmo.ply");
  saveMesh(opt_mesh, ply_name);
  return true;
}

bool DsgBackend::saveTrajectoryCallback(std_srvs::Empty::Request&,
                                        std_srvs::Empty::Response&) {
  Path optimized_path;
  {
    std::unique_lock<std::mutex> pgmo_lock(pgmo_mutex_);
    optimized_path = getOptimizedTrajectory(robot_id_);
  }
  // Save trajectory
  std::string csv_name = config_.pgmo.log_path + std::string("/traj_pgmo.csv");
  saveTrajectory(optimized_path, timestamps_, csv_name);
  return true;
}

void DsgBackend::addPlacesToDeformationGraph() {
  if (shared_places_copy_.nodes().empty()) {
    LOG(WARNING) << "Attempting to add places to deformation graph with empty "
                    "places layer";
    return;
  }

  ScopedTimer spin_timer("backend/add_places", last_timestamp_);

  deformation_graph_->clearTemporaryStructures();

  MinimumSpanningTreeInfo mst_info;
  {
    ScopedTimer spin_timer("backend/places_mst", last_timestamp_);
    mst_info = getMinimumSpanningEdges(shared_places_copy_);
  }

  {
    ScopedTimer spin_timer("backend/add_places_nodes", last_timestamp_);

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
        place_node_valences.push_back(attrs.pcl_mesh_connections);
      } else {
        place_node_valences.push_back(std::vector<size_t>{});
      }
    }

    deformation_graph_->addNewTempNodesValences(place_nodes,
                                                place_node_poses,
                                                place_node_valences,
                                                robot_vertex_prefix_,
                                                false,
                                                config_.pgmo.place_mesh_variance);
  }

  {
    ScopedTimer spin_timer("backend/add_places_between", last_timestamp_);
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
  }
}

bool DsgBackend::addInternalLCDToDeformationGraph() {
  std::list<LoopClosureLog> to_process;

  {  // start critical section
    std::unique_lock<std::mutex> lock(shared_dsg_->lcd_mutex);
    while (!shared_dsg_->loop_closures.empty()) {
      auto result = shared_dsg_->loop_closures.front();
      shared_dsg_->loop_closures.pop();

      // TODO(nathan) this is kinda ugly, we can probably grab the GTSAM symbol in the
      // frontend and pass it with the result
      NodeId from_key;
      NodeId to_key;
      const auto& from_attrs = shared_dsg_->graph->getDynamicNode(result.from_node)
                                   .value()
                                   .get()
                                   .attributes<AgentNodeAttributes>();
      from_key = from_attrs.external_key;
      const auto& to_attrs = shared_dsg_->graph->getDynamicNode(result.to_node)
                                 .value()
                                 .get()
                                 .attributes<AgentNodeAttributes>();
      to_key = to_attrs.external_key;

      // note that pose graph convention is pose = src.between(dest) where the edge
      // connects frames "to -> from" (i.e. src = to, dest = from, pose = to_T_from)
      loop_closures_.push_back(
          {result.to_node, result.from_node, result.to_T_from, true, result.level});
      to_process.push_back({to_key, from_key, result.to_T_from, true, result.level});
    }
  }  // end critical section

  bool added_new_loop_closure = false;
  for (const auto& lc : to_process) {
    deformation_graph_->addNewBetween(
        lc.src, lc.dest, lc.src_T_dest, gtsam::Pose3(), lc_variance_);
    added_new_loop_closure = true;
    num_loop_closures_++;

    have_loopclosures_ = true;
  }

  return added_new_loop_closure;
}

void DsgBackend::updateDsgMesh(bool force_mesh_update) {
  // avoid scope problems by using a smart pointer
  std::unique_ptr<ScopedTimer> timer;

  pcl::PolygonMesh input_mesh;
  if (!latest_mesh_) {
    return;
  }

  if (!force_mesh_update && !have_new_mesh_) {
    return;
  }

  timer.reset(new ScopedTimer("backend/mesh_update", last_timestamp_));
  input_mesh =
      kimera_pgmo::PgmoMeshMsgToPolygonMesh(*latest_mesh_, &mesh_vertex_stamps_);
  have_new_mesh_ = false;

  if (input_mesh.cloud.height * input_mesh.cloud.width == 0) {
    return;
  }
  VLOG(3) << "Deforming mesh with " << mesh_vertex_stamps_.size() << " vertices";

  auto opt_mesh = deformation_graph_->deformMesh(input_mesh,
                                                 mesh_vertex_stamps_,
                                                 robot_vertex_prefix_,
                                                 num_interp_pts_,
                                                 interp_horizon_);
  {
    // start private dsg critical section
    std::unique_lock<std::mutex> graph_lock(private_dsg_->mutex);
    private_dsg_->graph->setMeshDirectly(opt_mesh);
  }

  if (viz_mesh_mesh_edges_pub_.getNumSubscribers() > 0 ||
      viz_pose_mesh_edges_pub_.getNumSubscribers() > 0) {
    visualizeDeformationGraphEdges();
  }
}

void DsgBackend::optimize() {
  if (config_.add_places_to_deformation_graph) {
    addPlacesToDeformationGraph();
  }

  {  // timer scope
    ScopedTimer timer("backend/optimization", last_timestamp_, true, 0, false);
    deformation_graph_->optimize();
  }  // timer scope

  updateDsgMesh(true);

  gtsam::Values pgmo_values = deformation_graph_->getGtsamValues();
  gtsam::Values places_values = deformation_graph_->getGtsamTempValues();

  callUpdateFunctions(places_values, pgmo_values);

  if (pose_graph_pub_.getNumSubscribers() > 0) {
    visualizePoseGraph();
  }
}

void DsgBackend::callUpdateFunctions(const gtsam::Values& places_values,
                                     const gtsam::Values& pgmo_values) {
  ScopedTimer spin_timer("backend/update_layers", last_timestamp_);
  {  // start private dsg critical section
    std::unique_lock<std::mutex> graph_lock(private_dsg_->mutex);
    for (const auto& update_func : dsg_update_funcs_) {
      update_func(*private_dsg_->graph,
                  places_values,
                  pgmo_values,
                  config_.enable_node_merging);
    }
  }  // end private dsg critical section
  updateRoomsNodes();
  updateBuildingNode();
}

ActiveNodeSet DsgBackend::getNodesForRoomDetection(const NodeIdSet& latest_places) {
  std::unordered_set<NodeId> active_places(latest_places.begin(), latest_places.end());
  // TODO(nathan) grab this from a set of active rooms
  const auto& rooms = private_dsg_->graph->getLayer(KimeraDsgLayers::ROOMS);
  for (const auto& id_node_pair : rooms.nodes()) {
    active_places.insert(id_node_pair.second->children().begin(),
                         id_node_pair.second->children().end());
  }

  // TODO(nathan) this is threadsafe as long as places and rooms are on the same thread
  const auto& places = private_dsg_->graph->getLayer(KimeraDsgLayers::PLACES);
  for (const auto& node_id : unlabeled_place_nodes_) {
    if (!places.hasNode(node_id)) {
      continue;
    }

    active_places.insert(node_id);
  }

  return active_places;
}

void DsgBackend::storeUnlabeledPlaces(const ActiveNodeSet active_nodes) {
  const auto& places = private_dsg_->graph->getLayer(KimeraDsgLayers::PLACES);

  unlabeled_place_nodes_.clear();
  for (const auto& node_id : active_nodes) {
    if (!places.hasNode(node_id)) {
      continue;
    }

    if (places.getNode(node_id)->get().hasParent()) {
      continue;
    }

    unlabeled_place_nodes_.insert(node_id);
  }
}

void DsgBackend::updateRoomsNodes() {
  if (room_finder_) {
    ScopedTimer timer("backend/room_detection", last_timestamp_, true, 1, false);
    ActiveNodeSet active_place_nodes =
        getNodesForRoomDetection(*private_dsg_->latest_places);
    VLOG(3) << "Detecting rooms for " << active_place_nodes.size() << " nodes";
    room_finder_->findRooms(*private_dsg_, active_place_nodes);
    storeUnlabeledPlaces(active_place_nodes);
  }
}

void DsgBackend::updateBuildingNode() {
  const NodeSymbol building_node_id('B', 0);
  std::unique_lock<std::mutex> lock(private_dsg_->mutex);
  const auto& rooms = private_dsg_->graph->getLayer(KimeraDsgLayers::ROOMS);

  if (!rooms.numNodes()) {
    if (private_dsg_->graph->hasNode(building_node_id)) {
      private_dsg_->graph->removeNode(building_node_id);
    }

    return;
  }

  Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
  for (const auto& id_node_pair : rooms.nodes()) {
    centroid += id_node_pair.second->attributes().position;
  }
  centroid /= rooms.numNodes();

  if (!private_dsg_->graph->hasNode(building_node_id)) {
    SemanticNodeAttributes::Ptr attrs(new SemanticNodeAttributes());
    attrs->position = centroid;
    attrs->color = config_.building_color;
    attrs->semantic_label = config_.building_semantic_label;
    attrs->name = building_node_id.getLabel();
    private_dsg_->graph->emplaceNode(
        KimeraDsgLayers::BUILDINGS, building_node_id, std::move(attrs));
  } else {
    private_dsg_->graph->getNode(building_node_id)->get().attributes().position =
        centroid;
  }

  for (const auto& id_node_pair : rooms.nodes()) {
    private_dsg_->graph->insertEdge(building_node_id, id_node_pair.first);
  }
}

void DsgBackend::logStatus(bool init) const {
  std::ofstream file;
  std::string filename = config_.pgmo.log_path + std::string("/dsg_pgmo_status.csv");
  if (init) {
    ROS_INFO("DSG Backend logging PGMO status output to %s", filename.c_str());
    file.open(filename);
    // file format
    file << "total_lc,new_lc,total_factors,total_values,new_factors,new_graph_"
            "factors,trajectory_len,run_time,optimize_time,mesh_update_time\n";
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
       << timer.getLastElapsed("backend/mesh_update").value_or(nan) << std::endl;
  file.close();
  return;
}

void DsgBackend::visualizePoseGraph() const {
  // Publish pose graph
  std::map<size_t, std::vector<ros::Time>> id_timestamps;
  id_timestamps[robot_id_] = timestamps_;
  const PoseGraph::ConstPtr& pose_graph_ptr =
      deformation_graph_->getPoseGraph(id_timestamps);
  pose_graph_pub_.publish(*pose_graph_ptr);
}

void DsgBackend::visualizeDeformationGraphEdges() const {
  visualizeDeformationGraphMeshEdges(&viz_mesh_mesh_edges_pub_,
                                     &viz_pose_mesh_edges_pub_);
}

void DsgBackend::loadState(const std::string& state_path,
                           const std::string& dgrf_path) {
  pcl::PolygonMeshPtr mesh(new pcl::PolygonMesh());
  kimera_pgmo::ReadMeshWithStampsFromPly(
      state_path + "/mesh.ply", mesh, &mesh_vertex_stamps_);

  latest_mesh_.reset(new kimera_pgmo::KimeraPgmoMesh(
      kimera_pgmo::PolygonMeshToPgmoMeshMsg(0, *mesh, mesh_vertex_stamps_, "world")));
  have_new_mesh_ = true;

  loadDeformationGraphFromFile(dgrf_path);
  LOG(WARNING) << "Loaded " << deformation_graph_->getNumVertices()
               << " vertices for deformation graph";
}

}  // namespace incremental
}  // namespace kimera
