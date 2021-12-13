#include "kimera_dsg_builder/incremental_dsg_backend.h"
#include "kimera_dsg_builder/common.h"
#include "kimera_dsg_builder/minimum_spanning_tree.h"
#include "kimera_dsg_builder/pcl_conversion.h"
#include "kimera_dsg_builder/timing_utilities.h"
#include "kimera_dsg_builder/visualizer_plugins.h"

#include <kimera_dsg/node_attributes.h>
#include <pcl/search/kdtree.h>
#include <voxblox/core/block_hash.h>

#include <glog/logging.h>

namespace kimera {
namespace incremental {

using kimera_pgmo::DeformationGraph;
using kimera_pgmo::DeformationGraphPtr;
using kimera_pgmo::KimeraPgmoInterface;
using kimera_pgmo::KimeraPgmoMesh;
using kimera_pgmo::Path;
using pose_graph_tools::PoseGraph;
using Node = SceneGraph::Node;

inline void mergePoseGraphs(const PoseGraph& new_graph, PoseGraph& graph_to_fill) {
  // TODO(nathan) this needs checks for the multi-robot case
  graph_to_fill.header = new_graph.header;
  graph_to_fill.nodes.insert(
      graph_to_fill.nodes.end(), new_graph.nodes.begin(), new_graph.nodes.end());
  graph_to_fill.edges.insert(
      graph_to_fill.edges.end(), new_graph.edges.begin(), new_graph.edges.end());
}

void parseRoomClusterMode(const ros::NodeHandle& nh,
                          const std::string& name,
                          RoomFinder::Config::ClusterMode& param) {
  std::string clustering_mode = "MODULARITY";
  nh.getParam(name, clustering_mode);

  std::string to_check = clustering_mode;
  std::transform(to_check.begin(), to_check.end(), to_check.begin(), [](const auto& c) {
    return std::toupper(c);
  });

  if (to_check == "SPECTRAL") {
    param = RoomFinder::Config::ClusterMode::SPECTRAL;
  } else if (to_check == "MODULARITY") {
    param = RoomFinder::Config::ClusterMode::MODULARITY;
  } else if (to_check == "NONE") {
    param = RoomFinder::Config::ClusterMode::NONE;
  } else {
    ROS_ERROR_STREAM("Unrecognized room clustering mode: " << to_check
                                                           << ". Defaulting to NONE");
    param = RoomFinder::Config::ClusterMode::NONE;
  }
}

KimeraRPGO::Verbosity parseVerbosityFromString(const std::string& verb_str) {
  std::string to_check = verb_str;
  std::transform(to_check.begin(), to_check.end(), to_check.begin(), [](const auto& c) {
    return std::toupper(c);
  });

  if (to_check == "UPDATE") {
    return KimeraRPGO::Verbosity::UPDATE;
  } else if (to_check == "QUIET") {
    return KimeraRPGO::Verbosity::QUIET;
  } else if (to_check == "VERBOSE") {
    return KimeraRPGO::Verbosity::VERBOSE;
  } else {
    ROS_ERROR_STREAM("unrecognized verbosity option: " << to_check
                                                       << ". defaulting to UPDATE");
    return KimeraRPGO::Verbosity::UPDATE;
  }
}

KimeraRPGO::Solver parseSolverFromString(const std::string& solver_str) {
  std::string to_check = solver_str;
  std::transform(to_check.begin(), to_check.end(), to_check.begin(), [](const auto& c) {
    return std::toupper(c);
  });

  if (to_check == "LM") {
    return KimeraRPGO::Solver::LM;
  } else if (to_check == "GN") {
    return KimeraRPGO::Solver::GN;
  } else {
    ROS_ERROR_STREAM("unrecognized solver option: " << to_check
                                                    << ". defaulting to LM");
    return KimeraRPGO::Solver::LM;
  }
}

DsgBackend::DsgBackend(const ros::NodeHandle nh,
                       const SharedDsgInfo::Ptr& dsg,
                       const SharedDsgInfo::Ptr& backend_dsg)
    : KimeraPgmoInterface(),
      nh_(nh),
      shared_dsg_(dsg),
      private_dsg_(backend_dsg),
      robot_id_(0),
      add_places_to_deformation_graph_(true),
      optimize_on_lc_(true),
      enable_node_merging_(true),
      call_update_periodically_(true),
      places_merge_pos_threshold_m_(0.4),
      places_merge_distance_tolerance_m_(0.3),
      shared_places_copy_(KimeraDsgLayers::PLACES),
      have_loopclosures_(false),
      have_new_mesh_(false),
      visualizer_should_reset_(false) {
  nh_.getParam("robot_id", robot_id_);

  if (!loadParameters(ros::NodeHandle(nh_, "pgmo"))) {
    ROS_FATAL("Failed to initialize pgmo parameters!");
    throw std::runtime_error("pgmo parameter parsing failed");
  }

  ros::NodeHandle dsg_nh(nh_, "dsg");
  dsg_nh.getParam("add_places_to_deformation_graph", add_places_to_deformation_graph_);
  dsg_nh.getParam("optimize_on_lc", optimize_on_lc_);
  dsg_nh.getParam("enable_node_merging", enable_node_merging_);
  dsg_nh.getParam("call_update_periodically", call_update_periodically_);
  dsg_nh.getParam("places_merge_pos_threshold_m", places_merge_pos_threshold_m_);
  dsg_nh.getParam("places_merge_distance_tolerance_m",
                  places_merge_distance_tolerance_m_);

  KimeraRPGO::RobustSolverParams params = deformation_graph_->getParams();

  std::string rpgo_verbosity = "UPDATE";
  dsg_nh.getParam("rpgo_verbosity", rpgo_verbosity);
  params.verbosity = parseVerbosityFromString(rpgo_verbosity);

  std::string rpgo_solver = "LM";
  dsg_nh.getParam("rpgo_solver", rpgo_solver);
  params.solver = parseSolverFromString(rpgo_solver);

  nh_.getParam("pgmo/log_output", pgmo_log_);
  if (pgmo_log_) {
    if (nh_.getParam("pgmo/log_path", pgmo_log_path_)) {
      params.logOutput(pgmo_log_path_);
      logStatus(true);
    } else {
      ROS_ERROR("Failed to get pgmo log path");
    }
  }

  deformation_graph_->setParams(params);
  setVerboseFlag(false);

  robot_prefix_ = kimera_pgmo::robot_id_to_prefix.at(robot_id_);
  robot_vertex_prefix_ = kimera_pgmo::robot_id_to_vertex_prefix.at(robot_id_);

  bool enable_rooms = true;
  nh_.getParam("enable_rooms", enable_rooms);
  if (enable_rooms) {
    // TODO(nathan) clean up
    RoomFinder::Config config;
    nh_.getParam("room_finder/min_dilation_m", config.min_dilation_m);
    nh_.getParam("room_finder/max_dilation_m", config.max_dilation_m);
    parseParam(nh_, "room_finder/num_steps", config.num_steps);
    parseParam(nh_, "room_finder/min_component_size", config.min_component_size);
    parseParam(nh_, "room_finder/max_kmeans_iters", config.max_kmeans_iters);
    parseParam(nh_, "room_finder/min_room_size", config.min_room_size);
    nh_.getParam("room_finder/room_vote_min_overlap", config.room_vote_min_overlap);
    nh_.getParam("room_finder/use_sparse_eigen_decomp", config.use_sparse_eigen_decomp);
    nh_.getParam("room_finder/sparse_decomp_tolerance", config.sparse_decomp_tolerance);
    nh_.getParam("room_finder/max_modularity_iters", config.max_modularity_iters);
    nh_.getParam("room_finder/modularity_gamma", config.modularity_gamma);
    parseRoomClusterMode(nh_, "room_finder/clustering_mode", config.clustering_mode);
    room_finder_.reset(new RoomFinder(config));
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
                              places_merge_pos_threshold_m_,
                              places_merge_distance_tolerance_m_);
  });
  // dsg_update_funcs_.push_back(&dsg_updates::updateRooms); // redundant to updateRoomsNodes
  // dsg_update_funcs_.push_back(&dsg_updates::updateBuildings); // redundant to updateBuildingNode

  deformation_graph_->storeOnlyNoOptimization();

  // purple
  std::vector<double> building_color{0.662, 0.0313, 0.7607};
  nh_.getParam("building_color", building_color);
  if (building_color.size() != 3) {
    ROS_ERROR_STREAM("supplied building color size " << building_color.size()
                                                     << " != 3");
    building_color = std::vector<double>{0.662, 0.0313, 0.7607};
  }

  building_color_ << std::clamp(static_cast<int>(255 * building_color.at(0)), 0, 255),
      std::clamp(static_cast<int>(255 * building_color.at(1)), 0, 255),
      std::clamp(static_cast<int>(255 * building_color.at(2)), 0, 255);

  nh_.getParam("dsg_log_output", dsg_log_);
  if (dsg_log_ && nh_.getParam("dsg_output_path", dsg_log_path_)) {
    backend_graph_logger_.setOutputPath(dsg_log_path_ + "/backend");
    ROS_INFO("Logging backend graph to %s", (dsg_log_path_ + "/backend").c_str());
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
  should_shutdown_ = true;

  VLOG(2) << "[DSG Backend] joining visualizer thread";
  if (visualizer_thread_) {
    visualizer_thread_->join();
    visualizer_thread_.reset();
  }
  VLOG(2) << " [DSG Backend] joined visualized thread";

  visualizer_.reset();

  VLOG(2) << " [DSG Backend] joining optimizer thread";
  if (optimizer_thread_) {
    optimizer_thread_->join();
    optimizer_thread_.reset();
  }
  VLOG(2) << " [DSG Backend] joined optimizer thread";
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
  visualizer_->addPlugin(std::make_shared<PgmoMeshPlugin>(nh, "dsg_mesh"));
  // TODO(nathan) voxblox mesh plugin in rviz doesn't handle large graphs well
  // (rviz frame-rate drops significantly after ~15 seconds)
  // visualizer_->addPlugin(std::make_shared<VoxbloxMeshPlugin>(nh, "dsg_mesh"));

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
  while (ros::ok() && !should_shutdown_) {
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
  // TODO(Yun) Fix to update with only new changes while ignoring old
  bool have_frontend_updates = shared_dsg_->updated;
  if (have_frontend_updates) {
    {  // start joint critical section
      std::unique_lock<std::mutex> shared_graph_lock(shared_dsg_->mutex);
      private_dsg_->graph->mergeGraph(*shared_dsg_->graph);
      *private_dsg_->latest_places = *shared_dsg_->latest_places;

      if (shared_dsg_->graph->hasLayer(KimeraDsgLayers::PLACES)) {
        const SceneGraphLayer& shared_places =
            *(shared_dsg_->graph->getLayer(KimeraDsgLayers::PLACES));
        shared_places_copy_.mergeLayer(shared_places);
        std::vector<NodeId> removed_place_nodes;
        shared_places.getRemovedNodes(&removed_place_nodes);
        for (const auto& place_id : removed_place_nodes) {
          shared_places_copy_.removeNode(place_id);
        }
      }

      if (dsg_log_) {
        backend_graph_logger_.logGraph(private_dsg_->graph);
      }
    }  // end joint critical section
    shared_dsg_->updated = false;
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

void DsgBackend::runPgmo() {
  ros::WallRate r(5);
  while (ros::ok()) {
    status_.reset();
    ScopedTimer spin_timer("backend/spin", last_timestamp_);
    const size_t prev_loop_closures = num_loop_closures_;
    bool have_graph_updates = false;

    {  // start pgmo critical section
      std::unique_lock<std::mutex> lock(pgmo_mutex_);
      ScopedTimer spin_timer("backend/process_factors", last_timestamp_);
      if (deformation_graph_updates_) {
        status_.new_graph_factors_ = deformation_graph_updates_->edges.size();
        status_.new_factors_ += deformation_graph_updates_->edges.size();
        processIncrementalMeshGraph(
            deformation_graph_updates_, timestamps_, &unconnected_nodes_);
        deformation_graph_updates_.reset();
        have_graph_updates = true;
      }

      if (pose_graph_updates_) {
        status_.new_factors_ += pose_graph_updates_->edges.size();
        processIncrementalPoseGraph(
            pose_graph_updates_, &trajectory_, &unconnected_nodes_, &timestamps_);
        logIncrementalLoopClosures(*pose_graph_updates_);
        pose_graph_updates_.reset();
        have_graph_updates = true;
      }

      have_graph_updates |= addInternalLCDToDeformationGraph();
    }  // end pgmo critical section

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
    {
      // start private dsg critical section
      std::unique_lock<std::mutex> graph_lock(private_dsg_->mutex);
      have_dsg_updates = updatePrivateDsg();
      if (optimize_on_lc_ && have_graph_updates && have_loopclosures_) {
        optimize();
      } else if (call_update_periodically_ && have_dsg_updates) {
        std::unique_lock<std::mutex> pgmo_lock(pgmo_mutex_);
        updateDsgMesh();
        callUpdateFunctions();
      }
      private_dsg_->updated = true;
    }  // end private dsg critical section

    if (have_graph_updates && pgmo_log_) {
      logStatus();
    }

    if (have_dsg_updates) {
      updateRoomsNodes();
      updateBuildingNode();
    }

    if (should_shutdown_ && !have_graph_updates && !have_dsg_updates) {
      break;
    }

    r.sleep();
  }
}

void DsgBackend::fullMeshCallback(const KimeraPgmoMesh::ConstPtr& msg) {
  std::unique_lock<std::mutex> lock(pgmo_mutex_);
  latest_mesh_ = msg;
  have_new_mesh_ = true;
}

void DsgBackend::deformationGraphCallback(const PoseGraph::ConstPtr& msg) {
  std::unique_lock<std::mutex> lock(pgmo_mutex_);
  if (!deformation_graph_updates_) {
    deformation_graph_updates_.reset(new PoseGraph(*msg));
  } else {
    mergePoseGraphs(*msg, *deformation_graph_updates_);
  }
  last_timestamp_ = msg->header.stamp.toNSec();
}

void DsgBackend::poseGraphCallback(const PoseGraph::ConstPtr& msg) {
  std::unique_lock<std::mutex> lock(pgmo_mutex_);
  if (!pose_graph_updates_) {
    pose_graph_updates_.reset(new PoseGraph(*msg));
  } else {
    mergePoseGraphs(*msg, *pose_graph_updates_);
  }
}

bool DsgBackend::saveMeshCallback(std_srvs::Empty::Request&,
                                  std_srvs::Empty::Response&) {
  pcl::PolygonMesh opt_mesh;
  {
    std::unique_lock<std::mutex> graph_lock(private_dsg_->mutex);
    opt_mesh = private_dsg_->graph->getMesh();
  }
  // Save mesh
  std::string ply_name = pgmo_log_path_ + std::string("/mesh_pgmo.ply");
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
  std::string csv_name = pgmo_log_path_ + std::string("/traj_pgmo.csv");
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
                                                false);
  }

  {
    ScopedTimer spin_timer("backend/add_places_between", last_timestamp_);
    PoseGraph mst_edges;
    for (const auto& edge : mst_info.edges) {
      gtsam::Pose3 source(gtsam::Rot3(),
                          shared_places_copy_.getPosition(edge.source));
      gtsam::Pose3 target(gtsam::Rot3(),
                          shared_places_copy_.getPosition(edge.target));
      pose_graph_tools::PoseGraphEdge mst_e;
      mst_e.key_from = edge.source;
      mst_e.key_to = edge.target;
      mst_e.pose = kimera_pgmo::GtsamToRos(source.between(target));
      mst_edges.edges.push_back(mst_e);
    }
    deformation_graph_->addNewTempEdges(mst_edges);
  }
}

bool DsgBackend::addInternalLCDToDeformationGraph() {
  bool added_new_loop_closure = false;
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
    deformation_graph_->addNewBetween(to_key, from_key, result.to_T_from);
    loop_closures_.push_back(
        {result.to_node, result.from_node, result.to_T_from, true, result.level});

    added_new_loop_closure = true;
    num_loop_closures_++;
    have_loopclosures_ = true;
  }

  return added_new_loop_closure;
}

void DsgBackend::updateDsgMesh() {
  // avoid scope problems by using a smart pointer
  std::unique_ptr<ScopedTimer> timer;

  pcl::PolygonMesh opt_mesh;
  if (!latest_mesh_) {
    return;
  }

  if (!have_new_mesh_) {
    return;
  }

  timer.reset(new ScopedTimer("backend/mesh_update", last_timestamp_));
  std::vector<ros::Time> mesh_vertex_stamps;
  auto input_mesh =
      kimera_pgmo::PgmoMeshMsgToPolygonMesh(*latest_mesh_, &mesh_vertex_stamps);
  have_new_mesh_ = false;

  if (input_mesh.cloud.height * input_mesh.cloud.width == 0) {
    return;
  }

  opt_mesh = deformation_graph_->deformMesh(input_mesh,
                                            mesh_vertex_stamps,
                                            robot_vertex_prefix_,
                                            num_interp_pts_,
                                            interp_horizon_);

  private_dsg_->graph->setMeshDirectly(opt_mesh);
  private_dsg_->updated = true;

  if (viz_mesh_mesh_edges_pub_.getNumSubscribers() > 0 ||
      viz_pose_mesh_edges_pub_.getNumSubscribers() > 0) {
    visualizeDeformationGraphMeshEdges(&viz_mesh_mesh_edges_pub_,
                                       &viz_pose_mesh_edges_pub_);
  }
}

void DsgBackend::optimize() {
  std::unique_lock<std::mutex> pgmo_lock(pgmo_mutex_);
  if (add_places_to_deformation_graph_) {
    addPlacesToDeformationGraph();
  }

  {  // timer scope
    ScopedTimer timer("backend/optimization", last_timestamp_, true, 0, false);
    deformation_graph_->optimize();
  }  // timer scope

  updateDsgMesh();

  gtsam::Values pgmo_values = deformation_graph_->getGtsamValues();
  gtsam::Values places_values = deformation_graph_->getGtsamTempValues();

  callUpdateFunctions(places_values, pgmo_values);
}

void DsgBackend::callUpdateFunctions(const gtsam::Values& places_values,
                                     const gtsam::Values& pgmo_values) {
  ScopedTimer spin_timer("backend/update_layers", last_timestamp_);
  for (const auto& update_func : dsg_update_funcs_) {
    // TODO(nathan) might need diferrent values
    update_func(*private_dsg_->graph, places_values, pgmo_values, enable_node_merging_);
  }
}

ActiveNodeSet DsgBackend::getNodesForRoomDetection(const NodeIdSet& latest_places) {
  std::unique_lock<std::mutex> graph_lock(private_dsg_->mutex);
  std::unordered_set<NodeId> active_places(latest_places.begin(), latest_places.end());
  // TODO(nathan) grab this from a set of active rooms
  const SceneGraphLayer& rooms =
      private_dsg_->graph->getLayer(KimeraDsgLayers::ROOMS).value();
  for (const auto& id_node_pair : rooms.nodes()) {
    active_places.insert(id_node_pair.second->children().begin(),
                         id_node_pair.second->children().end());
  }

  // TODO(nathan) this is threadsafe as long as places and rooms are on the same thread
  const SceneGraphLayer& places =
      private_dsg_->graph->getLayer(KimeraDsgLayers::PLACES).value();
  for (const auto& node_id : unlabeled_place_nodes_) {
    if (!places.hasNode(node_id)) {
      continue;
    }

    active_places.insert(node_id);
  }

  return active_places;
}

void DsgBackend::storeUnlabeledPlaces(const ActiveNodeSet active_nodes) {
  std::unique_lock<std::mutex> lock(private_dsg_->mutex);
  const SceneGraphLayer& places =
      private_dsg_->graph->getLayer(KimeraDsgLayers::PLACES).value();

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
  const SceneGraphLayer& rooms_layer =
      private_dsg_->graph->getLayer(KimeraDsgLayers::ROOMS).value();

  if (!rooms_layer.numNodes()) {
    if (private_dsg_->graph->hasNode(building_node_id)) {
      private_dsg_->graph->removeNode(building_node_id);
    }

    return;
  }

  Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
  for (const auto& id_node_pair : rooms_layer.nodes()) {
    centroid += id_node_pair.second->attributes().position;
  }
  centroid /= rooms_layer.numNodes();

  if (!private_dsg_->graph->hasNode(building_node_id)) {
    SemanticNodeAttributes::Ptr attrs(new SemanticNodeAttributes());
    attrs->position = centroid;
    attrs->color = building_color_;
    attrs->semantic_label = kBuildingSemanticLabel;
    attrs->name = building_node_id.getLabel();
    private_dsg_->graph->emplaceNode(
        KimeraDsgLayers::BUILDINGS, building_node_id, std::move(attrs));
  } else {
    private_dsg_->graph->getNode(building_node_id)->get().attributes().position =
        centroid;
  }

  for (const auto& id_node_pair : rooms_layer.nodes()) {
    private_dsg_->graph->insertEdge(building_node_id, id_node_pair.first);
  }
}

void DsgBackend::logStatus(bool init) const {
  std::ofstream file;
  std::string filename = pgmo_log_path_ + std::string("/dsg_pgmo_status.csv");
  if (init) {
    ROS_INFO("DSG Backend logging PGMO status output to %s", filename.c_str());
    file.open(filename);
    // file format
    file << "total_lc,new_lc,total_factors,total_values,new_factors,new_graph_"
            "factors,trajectory_len,run_time,optimize_time,mesh_update_time\n";
    file.close();
    return;
  }

  const ElapsedTimeRecorder& timer = ElapsedTimeRecorder::instance();
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

}  // namespace incremental
}  // namespace kimera
