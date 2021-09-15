#include "kimera_dsg_builder/incremental_dsg_backend.h"
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
using Node = SceneGraph::Node;

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

DsgBackend::DsgBackend(const ros::NodeHandle nh, const SharedDsgInfo::Ptr& dsg)
    : KimeraPgmoInterface(),
      nh_(nh),
      dsg_(dsg),
      robot_id_(0),
      add_places_to_deformation_graph_(true),
      optimize_on_lc_(true),
      have_new_mesh_(false),
      have_new_poses_(false),
      have_new_deformation_graph_(false) {
  nh_.getParam("robot_id", robot_id_);

  if (!loadParameters(ros::NodeHandle(nh_, "pgmo"))) {
    ROS_FATAL("Failed to initialize pgmo parameters!");
    throw std::runtime_error("pgmo parameter parsing failed");
  }

  ros::NodeHandle dsg_nh(nh_, "dsg");
  dsg_nh.getParam("add_places_to_deformation_graph", add_places_to_deformation_graph_);
  dsg_nh.getParam("optimize_on_lc", optimize_on_lc_);

  KimeraRPGO::RobustSolverParams params = deformation_graph_->getParams();

  std::string rpgo_verbosity = "UPDATE";
  dsg_nh.getParam("rpgo_verbosity", rpgo_verbosity);
  params.verbosity = parseVerbosityFromString(rpgo_verbosity);

  std::string rpgo_solver = "LM";
  dsg_nh.getParam("rpgo_solver", rpgo_solver);
  params.solver = parseSolverFromString(rpgo_solver);

  deformation_graph_->setParams(params);

  robot_prefix_ = kimera_pgmo::robot_id_to_prefix.at(robot_id_);
  robot_vertex_prefix_ = kimera_pgmo::robot_id_to_vertex_prefix.at(robot_id_);

  dsg_update_funcs_.push_back(&dsg_updates::updateAgents);
  dsg_update_funcs_.push_back(&dsg_updates::updateObjects);
  dsg_update_funcs_.push_back(&dsg_updates::updatePlaces);
  dsg_update_funcs_.push_back(&dsg_updates::updateRooms);
  dsg_update_funcs_.push_back(&dsg_updates::updateBuildings);

  deformation_graph_->storeOnlyNoOptimization();
}

DsgBackend::~DsgBackend() {
  should_shutdown_ = true;

  VLOG(2) << "[DSG Backend] joining visualizer thread";
  if (visualizer_thread_) {
    visualizer_thread_->join();
  }
  VLOG(2) << "[DSG Backend] joined visualized thread";

  VLOG(2) << "[DSG Backend] joining pgmo thread";
  if (pgmo_thread_) {
    pgmo_thread_->join();
  }
  VLOG(2) << "[DSG Backend] joined pgmo thread";
}

void DsgBackend::start() {
  startPgmo();
  startVisualizer();
  LOG(INFO) << "[DSG Backend] started!";
}

void DsgBackend::startVisualizer() {
  std::string visualizer_ns;
  nh_.param<std::string>("visualizer_ns", visualizer_ns, "/kimera_dsg_visualizer");

  visualizer_queue_.reset(new ros::CallbackQueue());

  ros::NodeHandle nh(visualizer_ns);
  nh.setCallbackQueue(visualizer_queue_.get());

  visualizer_.reset(new DynamicSceneGraphVisualizer(nh, getDefaultLayerIds()));
  visualizer_->setGraph(dsg_->graph);

  visualizer_thread_.reset(new std::thread(&DsgBackend::runVisualizer, this));
}

void DsgBackend::runVisualizer() {
  ros::WallRate r(10);
  while (ros::ok() && !should_shutdown_) {
    // process any config changes
    visualizer_queue_->callAvailable(ros::WallDuration(0));

    // TODO(nathan) this is janky, avoid weird updated / redraw split
    if (dsg_->updated) {
      visualizer_->setGraphUpdated();
      dsg_->updated = false;
    }

    {  // start graph update critical section
      std::unique_lock<std::mutex> graph_lock(dsg_->mutex);
      visualizer_->redraw();
    }  // end graph update critical section

    r.sleep();
  }
}

void DsgBackend::fullMeshCallback(
    const kimera_pgmo::TriangleMeshIdStamped::ConstPtr& msg) {
  last_mesh_msg_ = msg;
  have_new_mesh_ = true;
}

void DsgBackend::deformationGraphCallback(
    const pose_graph_tools::PoseGraph::ConstPtr& msg) {
  processIncrementalMeshGraph(msg, timestamps_, &unconnected_nodes_);
  have_new_deformation_graph_ = true;
}

void DsgBackend::poseGraphCallback(const pose_graph_tools::PoseGraph::ConstPtr& msg) {
  processIncrementalPoseGraph(msg, &trajectory_, &unconnected_nodes_, &timestamps_);
  have_new_poses_ = true;
}

void DsgBackend::startPgmo() {
  pgmo_queue_.reset(new ros::CallbackQueue());

  pgmo_nh_ = ros::NodeHandle(nh_);
  pgmo_nh_.setCallbackQueue(pgmo_queue_.get());

  {  // start graph update critical section
    std::unique_lock<std::mutex> graph_lock(dsg_->mutex);
    if (!dsg_->graph->hasDynamicLayer(KimeraDsgLayers::AGENTS, robot_prefix_)) {
      dsg_->graph->createDynamicLayer(KimeraDsgLayers::AGENTS, robot_prefix_);
    }
  }  // end graph update critical section

  mesh_viz_plugin_.reset(
      new MeshPlaceConnectionsPlugin(ros::NodeHandle(nh_, "pm_graph_viz"),
                                     "mesh_connection_plugin",
                                     robot_vertex_prefix_,
                                     deformation_graph_));

  full_mesh_sub_ =
      pgmo_nh_.subscribe("full_mesh", 1, &DsgBackend::fullMeshCallback, this);
  // TODO(nathan) may need to handle these in a different thread from optimization
  deformation_graph_sub_ = pgmo_nh_.subscribe(
      "mesh_graph_incremental", 20, &DsgBackend::deformationGraphCallback, this);
  pose_graph_sub_ = pgmo_nh_.subscribe(
      "pose_graph_incremental", 20, &DsgBackend::poseGraphCallback, this);

  pgmo_thread_.reset(new std::thread(&DsgBackend::runPgmo, this));
}

void DsgBackend::drawPlacesSkeleton() {
  std::unique_lock<std::mutex> graph_lock(dsg_->mutex);
  // this points to the deformation graph, so we need to run this in the same
  // thread pgmo runs in
  mesh_viz_plugin_->draw(*dsg_->graph);
}

void DsgBackend::runPgmo() {
  ros::Rate r(10);
  while (ros::ok() && !should_shutdown_) {
    const size_t prev_loop_closures = num_loop_closures_;
    pgmo_queue_->callAvailable(ros::WallDuration(0));

    if (optimize_on_lc_ && prev_loop_closures != num_loop_closures_) {
      // TODO(nathan) management of this will have to change
      optimize();
      drawPlacesSkeleton();
      // we already filled the mesh from the latest message via optimze
      have_new_mesh_ = false;
    }

    if (have_new_mesh_) {
      updateDsgMesh();
      have_new_mesh_ = false;
    }

    if (have_new_deformation_graph_) {
      drawPlacesSkeleton();
      have_new_deformation_graph_ = false;
    }

    if (have_new_poses_) {
      addNewAgentPoses();
      have_new_poses_ = false;
    }

    r.sleep();
  }
}

void DsgBackend::addNewAgentPoses() {
  if (timestamps_.empty()) {
    // we don't have any pose graph updates, don't try and fill dynamic graph
    return;
  }

  // TODO(nathan) consider grabbing optimized trajectory from deformation graph
  const auto& trajectory = trajectory_;

  const DynamicSceneGraphLayer& agent_layer =
      dsg_->graph->getDynamicLayer(KimeraDsgLayers::AGENTS, robot_prefix_)->get();

  {  // start graph update critical section
    std::unique_lock<std::mutex> graph_lock(dsg_->mutex);

    for (size_t i = agent_layer.numNodes(); i < trajectory.size(); ++i) {
      Eigen::Vector3d position = trajectory[i].translation();
      Eigen::Quaterniond rotation(trajectory[i].rotation().matrix());

      // TODO(nathan) implicit assumption that pgmo ids are sequential starting at 0
      NodeSymbol pgmo_key(robot_prefix_, i);

      dsg_->graph->emplaceDynamicNode(
          agent_layer.id,
          agent_layer.prefix,
          std::chrono::nanoseconds(timestamps_.at(i).toNSec()),
          std::make_unique<AgentNodeAttributes>(rotation, position, pgmo_key));
    }
  }  // end graph update critical section
}

void DsgBackend::addPlacesToDeformationGraph() {
  CHECK(dsg_->graph->hasLayer(KimeraDsgLayers::PLACES));
  const SceneGraphLayer& places = *(dsg_->graph->getLayer(KimeraDsgLayers::PLACES));

  if (places.nodes().empty()) {
    LOG(WARNING) << "Attempting to add places to deformation graph with empty "
                    "places layer";
    return;
  }

  deformation_graph_->clearTemporaryStructures();

  MinimumSpanningTreeInfo mst_info = getMinimumSpanningEdges(places);

  for (const auto& id_node_pair : places.nodes()) {
    const auto& node = *id_node_pair.second;
    const auto& attrs = node.attributes<PlaceNodeAttributes>();

    gtsam::Pose3 curr_pose(gtsam::Rot3(), attrs.position);
    deformation_graph_->addNewTempNode(node.id, curr_pose, false);

    if (!mst_info.leaves.count(node.id)) {
      continue;
    }

    if (attrs.pcl_mesh_connections.empty()) {
      continue;
    }

    deformation_graph_->addTempNodeValence(
        node.id, attrs.pcl_mesh_connections, robot_vertex_prefix_);
  }

  for (const auto& edge : mst_info.edges) {
    gtsam::Pose3 source(gtsam::Rot3(), places.getPosition(edge.source));
    gtsam::Pose3 target(gtsam::Rot3(), places.getPosition(edge.target));
    deformation_graph_->addNewTempBetween(
        edge.source, edge.target, source.between(target));
  }
}

void DsgBackend::updateDsgMesh() {
  if (!last_mesh_msg_) {
    return;
  }

  const pcl::PolygonMesh input_mesh =
      kimera_pgmo::TriangleMeshMsgToPolygonMesh(last_mesh_msg_->mesh);
  if (input_mesh.cloud.height * input_mesh.cloud.width == 0) {
    return;
  }

  const pcl::PolygonMesh opt_mesh =
      deformation_graph_->deformMesh(input_mesh, robot_vertex_prefix_);

  {  // start graph update critical section
    std::unique_lock<std::mutex> graph_lock(dsg_->mutex);
    dsg_->graph->setMeshDirectly(opt_mesh);
    dsg_->updated = true;
  }  // end graph update critical section
}

void DsgBackend::optimize() {
  if (add_places_to_deformation_graph_) {
    {  // start dsg mutex critical section
      std::unique_lock<std::mutex> graph_lock(dsg_->mutex);
      addPlacesToDeformationGraph();
    }  // end dsg mutex critical section
  }

  {  // timer scope
    ScopedTimer timer("backend/optimization", true, 1, false);
    deformation_graph_->optimize();
  }  // timer scope

  updateDsgMesh();

  gtsam::Values pgmo_values = deformation_graph_->getGtsamValues();
  gtsam::Values places_values = deformation_graph_->getGtsamTempValues();

  for (const auto& update_func : dsg_update_funcs_) {
    {  // start dsg mutex critical section
      std::unique_lock<std::mutex> graph_lock(dsg_->mutex);
      // TODO(nathan) might need diferrent values
      update_func(*dsg_->graph, places_values, pgmo_values);
      dsg_->updated = true;
    }  // end dsg mutex criticial section
  }
}

}  // namespace incremental
}  // namespace kimera
