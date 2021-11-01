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
using kimera_pgmo::TriangleMeshIdStamped;
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
  dsg_nh.getParam("enable_node_merging", enable_node_merging_);

  KimeraRPGO::RobustSolverParams params = deformation_graph_->getParams();

  std::string rpgo_verbosity = "UPDATE";
  dsg_nh.getParam("rpgo_verbosity", rpgo_verbosity);
  params.verbosity = parseVerbosityFromString(rpgo_verbosity);

  std::string rpgo_solver = "LM";
  dsg_nh.getParam("rpgo_solver", rpgo_solver);
  params.solver = parseSolverFromString(rpgo_solver);

  nh_.getParam("pgmo/log_output", log_);
  if (log_) {
    if (nh_.getParam("pgmo/log_path", log_path_)) {
      params.logOutput(log_path_);
      logStatus(true);
    } else {
      ROS_ERROR("Failed to get backend log path");
    }
  }

  deformation_graph_->setParams(params);
  setVerboseFlag(false);

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
  VLOG(2) << " [DSG Backend] joined visualized thread";

  VLOG(2) << " [DSG Backend] joining pgmo thread";
  if (pgmo_thread_) {
    pgmo_thread_->join();
  }
  VLOG(2) << " [DSG Backend] joined pgmo thread";
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

  visualizer_->setGraph(private_dsg_->graph);

  visualizer_thread_.reset(new std::thread(&DsgBackend::runVisualizer, this));
}

void DsgBackend::runVisualizer() {
  ros::WallRate r(5);
  while (ros::ok() && !should_shutdown_) {
    // process any config changes
    visualizer_queue_->callAvailable(ros::WallDuration(0));

    // TODO(nathan) this is janky, avoid weird updated / redraw split
    if (private_dsg_->updated) {
      visualizer_->setGraphUpdated();
      private_dsg_->updated = false;
    }

    {  // start graph update critical section
      std::unique_lock<std::mutex> graph_lock(private_dsg_->mutex);
      visualizer_->redraw();
    }  // end graph update critical section

    r.sleep();
  }
}

void DsgBackend::fullMeshCallback(const TriangleMeshIdStamped::ConstPtr& msg) {
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
}

void DsgBackend::poseGraphCallback(const PoseGraph::ConstPtr& msg) {
  std::unique_lock<std::mutex> lock(pgmo_mutex_);
  if (!pose_graph_updates_) {
    pose_graph_updates_.reset(new PoseGraph(*msg));
  } else {
    mergePoseGraphs(*msg, *pose_graph_updates_);
  }
}

void DsgBackend::startPgmo() {
  {  // start graph update critical section
    std::unique_lock<std::mutex> graph_lock(private_dsg_->mutex);
    if (!private_dsg_->graph->hasDynamicLayer(KimeraDsgLayers::AGENTS, robot_prefix_)) {
      private_dsg_->graph->createDynamicLayer(KimeraDsgLayers::AGENTS, robot_prefix_);
    }
  }  // end graph update critical section

  full_mesh_sub_ =
      nh_.subscribe("pgmo/full_mesh", 1, &DsgBackend::fullMeshCallback, this);
  deformation_graph_sub_ = nh_.subscribe(
      "pgmo/mesh_graph_incremental", 20, &DsgBackend::deformationGraphCallback, this);
  pose_graph_sub_ =
      nh_.subscribe("pose_graph_incremental", 20, &DsgBackend::poseGraphCallback, this);

  pgmo_thread_.reset(new std::thread(&DsgBackend::runPgmo, this));
}

void DsgBackend::runPgmo() {
  ros::Rate r(10);
  while (ros::ok() && !should_shutdown_) {
    status_.reset();
    {  // spin timer scope
      ScopedTimer spin_timer("pgmo/spin");
      const size_t prev_loop_closures = num_loop_closures_;

      {  // start pgmo critical section
        std::unique_lock<std::mutex> lock(pgmo_mutex_);

        if (deformation_graph_updates_) {
          status_.new_graph_factors_ = deformation_graph_updates_->edges.size();
          status_.new_factors_ += deformation_graph_updates_->edges.size();
          processIncrementalMeshGraph(
              deformation_graph_updates_, timestamps_, &unconnected_nodes_);
          deformation_graph_updates_.reset();
          have_new_deformation_graph_ = true;
        }

        if (pose_graph_updates_) {
          status_.new_factors_ += pose_graph_updates_->edges.size();
          processIncrementalPoseGraph(
              pose_graph_updates_, &trajectory_, &unconnected_nodes_, &timestamps_);
          pose_graph_updates_.reset();
          have_new_poses_ = true;
        }
      }  // end pgmo critical section
      if (num_loop_closures_ > 0) {
        status_.total_loop_closures_ = num_loop_closures_;
        status_.new_loop_closures_ = num_loop_closures_ - prev_loop_closures;
      }
      status_.trajectory_len_ = trajectory_.size();

      // TODO(Yun) Fix to update with only new changes while ignoring old
      {
        std::unique_lock<std::mutex> graph_lock(private_dsg_->mutex);
        std::unique_lock<std::mutex> shared_graph_lock(shared_dsg_->mutex);
        private_dsg_->graph->mergeGraph(*shared_dsg_->graph);
      }

      updatePlaceMeshMapping();

      if (optimize_on_lc_ && prev_loop_closures != num_loop_closures_) {
        ScopedTimer optimize_timer("pgmo/optimize");
        optimize();
        // we already filled the mesh from the latest message via optimze
        have_new_mesh_ = false;
      } else {
        callUpdateFunctions();
      }

      if (have_new_mesh_) {
        ScopedTimer mesh_update_time("pgmo/mesh_update");
        updateDsgMesh();
        have_new_mesh_ = false;
      }

      if (have_new_poses_) {
        addNewAgentPoses();
        have_new_poses_ = false;
      }

      status_.total_factors_ = deformation_graph_->getGtsamFactors().size();
      status_.total_values_ = deformation_graph_->getGtsamValues().size();
    }  // end spin timer scope

    if (log_) {
      logStatus();
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
      private_dsg_->graph
          ->getDynamicLayer(KimeraDsgLayers::AGENTS, robot_prefix_)
          ->get();

  {  // start graph update critical section
    std::unique_lock<std::mutex> graph_lock(private_dsg_->mutex);

    for (size_t i = agent_layer.numNodes(); i < trajectory.size(); ++i) {
      Eigen::Vector3d position = trajectory[i].translation();
      Eigen::Quaterniond rotation(trajectory[i].rotation().matrix());

      // TODO(nathan) implicit assumption that pgmo ids are sequential starting at 0
      NodeSymbol pgmo_key(robot_prefix_, i);

      private_dsg_->graph->emplaceDynamicNode(
          agent_layer.id,
          agent_layer.prefix,
          std::chrono::nanoseconds(timestamps_.at(i).toNSec()),
          std::make_unique<AgentNodeAttributes>(rotation, position, pgmo_key));
    }
  }  // end graph update critical section
}

void DsgBackend::addPlacesToDeformationGraph() {
  CHECK(shared_dsg_->graph->hasLayer(KimeraDsgLayers::PLACES));
  const SceneGraphLayer& places =
      *(shared_dsg_->graph->getLayer(KimeraDsgLayers::PLACES));

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
  pcl::PolygonMesh input_mesh;

  {  // pgmo critical section
    std::unique_lock<std::mutex> lock(pgmo_mutex_);
    if (!latest_mesh_) {
      return;
    }

    input_mesh = kimera_pgmo::TriangleMeshMsgToPolygonMesh(latest_mesh_->mesh);
  }  // pgmo critical section

  if (input_mesh.cloud.height * input_mesh.cloud.width == 0) {
    return;
  }

  const pcl::PolygonMesh opt_mesh =
      deformation_graph_->deformMesh(input_mesh, robot_vertex_prefix_);

  {  // start graph update critical section
    std::unique_lock<std::mutex> graph_lock(private_dsg_->mutex);
    private_dsg_->graph->setMeshDirectly(opt_mesh);
    private_dsg_->updated = true;
  }  // end graph update critical section
}

void DsgBackend::updatePlaceMeshMapping() {
  {  // get block mesh mapping from shared dsg
    std::unique_lock<std::mutex> lock(shared_dsg_->mutex);
    *private_dsg_->block_mesh_mapping = *shared_dsg_->block_mesh_mapping;
  }

  std::unique_lock<std::mutex> lock(private_dsg_->mutex);

  const SceneGraphLayer& places_layer =
      private_dsg_->graph->getLayer(KimeraDsgLayers::PLACES).value();

  size_t num_invalid = 0;
  for (const auto& id_node_pair : places_layer.nodes()) {
    auto& attrs = id_node_pair.second->attributes<PlaceNodeAttributes>();
    if (!attrs.is_active) {
      continue;
    }

    if (attrs.voxblox_mesh_connections.empty()) {
      continue;
    }

    // reset connections (and mark inactive to avoid processing outside active
    // window)
    attrs.is_active = false;
    attrs.pcl_mesh_connections.clear();
    attrs.pcl_mesh_connections.reserve(attrs.voxblox_mesh_connections.size());

    for (const auto& connection : attrs.voxblox_mesh_connections) {
      voxblox::BlockIndex index =
          Eigen::Map<const voxblox::BlockIndex>(connection.block);
      if (!private_dsg_->block_mesh_mapping->count(index)) {
        num_invalid++;
        continue;
      }

      const auto& vertex_mapping = private_dsg_->block_mesh_mapping->at(index);
      if (!vertex_mapping.count(connection.vertex)) {
        num_invalid++;
        continue;
      }

      attrs.pcl_mesh_connections.push_back(
          vertex_mapping.at(connection.vertex));
    }
  }

  // TODO(nathan) prune removed blocks? requires more of a handshake with gvd
  // integrator

  if (num_invalid) {
    VLOG(2) << "[DSG Backend] Place-Mesh Update: " << num_invalid
            << " invalid connections";
  }
}

void DsgBackend::optimize() {
  if (add_places_to_deformation_graph_) {
    {  // start dsg mutex critical section
      std::unique_lock<std::mutex> graph_lock(shared_dsg_->mutex);
      addPlacesToDeformationGraph();
    }  // end dsg mutex critical section
  }

  {  // timer scope
    ScopedTimer timer("backend/optimization", true, 0, false);
    deformation_graph_->optimize();
  }  // timer scope

  updateDsgMesh();

  gtsam::Values pgmo_values = deformation_graph_->getGtsamValues();
  gtsam::Values places_values = deformation_graph_->getGtsamTempValues();

  callUpdateFunctions(places_values, pgmo_values);
}

void DsgBackend::callUpdateFunctions(const gtsam::Values& places_values,
                                     const gtsam::Values& pgmo_values) {
  std::unique_lock<std::mutex> graph_lock(private_dsg_->mutex);
  for (const auto& update_func : dsg_update_funcs_) {
    // TODO(nathan) might need diferrent values
    update_func(
        *private_dsg_->graph, places_values, pgmo_values, enable_node_merging_);
  }
  private_dsg_->updated = true;
}

void DsgBackend::logStatus(bool init) const {
  std::ofstream file;
  std::string filename = log_path_ + std::string("/dsg_pgmo_status.csv");
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
       << timer.getLastElapsed("pgmo/spin").value_or(nan) << ","
       << timer.getLastElapsed("pgmo/optimize").value_or(nan) << ","
       << timer.getLastElapsed("pgmo/mesh_update").value_or(nan) << std::endl;
  file.close();
  return;
}

}  // namespace incremental
}  // namespace kimera
