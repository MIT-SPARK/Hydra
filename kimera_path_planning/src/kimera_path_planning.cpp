// NOTE: Most code is derived from mav_voxblox_planning:
// github.com/ethz-asl/mav_voxblox_planning
// Copyright (c) 2016, ETHZ ASL
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// * Neither the name of voxblox nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include "kimera_path_planning/kimera_path_planning.h"

#include <geometry_msgs/PoseArray.h>

#include <mav_planning_common/path_visualization.h>
#include <mav_planning_common/utils.h>

#include <kimera_scene_graph/utils/kimera_to_voxblox.h>

namespace kimera {

SceneGraphGlobalPlanner::SceneGraphGlobalPlanner(
    const ros::NodeHandle& nh,
    const ros::NodeHandle& nh_private)
    : nh_(nh),
      nh_private_(nh_private),
      frame_id_("map"),
      visualize_(true),
      esdf_server_(nh_, nh_private_),
      skeleton_graph_planner_(nh_, nh_private_),
      scene_graph_(nullptr),
      scene_graph_builder_(nh_, nh_private_) {
  // TODO
  constraints_.setParametersFromRos(nh_private_);

  // Parse params
  //! This is the path to the multi-layer file containing the TSDF, then the
  //! ESDF, and then the Skeleton layer.
  std::string skeleton_tsdf_esdf_path;
  nh_private_.param(
      "esdf_path", skeleton_tsdf_esdf_path, skeleton_tsdf_esdf_path);
  nh_private_.param("scene_graph_path", scene_graph_path_, scene_graph_path_);
  nh_private_.param(
      "sparse_graph_path", sparse_graph_path_, sparse_graph_path_);
  nh_private_.param("visualize", visualize_, visualize_);
  nh_private_.param("frame_id", frame_id_, frame_id_);

  // Publishers
  path_marker_pub_ =
      nh_private_.advertise<visualization_msgs::MarkerArray>("path", 1, true);
  skeleton_pub_ = nh_private_.advertise<pcl::PointCloud<pcl::PointXYZ> >(
      "skeleton", 1, true);
  sparse_graph_pub_ = nh_private_.advertise<visualization_msgs::MarkerArray>(
      "sparse_graph", 1, true);
  waypoint_list_pub_ =
      nh_.advertise<geometry_msgs::PoseArray>("waypoint_list", 1);

  // Services
  planner_srv_ = nh_private_.advertiseService(
      "plan", &SceneGraphGlobalPlanner::plannerServiceCallback, this);
  path_pub_srv_ = nh_private_.advertiseService(
      "publish_path", &SceneGraphGlobalPlanner::publishPathCallback, this);

  // Load a file from the params.
  if (skeleton_tsdf_esdf_path.empty()) {
    LOG(ERROR) << "Couldn't load map, empty filename.";
    return;
  }

  std::shared_ptr<voxblox::EsdfMap> esdf_map = esdf_server_.getEsdfMapPtr();
  CHECK(esdf_map);

  // VIT: this is the one that gets used by all planners.
  LOG(INFO) << "Loading ESDF Map.";
  if (!esdf_server_.loadMap(skeleton_tsdf_esdf_path)) {
    LOG(ERROR) << "Couldn't load ESDF map!";
  }

  LOG(INFO)
      << "Size: "
      << esdf_server_.getEsdfMapPtr()->getEsdfLayerPtr()->voxel_size()
      << " VPS: "
      << esdf_server_.getEsdfMapPtr()->getEsdfLayerPtr()->voxels_per_side();

  // Also make a new skeleton layer and load it.
  // Make this as an unmanaged raw pointer, since we'll give it to skeleton
  // generator to own.
  voxblox::Layer<voxblox::SkeletonVoxel>* skeleton_layer =
      new voxblox::Layer<voxblox::SkeletonVoxel>(
          esdf_server_.getEsdfMapPtr()->getEsdfLayerPtr()->voxel_size(),
          esdf_server_.getEsdfMapPtr()->getEsdfLayerPtr()->voxels_per_side());

  // We must save the layer as a skeleton voxel layer (or this will take forever
  // to generate).
  LOG(INFO) << "Loading Skeleton Voxel Layer.";
  if (!voxblox::io::LoadBlocksFromFile<voxblox::SkeletonVoxel>(
          skeleton_tsdf_esdf_path,
          voxblox::Layer<
              voxblox::SkeletonVoxel>::BlockMergingStrategy::kReplace,
          true,
          skeleton_layer)) {
    LOG(ERROR) << "Coudln't load skeleton layer.";
    return;
  }

  esdf_server_.setTraversabilityRadius(constraints_.robot_radius);

  // Now set up the skeleton generator: we will not really generate anything
  // since we load the graph
  skeleton_generator_.setEsdfLayer(
      esdf_server_.getEsdfMapPtr()->getEsdfLayerPtr());
  skeleton_generator_.setSkeletonLayer(skeleton_layer);
  skeleton_generator_.setMinGvdDistance(constraints_.robot_radius);
  skeleton_generator_.setGenerateByLayerNeighbors(true);

  // TODO(Toni): set one skeleton planner per layers:
  // building, rooms, and places.
  // Set up the A* planners (one per layer!)
  // CHECK(scene_graph_);
  // const SceneGraphLayer& buildings_layer_ =
  //    scene_graph_->getLayer(LayerId::kBuildingsLayerId);
  // const SceneGraphLayer& rooms_layer =
  //    scene_graph_->getLayer(LayerId::kRoomsLayerId);
  // const SceneGraphLayer& places_layer =
  //    scene_graph_->getLayer(LayerId::kPlacesLayerId);
  //
  // vxb::SparseSkeletonGraph places_skeleton_layer;
  // utils::convertLayerToSkeleton(places_layer, &places_skeleton_layer);

  // Skeleton Graph Planner -> SparseSkeletonGraph
  // Skeleton Planner -> Skeleton Voxel Layer (does A* on the voxel grid:
  // aka slow, leave it for comparison and evaluation).
  // Set up the A* planners.
  // TODO(TONI): not sure if the A* planner should work bcs we are calling
  // this getSkeletonLayer() but the skeleton layer is not computed at this
  // point, nor it will be computed if we load the sparse skeleton graph from a
  // file...
  // I believe it is only computed when we re-generate the sparse skeleton
  // graph.
  skeleton_planner_.setSkeletonLayer(skeleton_generator_.getSkeletonLayer());
  skeleton_planner_.setEsdfLayer(
      esdf_server_.getEsdfMapPtr()->getEsdfLayerPtr());
  skeleton_planner_.setMinEsdfDistance(constraints_.robot_radius);

  // Set up skeleton graph planner.
  // TODO(Toni): we must setup one skeleton graph planner per layer!
  skeleton_graph_planner_.setEsdfLayer(
      esdf_server_.getEsdfMapPtr()->getEsdfLayerPtr());

  // Set up shortener.
  path_shortener_.setEsdfLayer(esdf_server_.getEsdfMapPtr()->getEsdfLayerPtr());
  path_shortener_.setConstraints(constraints_);

  // Loco smoother!
  loco_smoother_.setParametersFromRos(nh_private_);
  loco_smoother_.setMinCollisionCheckResolution(
      esdf_server_.getEsdfMapPtr()->getEsdfLayerPtr()->voxel_size());
  loco_smoother_.setMapDistanceCallback(std::bind(
      &SceneGraphGlobalPlanner::getMapDistance, this, std::placeholders::_1));

  if (true && visualize_) {
    esdf_server_.generateMesh();
    esdf_server_.publishSlices();
    esdf_server_.publishPointclouds();
    esdf_server_.publishMap();
  }
}

void SceneGraphGlobalPlanner::generateSceneGraph() {
  if (!scene_graph_path_.empty() &&
      scene_graph_builder_.loadSceneGraph(scene_graph_path_)) {
    LOG(INFO) << "Loaded scene graph from file: " << scene_graph_path_;
  } else {
    LOG(ERROR) << "SceneGraph generation from scratch";
    // True bcs we only want to reconstruct places, rooms, and buildings.
    scene_graph_builder_.sceneGraphReconstruction(true);
  }

  if (true && visualize_) {
    scene_graph_builder_.visualizeSceneGraph();
  }

  setupSceneGraphPlanners();

  //
  // mav_trajectory_generation::timing::Timer scene_graph_reconstruction(
  //     "scene_graph/reconstruction");
  // //
  // skeleton_graph_planner_.setSparseGraph(&skeleton_generator_.getSparseGraph());
  // scene_graph_reconstruction.Stop();

  // ROS_INFO_STREAM("Generation timings: " << std::endl
  //                                        <<
  //                                        voxblox::timing::Timing::Print());
}

void SceneGraphGlobalPlanner::setupSceneGraphPlanners() {
  CHECK(scene_graph_);

  // Set up the scene graph planner: there should be one per layer.
  // We use one SkeletonAStar planner per layer.
  const SceneGraphLayer& places_layer =
      scene_graph_->getLayer(LayerId::kPlacesLayerId);
  const SceneGraphLayer& rooms_layer =
      scene_graph_->getLayer(LayerId::kRoomsLayerId);
  const SceneGraphLayer& buildings_layer =
      scene_graph_->getLayer(LayerId::kBuildingsLayerId);

  utils::convertLayerToSkeleton(places_layer, &places_skeleton_layer_);
  utils::convertLayerToSkeleton(rooms_layer, &rooms_skeleton_layer_);
  utils::convertLayerToSkeleton(buildings_layer, &buildings_skeleton_layer_);

  places_skeleton_planner_.setGraph(&places_skeleton_layer_);
  places_skeleton_planner_.setup();

  rooms_skeleton_planner_.setGraph(&rooms_skeleton_layer_);
  rooms_skeleton_planner_.setup();

  buildings_skeleton_planner_.setGraph(&buildings_skeleton_layer_);
  buildings_skeleton_planner_.setup();
}

void SceneGraphGlobalPlanner::generateSparseGraph() {
  LOG(INFO) << "About to generate skeleton graph.";
  if (!sparse_graph_path_.empty() &&
      skeleton_generator_.loadSparseGraphFromFile(sparse_graph_path_)) {
    LOG(INFO) << "Loaded sparse graph from file: " << sparse_graph_path_;
  } else {
    LOG(WARNING) << "Re-creating skeleton sparse graph! This may take a while.";
    skeleton_generator_.updateSkeletonFromLayer();
    LOG(INFO) << "Re-populated from layer.";
    skeleton_generator_.generateSparseGraph();
    LOG(INFO) << "Generated skeleton graph.";
  }
  if (true && visualize_) {
    voxblox::Pointcloud pointcloud;
    std::vector<float> distances;
    skeleton_generator_.getSkeleton().getEdgePointcloudWithDistances(
        &pointcloud, &distances);

    // Publish the skeleton.
    pcl::PointCloud<pcl::PointXYZI> ptcloud_pcl;
    voxblox::pointcloudToPclXYZI(pointcloud, distances, &ptcloud_pcl);
    ptcloud_pcl.header.frame_id = frame_id_;
    skeleton_pub_.publish(ptcloud_pcl);

    // Now visualize the graph.
    const voxblox::SparseSkeletonGraph& graph =
        skeleton_generator_.getSparseGraph();
    visualization_msgs::MarkerArray marker_array;
    vxb::visualizeSkeletonGraph(graph, frame_id_, &marker_array);
    sparse_graph_pub_.publish(marker_array);
  }

  // Set up the skeleton graph planner.
  mav_trajectory_generation::timing::Timer kd_tree_init("plan/graph/setup");
  skeleton_graph_planner_.setSparseGraph(&skeleton_generator_.getSparseGraph());
  kd_tree_init.Stop();

  LOG(INFO) << "Generation timings: \n" << voxblox::timing::Timing::Print();
}

bool SceneGraphGlobalPlanner::plannerServiceCallback(
    mav_planning_msgs::PlannerServiceRequest& request,
    mav_planning_msgs::PlannerServiceResponse& response) {
  LOG(INFO) << "Planning path.";

  mav_msgs::EigenTrajectoryPoint start_pose, goal_pose;

  mav_msgs::eigenTrajectoryPointFromPoseMsg(request.start_pose, &start_pose);
  mav_msgs::eigenTrajectoryPointFromPoseMsg(request.goal_pose, &goal_pose);

  if (getMapDistance(start_pose.position_W) < constraints_.robot_radius) {
    LOG(ERROR) << "Start pose occupied!";
    return false;
  }
  if (getMapDistance(goal_pose.position_W) < constraints_.robot_radius) {
    LOG(ERROR) << "Goal pose occupied!";
    return false;
  }

  voxblox::Point start_point =
      start_pose.position_W.cast<voxblox::FloatingPoint>();
  voxblox::Point goal_point =
      goal_pose.position_W.cast<voxblox::FloatingPoint>();

  visualization_msgs::MarkerArray marker_array;

  static constexpr bool run_astar_esdf = false;
  static constexpr bool run_astar_diagram = true;
  static constexpr bool run_astar_graph = false;
  static constexpr bool run_astar_scene_graph = true;
  static constexpr bool shorten_graph = false;
  static constexpr bool exact_start_and_goal = true;
  static constexpr bool smooth_path = true;

  if (run_astar_esdf) {
    // First, run just the ESDF A*...
    LOG(INFO) << "Running ESDF A*.";
    voxblox::AlignedVector<voxblox::Point> esdf_coordinate_path;
    mav_trajectory_generation::timing::Timer astar_esdf_timer(
        "plan/astar_esdf");
    bool success = skeleton_planner_.getPathInEsdf(
        start_point, goal_point, &esdf_coordinate_path);
    mav_msgs::EigenTrajectoryPointVector esdf_path;
    convertCoordinatePathToPath(esdf_coordinate_path, &esdf_path);
    double path_length = mav_planning::computePathLength(esdf_path);
    int num_vertices = esdf_path.size();
    astar_esdf_timer.Stop();
    LOG(INFO) << "ESDF A* \n"
              << "- Success? " << success << '\n'
              << "- Path length: " << path_length << '\n'
              << "- Vertices: " << num_vertices;

    if (visualize_) {
      marker_array.markers.push_back(
          mav_planning::createMarkerForPath(esdf_path,
                                            frame_id_,
                                            mav_visualization::Color::Yellow(),
                                            "astar_esdf",
                                            0.1));
    }
  }

  if (run_astar_diagram) {
    LOG(INFO) << "Running Diagram A*";
    voxblox::AlignedVector<voxblox::Point> diagram_coordinate_path;
    mav_trajectory_generation::timing::Timer astar_diag_timer(
        "plan/astar_diag");
    // TODO(TONI): uncertain that this should work since the skeleton layer
    // is empty at this point... I believe it only gets generated when the
    // sparse skeleton graph gets re-generated (instead of file-loaded).
    bool success = skeleton_planner_.getPathUsingEsdfAndDiagram(
        start_point, goal_point, &diagram_coordinate_path);
    mav_msgs::EigenTrajectoryPointVector diagram_path;
    convertCoordinatePathToPath(diagram_coordinate_path, &diagram_path);
    double path_length = mav_planning::computePathLength(diagram_path);
    int num_vertices = diagram_path.size();
    astar_diag_timer.Stop();
    LOG(INFO) << "Diagram A* planning\n"
              << "- Success? " << success << '\n'
              << "- Path length: " << path_length << '\n'
              << "- Vertices: " << num_vertices;

    if (visualize_) {
      LOG(INFO) << "Visualize Diagram A* of size: " << diagram_path.size();
      if (!diagram_path.empty()) {
        visualization_msgs::Marker marker = mav_planning::createMarkerForPath(
            diagram_path,
            frame_id_,
            mav_visualization::Color::Purple(),
            "astar_diag",
            0.1);
        marker_array.markers.push_back(marker);
      } else {
        LOG(ERROR) << "Requested visualization of empty path...";
      }
    }

    if (shorten_graph) {
      LOG(INFO) << "Shorten Diag A* path";
      mav_trajectory_generation::timing::Timer shorten_timer(
          "plan/astar_diag/shorten");
      mav_msgs::EigenTrajectoryPointVector short_path;
      path_shortener_.shortenPath(diagram_path, &short_path);
      path_length = mav_planning::computePathLength(short_path);
      num_vertices = short_path.size();
      LOG(INFO) << "Diag A* Shorten\n"
                << "- Success? " << success << '\n'
                << "- Path length: " << path_length << '\n'
                << "- Vertices: " << num_vertices;
      if (visualize_) {
        marker_array.markers.push_back(
            mav_planning::createMarkerForPath(short_path,
                                              frame_id_,
                                              mav_visualization::Color::Pink(),
                                              "short_astar_plan",
                                              0.1));
      }

      last_waypoints_ = short_path;

      shorten_timer.Stop();
    }
  }

  if (run_astar_graph) {
    LOG(INFO) << "Running A* Graph";
    mav_msgs::EigenTrajectoryPointVector graph_path;
    mav_trajectory_generation::timing::Timer graph_timer("plan/graph");
    skeleton_graph_planner_.setShortenPath(false);
    bool success = skeleton_graph_planner_.getPathBetweenWaypoints(
        start_pose, goal_pose, &graph_path);
    double path_length = mav_planning::computePathLength(graph_path);
    int num_vertices = graph_path.size();
    graph_timer.Stop();
    LOG(INFO) << "A* Graph planning\n"
              << "- Success? " << success << '\n'
              << "- Path length: " << path_length << '\n'
              << "- Vertices: " << num_vertices;

    if (visualize_) {
      LOG(INFO) << "Visualize A* Graph of size: " << graph_path.size();
      marker_array.markers.push_back(
          mav_planning::createMarkerForPath(graph_path,
                                            frame_id_,
                                            mav_visualization::Color::Blue(),
                                            "graph_plan",
                                            0.1));
    }

    last_waypoints_ = graph_path;

    if (shorten_graph) {
      LOG(INFO) << "Shorten A* Graph";
      mav_trajectory_generation::timing::Timer shorten_timer(
          "plan/graph/shorten");
      mav_msgs::EigenTrajectoryPointVector short_path;
      success = path_shortener_.shortenPath(graph_path, &short_path);
      path_length = mav_planning::computePathLength(short_path);
      num_vertices = short_path.size();
      LOG(INFO) << "Graph planning Shorten\n"
                << "- Success? " << success << '\n'
                << "- Path length: " << path_length << '\n'
                << "- Vertices: " << num_vertices;
      shorten_timer.Stop();

      if (visualize_) {
        marker_array.markers.push_back(
            mav_planning::createMarkerForPath(short_path,
                                              frame_id_,
                                              mav_visualization::Color::Green(),
                                              "short_plan",
                                              0.1));
      }

      last_waypoints_ = short_path;

      if (smooth_path) {
        mav_msgs::EigenTrajectoryPointVector loco_path;
        mav_trajectory_generation::timing::Timer loco_timer("plan/graph/loco");
        loco_smoother_.setResampleVisibility(true);
        loco_smoother_.setAddWaypoints(false);
        loco_smoother_.setNumSegments(5);
        loco_smoother_.getPathBetweenWaypoints(short_path, &loco_path);

        loco_timer.Stop();
        if (visualize_) {
          marker_array.markers.push_back(mav_planning::createMarkerForPath(
              loco_path,
              frame_id_,
              mav_visualization::Color::Teal(),
              "loco_plan",
              0.1));
        }

        last_waypoints_ = loco_path;
      }
    }
  }

  if (run_astar_scene_graph) {
    LOG(ERROR) << "Running A* on scene-graph.";
    voxblox::AlignedVector<voxblox::Point> places_coordinate_path;
    voxblox::AlignedVector<voxblox::Point> rooms_coordinate_path;
    voxblox::AlignedVector<voxblox::Point> buildings_coordinate_path;

    LOG(INFO) << "A* on Places Layer.";
    mav_trajectory_generation::timing::Timer scene_graph_places_timer(
        "scene_graph/plan/places");
    bool success = places_skeleton_planner_.getPath(
        start_point, goal_point, &places_coordinate_path);
    mav_msgs::EigenTrajectoryPointVector places_path;
    convertCoordinatePathToPath(places_coordinate_path, &places_path);
    double places_path_length = mav_planning::computePathLength(places_path);
    int places_num_vertices = places_path.size();
    scene_graph_places_timer.Stop();

    LOG(INFO) << "A* places\n"
              << "- Success? " << success << '\n'
              << "- Path length: " << places_path_length << '\n'
              << "- Vertices: " << places_num_vertices;


    LOG(INFO) << "A* on Rooms Layer.";
    mav_trajectory_generation::timing::Timer scene_graph_rooms_timer(
        "scene_graph/plan/rooms");
    success = rooms_skeleton_planner_.getPath(
        start_point, goal_point, &rooms_coordinate_path);
    mav_msgs::EigenTrajectoryPointVector rooms_path;
    convertCoordinatePathToPath(rooms_coordinate_path, &rooms_path);
    double rooms_path_length = mav_planning::computePathLength(rooms_path);
    int rooms_num_vertices = rooms_path.size();
    scene_graph_rooms_timer.Stop();

    LOG(INFO) << "A* rooms\n"
              << "- Success? " << success << '\n'
              << "- Path length: " << rooms_path_length << '\n'
              << "- Vertices: " << rooms_num_vertices;

    LOG(INFO) << "A* on Buildings Layer.";
    mav_trajectory_generation::timing::Timer scene_graph_buildings_timer(
        "scene_graph/plan/buildings");
    success = buildings_skeleton_planner_.getPath(
        start_point, goal_point, &buildings_coordinate_path);
    mav_msgs::EigenTrajectoryPointVector buildings_path;
    convertCoordinatePathToPath(buildings_coordinate_path, &buildings_path);
    double buildings_path_length = mav_planning::computePathLength(buildings_path);
    int buildings_num_vertices = buildings_path.size();
    scene_graph_buildings_timer.Stop();

    LOG(INFO) << "A* buildings\n"
              << "- Success? " << success << '\n'
              << "- Path length: " << buildings_path_length << '\n'
              << "- Vertices: " << buildings_num_vertices;

    LOG(ERROR) << "Done running A* on scene-graph.";
  }

  if (visualize_) {
    LOG(INFO) << "Visualize all paths.";
    path_marker_pub_.publish(marker_array);
  }

  ROS_INFO_STREAM("All timings: "
                  << std::endl
                  << mav_trajectory_generation::timing::Timing::Print());
}

void SceneGraphGlobalPlanner::convertCoordinatePathToPath(
    const voxblox::AlignedVector<voxblox::Point>& coordinate_path,
    mav_msgs::EigenTrajectoryPointVector* path) const {
  CHECK_NOTNULL(path);
  path->clear();
  path->reserve(coordinate_path.size());

  for (const voxblox::Point& voxblox_point : coordinate_path) {
    mav_msgs::EigenTrajectoryPoint point;
    point.position_W = voxblox_point.cast<double>();
    path->push_back(point);
  }
}

double SceneGraphGlobalPlanner::getMapDistance(
    const Eigen::Vector3d& position) const {
  if (!esdf_server_.getEsdfMapPtr()) {
    return 0.0;
  }
  double distance = 0.0;
  if (!esdf_server_.getEsdfMapPtr()->getDistanceAtPosition(position,
                                                           &distance)) {
    return 0.0;
  }
  return distance;
}

bool SceneGraphGlobalPlanner::publishPathCallback(
    std_srvs::EmptyRequest& request,
    std_srvs::EmptyResponse& response) {
  ROS_INFO("Publishing waypoints.");

  geometry_msgs::PoseArray pose_array;
  pose_array.poses.reserve(last_waypoints_.size());
  for (const mav_msgs::EigenTrajectoryPoint& point : last_waypoints_) {
    geometry_msgs::PoseStamped pose_stamped;
    mav_msgs::msgPoseStampedFromEigenTrajectoryPoint(point, &pose_stamped);
    pose_array.poses.push_back(pose_stamped.pose);
  }

  pose_array.header.frame_id = frame_id_;
  waypoint_list_pub_.publish(pose_array);
  return true;
}
}
