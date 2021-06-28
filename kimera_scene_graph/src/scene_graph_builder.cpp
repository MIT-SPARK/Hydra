#include "kimera_scene_graph/scene_graph_builder.h"
#include "kimera_scene_graph/building_finder.h"
#include "kimera_scene_graph/connectivity_utils.h"
#include "kimera_scene_graph/voxblox_utils.h"
#include "kimera_scene_graph/wall_finder.h"

#include <voxblox_msgs/Mesh.h>
#include <kimera_semantics_ros/ros_params.h>
#include <pcl/PolygonMesh.h>
#include <pcl/conversions.h>
#include <pcl_ros/point_cloud.h>

#include <glog/logging.h>

namespace kimera {

using std_srvs::SetBool;
using std_srvs::Trigger;

using SemanticMeshMap = std::map<SemanticLabel, SubMesh>;

SemanticMeshMap getSemanticMeshes(
    const SemanticIntegratorBase::SemanticConfig& config,
    const pcl::PolygonMesh::ConstPtr& mesh) {
  CHECK(mesh);
  CHECK(config.semantic_label_to_color_);

  pcl::PointCloud<pcl::PointXYZRGB> vertices;
  pcl::fromPCLPointCloud2(mesh->cloud, vertices);

  SemanticMeshMap map;
  std::map<size_t, SemanticLabel> vertex_to_label;
  std::map<size_t, size_t> vertex_to_subvertex;
  for (size_t i = 0; i < vertices.size(); ++i) {
    const pcl::PointXYZRGB& color_point = vertices.at(i);
    const HashableColor color(color_point.r, color_point.g, color_point.b, 255);
    const SemanticLabel semantic_label =
        config.semantic_label_to_color_->getSemanticLabelFromColor(color);
    vertex_to_label[i] = semantic_label;

    if (!map.count(semantic_label)) {
      SubMesh submesh;
      submesh.vertices.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
      submesh.mesh.reset(new pcl::PolygonMesh());
      map[semantic_label] = submesh;
    }

    vertex_to_subvertex[i] = map.at(semantic_label).vertices->size();
    map.at(semantic_label).vertices->push_back(color_point);
    map.at(semantic_label).vertex_map.push_back(i);
  }

  for (const auto& face : mesh->polygons) {
    CHECK_EQ(face.vertices.size(), 3u);
    SemanticLabel label = vertex_to_label.at(face.vertices[0]);
    if (label == vertex_to_label.at(face.vertices[1]) &&
        label == vertex_to_label.at(face.vertices[2])) {
      pcl::Vertices new_face;
      new_face.vertices.push_back(vertex_to_subvertex.at(face.vertices[0]));
      new_face.vertices.push_back(vertex_to_subvertex.at(face.vertices[1]));
      new_face.vertices.push_back(vertex_to_subvertex.at(face.vertices[2]));
      map.at(label).mesh->polygons.push_back(new_face);
    }
  }

  for (auto& label_mesh_pair : map) {
    pcl::toPCLPointCloud2(*label_mesh_pair.second.vertices,
                          label_mesh_pair.second.mesh->cloud);
  }

  LOG(INFO) << "Found " << map.size() << " submeshes";
  return map;
}

// TODO(nathan) clean up
void SceneGraphBuilder::loadParams() {
  semantic_config_ = getSemanticTsdfIntegratorConfigFromRosParam(nh_private_);
  nh_private_.getParam("object_finder_type", object_finder_type_);

  nh_private_.getParam("world_frame", world_frame_);
  nh_private_.getParam("scene_graph_output_path", scene_graph_output_path_);
  nh_private_.getParam("room_finder_esdf_slice_level",
                       room_finder_esdf_slice_level_);

  CHECK(nh_private_.getParam("dynamic_semantic_labels", dynamic_labels_));
  CHECK(nh_private_.getParam("stuff_labels", stuff_labels_));
  CHECK(nh_private_.getParam("walls_labels", walls_labels_));
  CHECK(nh_private_.getParam("floor_labels", floor_labels_));

  default_building_color_ = std::vector<int>{169, 8, 194};
  nh_private_.getParam("default_building_color", default_building_color_);
  CHECK_EQ(3u, default_building_color_.size())
      << "Color must be three elements";
}

SceneGraphBuilder::SceneGraphBuilder(const ros::NodeHandle& nh,
                                     const ros::NodeHandle& nh_private)
    : nh_(nh),
      nh_private_(nh_private),
      world_frame_("world"),
      scene_graph_output_path_(""),
      object_finder_type_(0),
      semantic_pcl_pubs_("pcl", nh_private),
      visualizer_(nh, nh_private_, getDefaultLayerIds()) {
  scene_graph_ = std::make_shared<DynamicSceneGraph>();

  loadParams();

  debug_pub_ = nh_private_.advertise<voxblox_msgs::Mesh>("debug_mesh", 10);

  utils::VoxbloxConfig semantic_config =
      utils::loadVoxbloxConfig(ros::NodeHandle("~/semantic")).value();
  pcl::PolygonMesh::Ptr semantic_mesh;
  utils::loadVoxbloxInfo(semantic_config, esdf_layer_, semantic_mesh, &debug_pub_);
  scene_graph_->setMesh(semantic_mesh, true);

  utils::VoxbloxConfig rgb_config =
      utils::loadVoxbloxConfig(ros::NodeHandle("~/rgb")).value();
  rgb_config.load_esdf = false;
  voxblox::Layer<voxblox::EsdfVoxel>::Ptr temp;
  utils::loadVoxbloxInfo(rgb_config, temp, rgb_mesh_);

  object_finder_ = kimera::make_unique<ObjectFinder>(
      world_frame_, static_cast<ObjectFinderType>(object_finder_type_));
  room_finder_ = kimera::make_unique<RoomFinder>(
      nh_private, world_frame_, room_finder_esdf_slice_level_);

  rqt_callback_ =
      boost::bind(&SceneGraphBuilder::rqtReconfigureCallback, this, _1, _2);
  rqt_server_.setCallback(rqt_callback_);

  reconstruct_scene_graph_srv_ = nh_private_.advertiseService(
      "reconstruct_scene_graph", &SceneGraphBuilder::reconstructSrvCb, this);

  visualizer_srv_ = nh_private_.advertiseService(
      "visualize", &SceneGraphBuilder::visualizeSrvCb, this);
}

void SceneGraphBuilder::reconstruct() {
  if (!scene_graph_) {
    scene_graph_.reset(new DynamicSceneGraph());
  }

  std::string skeleton_file;
  if (!nh_private_.getParam("skeleton_file", skeleton_file)) {
    LOG(FATAL) << "Failed to get skeleton file";
  }
  utils::fillLayerFromSkeleton(skeleton_file, scene_graph_.get());

  SemanticMeshMap semantic_meshes =
      getSemanticMeshes(semantic_config_, scene_graph_->getMesh());

  for (const auto& label_mesh_pair : semantic_meshes) {
    SemanticLabel semantic_label = label_mesh_pair.first;
    CHECK(label_mesh_pair.second.vertices);
    CHECK(label_mesh_pair.second.mesh);

    if (label_mesh_pair.second.vertices->empty()) {
      LOG(WARNING) << "Semantic pointcloud for label "
                   << std::to_string(semantic_label) << " is empty.";
      continue;
    }

    semantic_pcl_pubs_.publish(semantic_label,
                               *label_mesh_pair.second.vertices);

    bool is_dynamic = std::find(dynamic_labels_.begin(),
                                dynamic_labels_.end(),
                                semantic_label) != dynamic_labels_.end();
    if (is_dynamic) {
      continue;
    }

    bool is_structure =
        std::find(stuff_labels_.begin(), stuff_labels_.end(), semantic_label) !=
        stuff_labels_.end();

    if (is_structure) {
      // TODO(nathan) do something here
      continue;
    }

    const auto& label_color_vbx =
        semantic_config_.semantic_label_to_color_->getColorFromSemanticLabel(
            semantic_label);

    NodeColor label_color;
    label_color << label_color_vbx.r, label_color_vbx.g, label_color_vbx.b;
    object_finder_->addObjectsToGraph(label_mesh_pair.second,
                                      label_color,
                                      semantic_label,
                                      scene_graph_.get());
  }

  VLOG(1) << "Start Room finding.";
  CHECK(room_finder_);
  RoomHullMap room_hulls =
      room_finder_->findRooms(*esdf_layer_, scene_graph_.get());
  VLOG(1) << "Finished Room finding.";

  VLOG(1) << "Start Building finding.";
  NodeColor building_color;
  building_color << default_building_color_.at(0),
      default_building_color_.at(1), default_building_color_.at(2);
  findBuildings(scene_graph_.get(), building_color);
  VLOG(1) << "Finished Building finding.";

  VLOG(1) << "Start Places Segmentation";
  findPlacesRoomConnectivity(scene_graph_.get(), room_hulls, kEsdfTruncation);
  VLOG(1) << "Finished Places Segmentation";

  VLOG(1) << "Start Room Connectivity finder";
  findRoomConnectivity(scene_graph_.get());
  VLOG(1) << "Finished Room Connectivity finder";

  VLOG(1) << "Start Object Connectivity finder";
  findObjectPlaceConnectivity(scene_graph_.get());
  VLOG(1) << "Finished Object Connectivity finder";

  if (!walls_labels_.empty() && semantic_meshes.count(walls_labels_.front())) {
    VLOG(1) << "Clustering walls.";
    segmented_walls_mesh_ =
        findWalls(semantic_meshes.at(walls_labels_.front()), *scene_graph_);
    VLOG(1) << "Done clustering walls.";
  } else {
    ROS_WARN("No mesh vertices found with the given wall semantic class!");
  }

  visualize();

  // if (!scene_graph_output_path_.empty()) {
  // LOG(INFO) << "Saving Scene-Graph to file: "
  //<< scene_graph_output_path_.c_str();
  //// TODO(nathan) serialization
  //// save(*scene_graph_, scene_graph_output_path_);
  // LOG(INFO) << "Done saving Scene-Graph to file";
  //}
}

void SceneGraphBuilder::visualize() {
  // TODO(nathan) fix exception here
  // visualizer_.clear();

  if (!scene_graph_) {
    ROS_ERROR("Scene graph wasn't constructed yet. Not visualizing");
    return;
  }

  visualizer_.visualize(scene_graph_);

  if (rgb_mesh_) {
    // visualizer_.visualizeMesh(*rgb_mesh_.get(), true);
  } else {
    ROS_WARN("Invalid RGB mesh when visualizing");
  }

  if (segmented_walls_mesh_) {
    visualizer_.visualizeWalls(*segmented_walls_mesh_);
  }

  // TODO(nathan) dynamic stuff
  // LOG(INFO) << "Visualizing Human Pose Graphs";
  // dynamic_scene_graph_.visualizePoseGraphs();

  // LOG(INFO) << "Visualizing Human Skeletons";
  // dynamic_scene_graph_.publishOptimizedMeshesAndVis();

  // TODO(nathan) add back in sliced esdf and room clusters visualization
}

bool SceneGraphBuilder::reconstructSrvCb(SetBool::Request&,
                                         SetBool::Response&) {
  LOG(INFO) << "Requested scene graph reconstruction.";
  scene_graph_->clear();
  reconstruct();
  return true;
}

bool SceneGraphBuilder::visualizeSrvCb(Trigger::Request&,
                                       Trigger::Response& resp) {
  if (scene_graph_) {
    visualize();
    resp.success = true;
  } else {
    resp.success = false;
    resp.message = "scene graph not initialized";
  }
  return true;
}

void SceneGraphBuilder::rqtReconfigureCallback(RqtSceneGraphConfig& config,
                                               uint32_t /*level*/) {
  ROS_INFO("Updating Object Finder params.");

  object_finder_->updateClusterEstimator(
      static_cast<ObjectFinderType>(config.object_finder_type));

  EuclideanClusteringParams ec_params;
  ec_params.cluster_tolerance = config.cluster_tolerance;
  ec_params.max_cluster_size = config.ec_max_cluster_size;
  ec_params.min_cluster_size = config.ec_min_cluster_size;
  object_finder_->updateEuclideanClusterParams(ec_params);

  RegionGrowingClusteringParams rg_params;
  rg_params.curvature_threshold = config.curvature_threshold;
  rg_params.max_cluster_size = config.rg_max_cluster_size;
  rg_params.min_cluster_size = config.rg_min_cluster_size;
  rg_params.normal_estimator_neighbour_size =
      config.normal_estimator_neighbour_size;
  rg_params.number_of_neighbours = config.number_of_neighbours;
  rg_params.smoothness_threshold = config.smoothness_threshold;
  object_finder_->updateRegionGrowingParams(rg_params);

  ROS_INFO_STREAM("Object finder: " << *object_finder_);

  ROS_WARN("Parameters updated. Reconstruct scene graph to see the effects.");
}

}  // namespace kimera
