#include "kimera_scene_graph/scene_graph_builder.h"
#include "kimera_scene_graph/building_finder.h"
#include "kimera_scene_graph/connectivity_utils.h"
#include "kimera_scene_graph/voxblox_conversions.h"

#include <glog/logging.h>

// clang-format off
// TODO(nathan) fix when voxblox fixes the missing header
#include <voxblox/io/mesh_ply.h>
#include <voxblox_ros/mesh_pcl.h>
// clang-format on
#include <voxblox/mesh/mesh_integrator.h>
#include <voxblox_ros/ros_params.h>
#include <voxblox_skeleton/io/skeleton_io.h>
#include <voxblox_skeleton/ros/skeleton_vis.h>
#include <voxblox_skeleton/skeleton_generator.h>
#include <kimera_semantics_ros/ros_params.h>

#include <pcl/common/copy_point.h>
#include <pcl/common/io.h>
#include <pcl/common/transforms.h>

namespace kimera {

// TODO(nathan) this should be part of voxblox, but...
inline auto operator==(const vxb::Color& lhs, const vxb::Color& rhs) -> bool {
  return lhs.r == rhs.r && lhs.g == rhs.g && lhs.b == rhs.b && lhs.a == rhs.a;
}

// TODO(nathan) this should be part of voxblox, but...
inline auto operator!=(const vxb::Color& lhs, const vxb::Color& rhs) -> bool {
  return not(lhs == rhs);
}

SceneGraphBuilder::SceneGraphBuilder(const ros::NodeHandle& nh,
                                     const ros::NodeHandle& nh_private)
    : nh_(nh),
      nh_private_(nh_private),
      color_clustered_pcl_pub_(),
      walls_clustered_pcl_pub_(),
      room_centroids_pub_(),
      mesh_pub_(),
      polygon_mesh_pub_(),
      sparse_graph_pub_(),
      world_frame_("world"),
      scene_graph_output_path_(""),
      semantic_pcl_pubs_("pcl", nh_private),
      rqt_server_(),
      rqt_callback_(),
      reconstruct_scene_graph_srv_(),
      enclosing_wall_finder_(nullptr),
      object_finder_(nullptr),
      room_finder_(nullptr),
      scene_graph_(nullptr),
      visualizer_(nh, nh_private_, getDefaultLayerIds()),
      semantic_config_(
          getSemanticTsdfIntegratorConfigFromRosParam(nh_private)) {
  // dynamic_scene_graph_(nh, nh_private) {
  // Create Scene graph
  scene_graph_ = std::make_shared<SceneGraph>();

  // TODO(Toni): put all of this in scene graph reconstructor config!
  // Build object finder
  int object_finder_type = 0;
  nh_private.getParam("object_finder_type", object_finder_type);

  ros::param::get("~world_frame", world_frame_);
  ros::param::get("~scene_graph_output_path", scene_graph_output_path_);
  ros::param::get("~room_finder_esdf_slice_level",
                  room_finder_esdf_slice_level_);
  ros::param::get("~tsdf_voxel_size", voxel_size_);
  ros::param::get("~tsdf_voxels_per_side", voxels_per_side_);

  tsdf_layer_.reset(
      new vxb::Layer<vxb::TsdfVoxel>(voxel_size_, voxels_per_side_));
  esdf_layer_.reset(
      new vxb::Layer<vxb::EsdfVoxel>(voxel_size_, voxels_per_side_));

  // purple-ish
  default_building_color_ = std::vector<int>{169, 8, 194};
  nh_private_.getParam("default_building_color", default_building_color_);
  CHECK_EQ(3u, default_building_color_.size()) << "Color must be three elements";

  // Load RGB TSDF for RGB mesh, overkill...
  double rgb_voxel_size = 0.025;
  double rgb_voxels_per_side = 16;
  ros::param::get("~rgb_tsdf_voxel_size", rgb_voxel_size);
  ros::param::get("~rgb_tsdf_voxels_per_side", rgb_voxels_per_side);
  tsdf_layer_rgb_.reset(
      new vxb::Layer<vxb::TsdfVoxel>(rgb_voxel_size, rgb_voxels_per_side));

  esdf_integrator_.reset(new vxb::EsdfIntegrator(
      vxb::getEsdfIntegratorConfigFromRosParam(nh_private),
      tsdf_layer_.get(),
      esdf_layer_.get()));

  // Build finders
  enclosing_wall_finder_ =
      kimera::make_unique<EnclosingWallFinder>(world_frame_);
  object_finder_ = kimera::make_unique<ObjectFinder>(
      world_frame_, static_cast<ObjectFinderType>(object_finder_type));
  room_finder_ = kimera::make_unique<RoomFinder>(
      nh_private, world_frame_, room_finder_esdf_slice_level_);

  // Get labels of interesting things
  CHECK(nh_private.getParam("dynamic_semantic_labels", dynamic_labels_));
  CHECK(nh_private.getParam("stuff_labels", stuff_labels_));
  CHECK(nh_private.getParam("walls_labels", walls_labels_));
  CHECK(nh_private.getParam("floor_labels", floor_labels_));

  // Pre-load map
  std::string file_path;
  LOG(INFO) << "Loading TSDF map:";
  CHECK(nh_private.getParam("load_tsdf_map", file_path));
  CHECK(tsdf_layer_);
  loadTsdfMap(file_path, tsdf_layer_.get());
  LOG(INFO) << "Loading RGB TSDF map:";
  CHECK(nh_private.getParam("load_rgb_tsdf_map", file_path));
  CHECK(tsdf_layer_rgb_);
  loadTsdfMap(file_path, tsdf_layer_rgb_.get());
  LOG(INFO) << "Loading ESDF map:";
  CHECK(nh_private.getParam("load_esdf_map", file_path));
  CHECK(nh_private.getParam("build_esdf_batch", build_esdf_batch_));
  loadEsdfMap(file_path);
  LOG(INFO) << "Loading Skeleton map:";
  CHECK(nh_private.getParam("load_sparse_graph", file_path));
  CHECK(vxb::io::loadSparseSkeletonGraphFromFile(file_path,
                                                 &sparse_skeleton_graph_));
  LOG(INFO) << "Successfully loaded Skeleton map";

  // ROS publishers.
  sparse_graph_pub_ = nh_private_.advertise<visualization_msgs::MarkerArray>(
      "sparse_graph", 1, true);
  color_clustered_pcl_pub_ =
      nh_private_.advertise<IntensityPointCloud>("clustered_pcls", 1, true);
  walls_clustered_pcl_pub_ = nh_private_.advertise<IntensityPointCloud>(
      "walls_clustered_pcls", 1, true);
  room_centroids_pub_ =
      nh_private_.advertise<ColorPointCloud>("room_centroids", 1, true);
  mesh_pub_ =
      nh_private_.advertise<voxblox_msgs::Mesh>("semantic_mesh", 1, true);
  rgb_mesh_pub_ =
      nh_private_.advertise<voxblox_msgs::Mesh>("rgb_mesh", 1, true);
  polygon_mesh_pub_ = nh_private_.advertise<pcl_msgs::PolygonMesh>(
      "polygon_semantic_mesh", 1, true);

  // Attach rqt reconfigure
  rqt_callback_ =
      boost::bind(&SceneGraphBuilder::rqtReconfigureCallback, this, _1, _2);
  rqt_server_.setCallback(rqt_callback_);

  // Add rosservice to reconstruct scene graph, this avoids having to
  // rebuild the simulation world and reintegrate the generated pointclouds.
  reconstruct_scene_graph_srv_ = nh_private_.advertiseService(
      "reconstruct_scene_graph",
      &SceneGraphBuilder::sceneGraphReconstructionServiceCall,
      this);

  // Add rosservice to reconstruct scene graph, this avoids having to
  // rebuild the simulation world and reintegrate the generated pointclouds.
  visualizer_srv_ = nh_private_.advertiseService(
      "visualize", &SceneGraphBuilder::visualizerServiceCall, this);
}

bool SceneGraphBuilder::sceneGraphReconstructionServiceCall(
    std_srvs::SetBool::Request& /*request*/,
    std_srvs::SetBool::Response&) {
  LOG(INFO) << "Requested scene graph reconstruction.";
  scene_graph_->clear();
  sceneGraphReconstruction();
  return true;
}

bool SceneGraphBuilder::visualizerServiceCall(
    std_srvs::Trigger::Request&,
    std_srvs::Trigger::Response& resp) {
  if (scene_graph_) {
    visualize();
    resp.success = true;
  } else {
    resp.success = false;
    resp.message = "scene graph not initialized";
  }
  return true;
}

bool SceneGraphBuilder::loadTsdfMap(const std::string& file_path,
                                    vxb::Layer<vxb::TsdfVoxel>* tsdf_layer) {
  CHECK_NOTNULL(tsdf_layer);
  constexpr bool kMulitpleLayerSupport = true;
  bool success = vxb::io::LoadBlocksFromFile(
      file_path,
      vxb::Layer<vxb::TsdfVoxel>::BlockMergingStrategy::kReplace,
      kMulitpleLayerSupport,
      tsdf_layer);
  if (success) {
    LOG(INFO) << "Successfully loaded TSDF layer.";
  } else {
    LOG(INFO) << "Failed to load TSDF layer.";
  }
  return success;
}

bool SceneGraphBuilder::loadEsdfMap(const std::string& file_path) {
  constexpr bool kMulitpleLayerSupport = true;
  bool success = vxb::io::LoadBlocksFromFile(
      file_path,
      vxb::Layer<vxb::EsdfVoxel>::BlockMergingStrategy::kReplace,
      kMulitpleLayerSupport,
      esdf_layer_.get());
  if (success) {
    LOG(INFO) << "Successfully loaded ESDF layer.";
  } else {
    LOG(INFO) << "Failed to load ESDF layer.";
  }
  return success;
}

void SceneGraphBuilder::reconstructMeshOutOfTsdf(
    vxb::Layer<vxb::TsdfVoxel>* tsdf,
    vxb::MeshLayer::Ptr mesh) {
  CHECK(tsdf);
  vxb::MeshIntegratorConfig config;
  vxb::MeshIntegrator<vxb::TsdfVoxel> integrator(config, *tsdf, mesh.get());
  integrator.generateMesh(false, false);
}

void SceneGraphBuilder::reconstructEsdfOutOfTsdf(bool save_to_file) {
  LOG(INFO) << "Building ESDF layer.";
  esdf_integrator_->setFullEuclidean(true);
  esdf_integrator_->updateFromTsdfLayerBatch();
  LOG(INFO) << "Saving ESDF layer.";
  if (save_to_file) {
    //    esdf_layer_->saveToFile("/home/tonirv/tesse_esdf.vxblx");
    esdf_layer_->saveToFile(
        "/home/lisa/Documents/goseek_scene_01/scene_1/esdf_from_tsdf.vxblx");
  }
  LOG(INFO) << "Done building ESDF layer.";
}

// TODO(nathan) PGMO instead of voxblox
void SceneGraphBuilder::sceneGraphReconstruction() {
  CHECK(scene_graph_);  // TODO(nathan) remove scene graph checks
  // This creates the places layer in the scene graph
  utils::fillLayerFromSkeleton(sparse_skeleton_graph_, scene_graph_.get());

  // TODO(nathan) create something to hand tsdf, esdf, mesh and skeleton to the
  // builder
  if (build_esdf_batch_) {
    reconstructEsdfOutOfTsdf(true);
  }

  VLOG(1) << "Reconstructing Mesh out of TSDF.";
  semantic_mesh_ =
      vxb::MeshLayer::Ptr(new vxb::MeshLayer(tsdf_layer_->block_size()));
  reconstructMeshOutOfTsdf(tsdf_layer_.get(), semantic_mesh_);
  VLOG(1) << "Finished reconstructing Mesh out of TSDF.";

  VLOG(1) << "Reconstructing RGB Mesh out of RGB TSDF.";
  rgb_mesh_ =
      vxb::MeshLayer::Ptr(new vxb::MeshLayer(tsdf_layer_rgb_->block_size()));
  reconstructMeshOutOfTsdf(tsdf_layer_rgb_.get(), rgb_mesh_);
  VLOG(1) << "Finished reconstructing RGB Mesh out of RGB TSDF.";

  // TODO(nathan) pair these up maybe
  // Use original colors to extract semantic labels.
  SemanticPointCloudMap semantic_pointclouds;
  getSemanticPointcloudsFromMesh(
      semantic_mesh_, vxb::ColorMode::kColor, &semantic_pointclouds);

  SemanticMeshMap semantic_meshes;
  getSemanticMeshesFromMesh(
      semantic_mesh_, vxb::ColorMode::kColor, &semantic_meshes);

  vxb::Mesh::Ptr walls_mesh = nullptr;
  for (const auto& semantic_pcl_it : semantic_pointclouds) {
    const SemanticLabel& semantic_label = semantic_pcl_it.first;
    const SemanticPointCloud::Ptr& semantic_pcl = semantic_pcl_it.second;
    CHECK(semantic_pcl);
    CHECK(semantic_meshes.find(semantic_label) != semantic_meshes.end())
        << "Couldn't find semantic mesh for semantic label: "
        << std::to_string(semantic_label);
    if (semantic_pcl->empty()) {
      LOG(WARNING) << "Semantic pointcloud for label "
                   << std::to_string(semantic_label) << " is empty.";
      continue;
    }
    // Publish semantic pointcloud
    semantic_pcl_pubs_.publish(semantic_label, *semantic_pcl);

    // Estimate objects in semantic pointcloud.
    // Only for those semantic labels that are not stuff (aka only for
    // things).
    bool is_not_stuff =
        std::find(stuff_labels_.begin(), stuff_labels_.end(), semantic_label) ==
        stuff_labels_.end();
    bool is_not_dynamic = std::find(dynamic_labels_.begin(),
                                    dynamic_labels_.end(),
                                    semantic_label) == dynamic_labels_.end();
    if (is_not_dynamic) {
      if (is_not_stuff) {
        const auto& label_color_vbx =
            semantic_config_.semantic_label_to_color_
                ->getColorFromSemanticLabel(semantic_label);

        NodeColor label_color;
        label_color << label_color_vbx.r, label_color_vbx.g, label_color_vbx.b;
        object_finder_->addObjectsToGraph(
            semantic_pcl, label_color, semantic_label, scene_graph_.get());
      } else {
        LOG(INFO) << "Skipping object extraction for `stuff` semantic label: "
                  << std::to_string(semantic_label);

        bool is_floor = std::find(floor_labels_.begin(),
                                  floor_labels_.end(),
                                  semantic_label) != floor_labels_.end();
        if (is_floor) {
          // Can we use wall finder for ceiling/floor?
          // Centroids wall_centroids;
          // std::vector<ColorPointCloud::Ptr> wall_pcls;
          // const auto& pcls =
          //     wall_finder_->findWalls(semantic_pcl, &wall_centroids,
          //     &wall_pcls);
          // ceil_floor_pcl_pub_.publish(pcls);
        }

        bool is_walls = std::find(walls_labels_.begin(),
                                  walls_labels_.end(),
                                  semantic_label) != walls_labels_.end();
        if (is_walls) {
          // Store walls mesh for later segmentation
          walls_mesh = semantic_meshes.at(semantic_label);
        }
      }
    } else {
      LOG(INFO) << "Not extracting things/stuff from dynamic "
                   "label: "
                << std::to_string(semantic_label);
    }
  }

  VLOG(1) << "Start Room finding.";
  CHECK(room_finder_);
  room_finder_->findRooms(*esdf_layer_, scene_graph_.get());
  VLOG(1) << "Finished Room finding.";

  VLOG(1) << "Start Building finding.";
  NodeColor building_color;
  building_color << default_building_color_.at(0),
      default_building_color_.at(1), default_building_color_.at(2);
  findBuildings(scene_graph_.get(), building_color);
  VLOG(1) << "Finished Building finding.";

  VLOG(1) << "Start Places Segmentation";
  findPlacesRoomConnectivity(scene_graph_.get(), kEsdfTruncation);
  VLOG(1) << "Finished Places Segmentation";

  VLOG(1) << "Start Room Connectivity finder";
  findRoomConnectivity(scene_graph_.get());
  VLOG(1) << "Finished Room Connectivity finder";

  VLOG(1) << "Start Object Connectivity finder";
  findObjectPlaceConnectivity(scene_graph_.get());
  VLOG(1) << "Finished Object Connectivity finder";

  if (walls_mesh) {
    VLOG(1) << "Clustering walls...";
    segmented_walls_mesh_.reset(new voxblox::Mesh());
    enclosing_wall_finder_->findWalls(
        *walls_mesh, *scene_graph_, segmented_walls_mesh_.get());
    VLOG(1) << "Done clustering walls...";
  } else {
    ROS_WARN("Not segmenting walls! No walls mesh found.");
  }
  //////////////////////////////////////////////////////////////////////////////

  visualize();

  if (!scene_graph_output_path_.empty()) {
    LOG(INFO) << "Saving Scene-Graph to file: "
              << scene_graph_output_path_.c_str();
    // TODO(nathan) serialization
    // save(*scene_graph_, scene_graph_output_path_);
    LOG(INFO) << "Done saving Scene-Graph to file";
  }
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
    visualizer_.visualizeMesh(
        rgb_mesh_.get(), voxblox::ColorMode::kLambertColor, true);
  } else {
    ROS_WARN("Invalid RGB mesh when visualizing");
  }

  if (semantic_mesh_) {
    visualizer_.visualizeMesh(
        semantic_mesh_.get(), voxblox::ColorMode::kLambertColor, false);
  } else {
    ROS_WARN("Invalid semantic mesh when visualizing");
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

void SceneGraphBuilder::getSemanticPointcloudsFromMesh(
    const vxb::MeshLayer::ConstPtr& mesh_layer,
    const vxb::ColorMode& color_mode,
    SemanticPointCloudMap* semantic_pointclouds) {
  CHECK_NOTNULL(semantic_pointclouds);
  semantic_pointclouds->clear();

  vxb::BlockIndexList mesh_indices;
  mesh_layer->getAllAllocatedMeshes(&mesh_indices);

  for (const vxb::BlockIndex& block_index : mesh_indices) {
    vxb::Mesh::ConstPtr mesh = mesh_layer->getMeshPtrByIndex(block_index);

    if (!mesh->hasVertices()) {
      continue;
    }

    // Check that we can actually do the color stuff.
    if (color_mode == vxb::kColor || color_mode == vxb::kLambertColor) {
      CHECK(mesh->hasColors());
    }
    if (color_mode == vxb::kNormals || color_mode == vxb::kLambert ||
        color_mode == vxb::kLambertColor) {
      CHECK(mesh->hasNormals());
    }

    for (size_t i = 0u; i < mesh->vertices.size(); i++) {
      ColorPoint point;
      point.x = mesh->vertices[i].x();
      point.y = mesh->vertices[i].y();
      point.z = mesh->vertices[i].z();

      vxb::Color color;
      vxb::colorMsgToVoxblox(getVertexColor(*mesh, color_mode, i), &color);
      point.r = color.r;
      point.g = color.g;
      point.b = color.b;

      // Add points that are red to centroid, very approx...
      const vxb::Color& original_color = mesh->colors[i];
      CHECK(semantic_config_.semantic_label_to_color_);
      const SemanticLabel& semantic_label =
          semantic_config_.semantic_label_to_color_->getSemanticLabelFromColor(
              HashableColor(
                  original_color.r, original_color.g, original_color.b, 255u));
      if (semantic_pointclouds->find(semantic_label) ==
          semantic_pointclouds->end()) {
        // We have never seen this color (aka semantic label) before.
        // Store idx in vector of semantic poinclouds for the this new semantic
        // label.
        // Make new semantic pointcloud for the new semantic label.
        SemanticPointCloud::Ptr new_semantic_pcl =
            boost::make_shared<SemanticPointCloud>();
        new_semantic_pcl->header.frame_id = world_frame_;
        (*semantic_pointclouds)[semantic_label] = new_semantic_pcl;
      }
      // Get semantic pointcloud corresponding to the current semantic label.
      SemanticPointCloud::Ptr semantic_pcl =
          semantic_pointclouds->at(semantic_label);

      // Create and accumulate points
      semantic_pcl->push_back(point);
    }
  }
}

void SceneGraphBuilder::getSemanticMeshesFromMesh(
    const vxb::MeshLayer::ConstPtr& mesh_layer,
    const vxb::ColorMode& color_mode,
    SemanticMeshMap* semantic_meshes) {
  CHECK_NOTNULL(semantic_meshes);
  semantic_meshes->clear();

  vxb::BlockIndexList mesh_indices;
  mesh_layer->getAllAllocatedMeshes(&mesh_indices);

  for (const vxb::BlockIndex& block_index : mesh_indices) {
    vxb::Mesh::ConstPtr mesh = mesh_layer->getMeshPtrByIndex(block_index);

    if (!mesh->hasVertices()) {
      continue;
    }

    // Check that we can actually do the color stuff.
    if (color_mode == vxb::kColor || color_mode == vxb::kLambertColor) {
      CHECK(mesh->hasColors());
    }
    if (color_mode == vxb::kNormals || color_mode == vxb::kLambert ||
        color_mode == vxb::kLambertColor) {
      CHECK(mesh->hasNormals());
    }

    const bool mesh_has_normals = mesh->hasNormals();
    CHECK(mesh->hasColors());
    for (size_t i = 0u; i < mesh->vertices.size(); i += 3) {
      // One triangle
      vxb::Point vtx_1 = mesh->vertices[i];
      vxb::Point vtx_2 = mesh->vertices[i + 1];
      vxb::Point vtx_3 = mesh->vertices[i + 2];

      vxb::Color color_1 = mesh->colors[i];
      vxb::Color color_2 = mesh->colors[i + 1];
      vxb::Color color_3 = mesh->colors[i + 2];

      if (color_1 != color_2 || color_1 != color_3) {
        // The triangle has not the same color, aka semantic label...
        // Discard.
        // TODO(Toni): this also discards the vertices which could be
        // potentially saved...
        continue;
      }

      CHECK(semantic_config_.semantic_label_to_color_);
      const SemanticLabel& semantic_label =
          semantic_config_.semantic_label_to_color_->getSemanticLabelFromColor(
              HashableColor(color_1.r, color_1.g, color_1.b, 255u));
      if (semantic_meshes->find(semantic_label) == semantic_meshes->end()) {
        // We have never seen this color (aka semantic label) before.
        // Store idx in vector of semantic poinclouds for the this new semantic
        // label.
        // Make new semantic mesh for the new semantic label.
        (*semantic_meshes)[semantic_label] = std::make_shared<vxb::Mesh>();
      }
      // Get semantic pointcloud corresponding to the current semantic label.
      vxb::Mesh::Ptr semantic_mesh = semantic_meshes->at(semantic_label);

      // Create and accumulate points
      semantic_mesh->vertices.push_back(vtx_1);
      semantic_mesh->vertices.push_back(vtx_2);
      semantic_mesh->vertices.push_back(vtx_3);

      semantic_mesh->colors.push_back(color_1);
      semantic_mesh->colors.push_back(color_2);
      semantic_mesh->colors.push_back(color_3);

      if (mesh_has_normals) {
        semantic_mesh->normals.push_back(mesh->normals[i]);
        semantic_mesh->normals.push_back(mesh->normals[i + 1]);
        semantic_mesh->normals.push_back(mesh->normals[i + 2]);
      }
    }
  }
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
