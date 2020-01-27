#include "kimera_scene_graph/scene_graph_server.h"

#include <voxblox_ros/mesh_pcl.h>

#include <pcl/common/copy_point.h>

#include <pcl/common/io.h>

namespace kimera {

SceneGraphSimulationServer::SceneGraphSimulationServer(
    const ros::NodeHandle& nh,
    const ros::NodeHandle& nh_private)
    : SemanticSimulationServer(nh, nh_private),
      color_clustered_pcl_pub_(),
      mesh_pub_(),
      polygon_mesh_pub_(),
      semantic_pcl_pubs_("pcl", nh_private),
      semantic_mesh_pubs_("mesh", nh_private),
      rqt_server_(),
      rqt_callback_(),
      room_finder_(nullptr),
      object_finder_(nullptr),
      scene_graph_(nh, nh_private),
      dynamic_scene_graph_(nh, nh_private),
      reconstruct_scene_graph_srv_(),
      load_map_srv_() {
  // TODO(Toni): put all of this in scene graph reconstructor config!
  // Build object finder
  int object_finder_type = 0;
  nh_private.param(
      "object_finder_type", object_finder_type, object_finder_type);
  object_finder_ = kimera::make_unique<ObjectFinder<ColorPoint>>(
      world_frame_, static_cast<ObjectFinderType>(object_finder_type));

  // Build room finder
  room_finder_ = kimera::make_unique<RoomFinder>(nh_private, world_frame_);

  // Get labels of interesting things
  CHECK(nh_private.getParam("stuff_labels", stuff_labels_));
  CHECK(nh_private.getParam("walls_labels", walls_labels_));
  CHECK(nh_private.getParam("floor_labels", floor_labels_));

  // Pre-load map
  std::string file_path;
  CHECK(nh_private.getParam("load_map", file_path));
  loadMap(file_path);

  // ROS publishers.
  color_clustered_pcl_pub_ =
      nh_private_.advertise<IntensityPointCloud>("clustered_pcls", 1, true);
  room_centroids_pub_ =
      nh_private_.advertise<ColoredPointCloud>("room_centroids", 1, true);
  mesh_pub_ =
      nh_private_.advertise<voxblox_msgs::Mesh>("semantic_mesh", 1, true);
  polygon_mesh_pub_ = nh_private_.advertise<pcl_msgs::PolygonMesh>(
      "polygon_semantic_mesh", 1, true);

  // Attach rqt reconfigure
  rqt_callback_ = boost::bind(
      &SceneGraphSimulationServer::rqtReconfigureCallback, this, _1, _2);
  rqt_server_.setCallback(rqt_callback_);

  // Add rosservice to reconstruct scene graph, this avoids having to
  // rebuild the simulation world and reintegrate the generated pointclouds.
  reconstruct_scene_graph_srv_ = nh_private_.advertiseService(
      "reconstruct_scene_graph",
      &SceneGraphSimulationServer::sceneGraphReconstructionServiceCall,
      this);
  load_map_srv_ = nh_private_.advertiseService(
      "load_map",
      &SemanticSimulationServer::loadMapCallback,
      dynamic_cast<SemanticSimulationServer*>(this));
}

bool SceneGraphSimulationServer::sceneGraphReconstructionServiceCall(
    std_srvs::Empty::Request& request,
    std_srvs::Empty::Response& response) {
  LOG(INFO) << "Requested scene graph reconstruction.";
  sceneGraphReconstruction();
  return true;
}

void SceneGraphSimulationServer::sceneGraphReconstruction() {
  // Also generate test mesh
  // TODO(Toni): this can be called from a rosservice now, so make sure
  // the sim world is ready before...
  // TODO(Toni): can't we load the mesh for now, this takes very long...
  vxb::MeshIntegratorConfig mesh_config;
  CHECK(tsdf_test_);
  // TODO(Toni): mind that the parent server already builds the mesh...
  vxb::MeshLayer::Ptr mesh_test(new vxb::MeshLayer(tsdf_test_->block_size()));
  vxb::MeshIntegrator<vxb::TsdfVoxel> mesh_integrator_test(
      mesh_config, tsdf_test_.get(), mesh_test.get());
  constexpr bool only_mesh_updated_blocks = false;
  constexpr bool clear_updated_flag = true;
  mesh_integrator_test.generateMesh(only_mesh_updated_blocks,
                                    clear_updated_flag);

  // Pubish mesh as vxblx msg
  voxblox_msgs::Mesh mesh_msg;
  vxb::generateVoxbloxMeshMsg(
      mesh_test, vxb::ColorMode::kLambertColor, &mesh_msg);
  mesh_msg.header.frame_id = world_frame_;
  mesh_pub_.publish(mesh_msg);

  // (Redundant) Publish mesh as PCL polygon mesh.
  pcl::PolygonMesh poly_mesh;
  vxb::toPCLPolygonMesh(*mesh_test, world_frame_, &poly_mesh);
  pcl_msgs::PolygonMesh pcl_msg_mesh;
  pcl_conversions::fromPCL(poly_mesh, pcl_msg_mesh);
  pcl_msg_mesh.header.stamp = ros::Time::now();
  pcl_msg_mesh.header.frame_id = world_frame_;
  polygon_mesh_pub_.publish(pcl_msg_mesh);

  // Get Semantic PCLs
  SemanticPointCloudMap semantic_pointclouds;
  // Use original colors to extract semantic labels.
  static const vxb::ColorMode& color_mode = vxb::ColorMode::kColor;
  getSemanticPointcloudsFromMesh(mesh_test, color_mode, &semantic_pointclouds);
  SemanticMeshMap semantic_meshes;
  getSemanticMeshesFromMesh(mesh_test, color_mode, &semantic_meshes);
  for (const auto& semantic_pcl_it : semantic_pointclouds) {
    const SemanticLabel& semantic_label = semantic_pcl_it.first;
    const SemanticPointCloud::Ptr& semantic_pcl = semantic_pcl_it.second;
    CHECK(semantic_meshes.find(semantic_label) != semantic_meshes.end());
    const vxb::Mesh::Ptr& semantic_mesh = semantic_meshes.at(semantic_label);
    if (semantic_pcl->empty()) {
      LOG(WARNING) << "Semantic pointcloud for label "
                   << std::to_string(semantic_label) << " is empty.";
      continue;
    }
    // Publish semantic pointcloud
    semantic_pcl_pubs_.publish(semantic_label, *semantic_pcl);

    // Publish semantic mesh
    visualization_msgs::Marker mesh_msg;
    vxb::fillMarkerWithMesh(
        *semantic_mesh, vxb::ColorMode::kLambertColor, &mesh_msg);
    mesh_msg.header.frame_id = world_frame_;
    semantic_mesh_pubs_.publish(semantic_label, mesh_msg);

    // Estimate objects in semantic pointcloud.
    // Only for those semantic labels that are not stuff (aka only for things).
    bool is_not_stuff =
        std::find(stuff_labels_.begin(), stuff_labels_.end(), semantic_label) ==
        stuff_labels_.end();
    if (is_not_stuff) {
      // LOG(INFO) << "Extracting objects for label: "
      //          << std::to_string(semantic_label);
      // CHECK(object_finder_);
      // Centroids centroids;
      // ObjectPointClouds object_pcls;
      //// TODO(Toni): do not publish right away the semantic pcl...
      // color_clustered_pcl_pub_.publish(
      //    object_finder_->findObjects(semantic_pcl, &centroids,
      //    &object_pcls));

      //// Create semantic instance for each centroid
      // InstanceId instance_id = 0;
      // for (size_t idx = 0; idx < centroids.size(); ++idx) {
      //  // Create SceneNode out of centroids
      //  SceneNode semantic_instance;
      //  semantic_instance.attributes_.semantic_label_ = semantic_label;
      //  semantic_instance.attributes_.color_ =
      //      semantic_pcl_it.second->at(0).getRGBVector3i();
      //  semantic_instance.attributes_.instance_id_ = instance_id;
      //  semantic_instance.id_ =
      //      std::to_string(semantic_label) + std::to_string(instance_id);
      //  ++instance_id;
      //  centroids.at(idx).get(semantic_instance.attributes_.position_);
      //  semantic_instance.attributes_.pcl_ = object_pcls.at(idx);

      //  // Add to database
      //  scene_graph_.addSceneNode(semantic_instance);
      //}
    } else {
      LOG(INFO) << "Skipping object extraction for semantic label: "
                << std::to_string(semantic_label);

      bool is_floor = std::find(floor_labels_.begin(),
                                floor_labels_.end(),
                                semantic_label) != floor_labels_.end();
      if (is_floor) {
        LOG(INFO) << "Finding room layout for label: "
                  << std::to_string(semantic_label);
        CHECK(room_finder_);
        // PlanarRegions planar_regions;
        Centroids room_centroids;
        // PointCloud::Ptr cloud_xyz(new PointCloud());
        // pcl::copyPointCloud(*semantic_pcl, *cloud_xyz);
        // room_finder_->findRooms(*semantic_mesh, &room_centroids);
        std::vector<IntensityPointCloud::Ptr> room_pcls;
        IntensityPointCloud::Ptr projected_cloud =
            room_finder_->findRooms(semantic_pcl, &room_centroids, &room_pcls);

        // Publish cloud
        projected_cloud->header.frame_id = world_frame_;
        color_clustered_pcl_pub_.publish(*projected_cloud);

        // Publish centroids
        // Create semantic instance for each centroid
        InstanceId instance_id = 0;
        for (size_t idx = 0; idx < room_centroids.size(); ++idx) {
          // Create SceneNode out of centroids
          SceneNode room_instance;
          static const Eigen::Vector3i kRoomColor(0u, 255u, 0u);
          room_instance.attributes_.semantic_label_ = kRoomSemanticLabel;
          room_instance.attributes_.color_ = kRoomColor;
          room_instance.attributes_.instance_id_ = instance_id;
          room_instance.id_ =
              std::to_string(semantic_label) + std::to_string(instance_id);
          ++instance_id;
          room_centroids.at(idx).get(room_instance.attributes_.position_);
          // We are copying to go from Intensity to RGB pointcloud...
          CHECK_LT(idx, room_pcls.size());
          CHECK(room_pcls.at(idx));
          room_instance.attributes_.pcl_.reset(new ColoredPointCloud);
          pcl::copyPointCloud(*(room_pcls.at(idx)),
                              *(room_instance.attributes_.pcl_));

          // Add to database
          scene_graph_.addSceneNode(room_instance);
        }

        LOG(INFO) << "Done finding room layout for label: "
                  << std::to_string(semantic_label);
      }

      bool is_walls = std::find(walls_labels_.begin(),
                                walls_labels_.end(),
                                semantic_label) != walls_labels_.end();
      if (is_walls) {
        LOG(WARNING) << "TODO add wall clustering.";
      }
    }
  }

  LOG(INFO) << "Visualize Scene Graph";
  scene_graph_.visualize();

  LOG(INFO) << "Visualize Human Pose Graphs";
  dynamic_scene_graph_.visualizePoseGraphs();

  LOG(INFO) << "Visualize Human Joints";
  dynamic_scene_graph_.visualizeJoints();
}

void SceneGraphSimulationServer::getSemanticPointcloudsFromMesh(
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

void SceneGraphSimulationServer::getSemanticMeshesFromMesh(
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

void SceneGraphSimulationServer::getPointcloudFromMesh(
    const vxb::MeshLayer::ConstPtr& mesh_layer,
    vxb::ColorMode color_mode,
    ColoredPointCloud::Ptr pointcloud) {
  CHECK_NOTNULL(pointcloud);
  pointcloud->clear();

  vxb::BlockIndexList mesh_indices;
  mesh_layer->getAllAllocatedMeshes(&mesh_indices);

  // Testing: only one centroid...
  std::vector<std::pair<vxb::Color, pcl::CentroidPoint<ColorPoint>>> centroids(
      1);
  centroids.at(0).first = vxb::Color::Red();
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
      const vxb::Color& instance_color = centroids.at(0).first;
      if (original_color.r == instance_color.r &&
          original_color.g == instance_color.g &&
          original_color.b == instance_color.b) {
        // Add to centroids
        centroids.at(0).second.add(point);
      }

      // Create and accumulate points
      pointcloud->push_back(point);
    }
  }

  // Create intermediate semantic instance
  SceneNode semantic_instance;
  semantic_instance.id_ = "x1";
  semantic_instance.attributes_.semantic_label_ = 0;
  centroids.at(0).second.get(semantic_instance.attributes_.position_);
  // Overwrites...
  scene_graph_.addSceneNode(semantic_instance);
}

void SceneGraphSimulationServer::rqtReconfigureCallback(
    RqtSceneGraphConfig& config,
    uint32_t level) {
  // Object Finder params
  object_finder_->updateClusterEstimator(
      static_cast<ObjectFinderType>(config.object_finder_type));

  EuclideanClusterEstimatorParams ec_params;
  ec_params.cluster_tolerance_ = config.cluster_tolerance;
  ec_params.max_cluster_size_ = config.ec_max_cluster_size;
  ec_params.min_cluster_size_ = config.ec_min_cluster_size;
  LOG(INFO) << ec_params.print();
  object_finder_->updateEuclideanClusterParams(ec_params);

  RegionGrowingClusterEstimatorParams rg_params;
  rg_params.curvature_threshold_ = config.curvature_threshold;
  rg_params.max_cluster_size_ = config.rg_max_cluster_size;
  rg_params.min_cluster_size_ = config.rg_min_cluster_size;
  rg_params.normal_estimator_neighbour_size_ =
      config.normal_estimator_neighbour_size;
  rg_params.number_of_neighbours_ = config.number_of_neighbours;
  rg_params.smoothness_threshold_ = config.smoothness_threshold;
  LOG(INFO) << rg_params.print();
  object_finder_->updateRegionGrowingParams(rg_params);

  // Room finder params
  MarchingCubesParams mc_params;
  mc_params.grid_res = config.grid_res;
  mc_params.iso_level_ = config.iso_level;
  mc_params.hoppe_or_rbf = config.hoppe_or_rbf;
  mc_params.extend_percentage = config.extend_percentage;
  mc_params.off_surface_displacement = config.off_surface_displacement;
  room_finder_->updateMarchingCubesParams(mc_params);

  OutlierFilterParams of_params;
  of_params.radius_search = config.radius_search;
  of_params.min_neighbors_in_radius = config.min_neighbors_in_radius;
  room_finder_->updateOutlierFilterParams(of_params);

  // LOG(WARNING) << "Clearing Scene Graph";
  // scene_graph_.clear();

  scene_graph_.updateEdgeAlpha(config.edge_alpha);
  scene_graph_.visualize();

  LOG(INFO) << "Object Finder params have been updated. "
               "Run scene graph reconstruction ros service to see the effects.";
}

}  // namespace kimera
