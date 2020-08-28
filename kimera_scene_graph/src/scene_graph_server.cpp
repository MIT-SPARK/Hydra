#include "kimera_scene_graph/scene_graph_server.h"

#include <voxblox_ros/mesh_pcl.h>
#include <voxblox_skeleton/io/skeleton_io.h>
#include <voxblox_skeleton/ros/skeleton_vis.h>
#include <voxblox_skeleton/skeleton_generator.h>

#include <pcl/common/copy_point.h>
#include <pcl/common/io.h>
#include <pcl/common/transforms.h>

#include "kimera_scene_graph/common.h"

#include "kimera_scene_graph/object_finder.h"
#include "kimera_scene_graph/places_room_connectivity_finder.h"
#include "kimera_scene_graph/room_connectivity_finder.h"
#include "kimera_scene_graph/room_finder.h"

#include "kimera_scene_graph/scene_graph.h"
#include "kimera_scene_graph/scene_graph_edge.h"
#include "kimera_scene_graph/scene_graph_layer.h"
#include "kimera_scene_graph/scene_graph_node.h"
#include "kimera_scene_graph/scene_graph_serialization.h"

#include "kimera_scene_graph/utils/kimera_to_voxblox.h"
#include "kimera_scene_graph/utils/voxblox_to_pcl.h"
#include "kimera_scene_graph/utils/voxblox_to_ros.h"

namespace kimera {

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
      edges_obj_skeleton_pub_(),
      world_frame_("world"),
      scene_graph_output_path_(""),
      semantic_pcl_pubs_("pcl", nh_private),
      semantic_mesh_pubs_("mesh", nh_private),
      semantic_mesh_2_pubs_("mesh_2", nh_private),
      rqt_server_(),
      rqt_callback_(),
      room_finder_(nullptr),
      places_in_rooms_finder_(nullptr),
      room_connectivity_finder_(nullptr),
      object_finder_(nullptr),
      enclosing_wall_finder_(nullptr),
      building_finder_(nullptr),
      scene_graph_(nullptr),
      scene_graph_visualizer_(nh, nh_private_),
      // dynamic_scene_graph_(nh, nh_private),
      semantic_config_(getSemanticTsdfIntegratorConfigFromRosParam(nh_private)),
      reconstruct_scene_graph_srv_() {
  // Create Scene graph
  scene_graph_ = std::make_shared<SceneGraph>();

  // TODO(Toni): remove
  skeleton_z_level_ = 2.0 * scene_graph_visualizer_.getLayerStepZ();

  // TODO(Toni): put all of this in scene graph reconstructor config!
  // Build object finder
  int object_finder_type = 0;
  nh_private.param(
      "object_finder_type", object_finder_type, object_finder_type);

  nh_private.param("world_frame", world_frame_, world_frame_);
  nh_private.param("scene_graph_output_path",
                   scene_graph_output_path_,
                   scene_graph_output_path_);
  nh_private.param("room_finder_esdf_slice_level",
                   room_finder_esdf_slice_level_,
                   room_finder_esdf_slice_level_);
  nh_private.param("tsdf_voxel_size", voxel_size_, voxel_size_);
  nh_private.param("tsdf_voxels_per_side", voxels_per_side_, voxels_per_side_);

  tsdf_layer_.reset(
      new vxb::Layer<vxb::TsdfVoxel>(voxel_size_, voxels_per_side_));
  esdf_layer_.reset(
      new vxb::Layer<vxb::EsdfVoxel>(voxel_size_, voxels_per_side_));

  esdf_integrator_.reset(new vxb::EsdfIntegrator(
      vxb::getEsdfIntegratorConfigFromRosParam(nh_private),
      tsdf_layer_.get(),
      esdf_layer_.get()));

  // Build finders
  enclosing_wall_finder_ =
      kimera::make_unique<EnclosingWallFinder>(world_frame_);
  object_finder_ = kimera::make_unique<ObjectFinder<ColorPoint>>(
      world_frame_, static_cast<ObjectFinderType>(object_finder_type));
  room_finder_ = kimera::make_unique<RoomFinder>(nh_private,
                                                 world_frame_,
                                                 room_finder_esdf_slice_level_,
                                                 skeleton_z_level_);
  places_in_rooms_finder_ = kimera::make_unique<PlacesRoomConnectivityFinder>(
      nh_private, skeleton_z_level_, world_frame_);
  room_connectivity_finder_ = kimera::make_unique<RoomConnectivityFinder>();
  building_finder_ = kimera::make_unique<BuildingFinder>();

  // Get labels of interesting things
  CHECK(nh_private.getParam("stuff_labels", stuff_labels_));
  CHECK(nh_private.getParam("walls_labels", walls_labels_));
  CHECK(nh_private.getParam("floor_labels", floor_labels_));

  // Pre-load map
  std::string file_path;
  LOG(INFO) << "Loading TSDF map:";
  CHECK(nh_private.getParam("load_tsdf_map", file_path));
  loadTsdfMap(file_path);
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
  polygon_mesh_pub_ = nh_private_.advertise<pcl_msgs::PolygonMesh>(
      "polygon_semantic_mesh", 1, true);
  edges_obj_skeleton_pub_ =
      nh_private_.advertise<visualization_msgs::MarkerArray>(
          "obj_skeleton_edges", 1, true);

  // This creates the places layer in the scene graph
  fillSceneGraphWithPlaces(sparse_skeleton_graph_);

  // Publish the skeleton.
  CHECK(scene_graph_);
  visualization_msgs::MarkerArray marker_array;
  vxb::SparseSkeletonGraph places_skeleton;
  utils::convertLayerToSkeleton(scene_graph_->getLayer(LayerId::kPlacesLayerId),
                                &places_skeleton);
  vxb::visualizeSkeletonGraph(places_skeleton, world_frame_, &marker_array);
  sparse_graph_pub_.publish(marker_array);

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

  // Build Object database action client
  LOG(INFO) << "Creating object database client.";
  object_db_client_ = kimera::make_unique<ObjectDBClient>("/object_db", true);
  LOG(INFO) << "Waiting for object database server.";
  object_db_client_->waitForServer();
  LOG(INFO) << "Object database server connected.";
}

bool SceneGraphBuilder::sceneGraphReconstructionServiceCall(
    std_srvs::SetBool::Request& request,
    std_srvs::SetBool::Response& response) {
  LOG(INFO) << "Requested scene graph reconstruction.";
  sceneGraphReconstruction(request.data);
  return true;
}

bool SceneGraphBuilder::fillSceneGraphWithPlaces(
    const vxb::SparseSkeletonGraph& sparse_skeleton) {
  CHECK(scene_graph_);
  static const NodeColor kPlaceColor = NodeColor(255u, 0u, 0u);
  std::vector<int64_t> vtx_ids;
  sparse_skeleton.getAllVertexIds(&vtx_ids);
  for (const auto& vtx_id : vtx_ids) {
    const vxb::SkeletonVertex& vtx = sparse_skeleton.getVertex(vtx_id);
    SceneGraphNode scene_graph_node;
    scene_graph_node.layer_id_ = LayerId::kPlacesLayerId;
    // Maybe we should use our own next_places_id_?
    scene_graph_node.node_id_ = vtx.vertex_id;
    // This will be filled automagically when adding the skeleton edges.
    // scene_graph_node.neighborhood_edge_map_
    scene_graph_node.attributes_.position_.x = vtx.point[0];
    scene_graph_node.attributes_.position_.y = vtx.point[1];
    scene_graph_node.attributes_.position_.z = vtx.point[2];
    scene_graph_node.attributes_.color_ = kPlaceColor;
    scene_graph_->addSceneNode(scene_graph_node);
  }
  CHECK(scene_graph_->hasLayer(LayerId::kPlacesLayerId));
  CHECK_EQ(scene_graph_->getLayer(LayerId::kPlacesLayerId).getNumberOfNodes(),
           vtx_ids.size())
      << "The skeleton and the scene graph layer should have the same "
         "number of nodes!";

  std::vector<int64_t> edge_ids;
  sparse_skeleton.getAllEdgeIds(&edge_ids);
  size_t invalid_edges = 0u;
  for (const auto& edge_id : edge_ids) {
    const vxb::SkeletonEdge& edge = sparse_skeleton.getEdge(edge_id);
    SceneGraphEdge scene_graph_edge;
    scene_graph_edge.start_layer_id_ = LayerId::kPlacesLayerId;
    scene_graph_edge.end_layer_id_ = LayerId::kPlacesLayerId;
    scene_graph_edge.start_node_id_ = edge.start_vertex;
    scene_graph_edge.end_node_id_ = edge.end_vertex;
    if (!scene_graph_edge.isSelfEdge()) {
      scene_graph_->addEdge(&scene_graph_edge);
    } else {
      LOG(ERROR) << "Not adding self-edge: " << scene_graph_edge.print();
      invalid_edges++;
    }
  }
  CHECK_EQ(scene_graph_->getLayer(LayerId::kPlacesLayerId).getNumberOfEdges(),
           edge_ids.size() - invalid_edges)
      << "The skeleton and the scene graph layer should have the same "
         "number of edges!";
}

bool SceneGraphBuilder::loadTsdfMap(const std::string& file_path) {
  constexpr bool kMulitpleLayerSupport = true;
  bool success = vxb::io::LoadBlocksFromFile(
      file_path,
      vxb::Layer<vxb::TsdfVoxel>::BlockMergingStrategy::kReplace,
      kMulitpleLayerSupport,
      tsdf_layer_.get());
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
    vxb::MeshLayer::Ptr mesh_layer) {
  vxb::MeshIntegratorConfig mesh_config;
  CHECK(tsdf_layer_);
  // TODO(Toni): mind that the parent server already builds the mesh (but that
  // is for the ground-truth sdf, not for the currently loaded one I believe.)
  vxb::MeshIntegrator<vxb::TsdfVoxel> mesh_integrator_test(
      mesh_config, tsdf_layer_.get(), mesh_layer.get());
  constexpr bool only_mesh_updated_blocks = false;
  constexpr bool clear_updated_flag = true;
  mesh_integrator_test.generateMesh(only_mesh_updated_blocks,
                                    clear_updated_flag);

  // Pubish mesh as vxblx msg
  voxblox_msgs::Mesh mesh_msg;
  vxb::generateVoxbloxMeshMsg(
      mesh_layer, vxb::ColorMode::kLambertColor, &mesh_msg);
  mesh_msg.header.frame_id = world_frame_;
  mesh_pub_.publish(mesh_msg);

  // (Redundant) Publish mesh as PCL polygon mesh.
  pcl::PolygonMesh poly_mesh;
  vxb::toPCLPolygonMesh(*mesh_layer, world_frame_, &poly_mesh);
  pcl_msgs::PolygonMesh pcl_msg_mesh;
  pcl_conversions::fromPCL(poly_mesh, pcl_msg_mesh);
  pcl_msg_mesh.header.stamp = ros::Time::now();
  pcl_msg_mesh.header.frame_id = world_frame_;
  polygon_mesh_pub_.publish(pcl_msg_mesh);
}

void SceneGraphBuilder::reconstructEsdfOutOfTsdf(const bool& save_to_file) {
  LOG(INFO) << "Building ESDF layer.";
  esdf_integrator_->setFullEuclidean(true);
  esdf_integrator_->updateFromTsdfLayerBatch();
  LOG(INFO) << "Saving ESDF layer.";
  if (save_to_file) {
    esdf_layer_->saveToFile("/home/tonirv/tesse_esdf.vxblx");
  }
  LOG(INFO) << "Done building ESDF layer.";
}

void SceneGraphBuilder::sceneGraphReconstruction(const bool& only_rooms) {
  // Also generate test mesh
  // TODO(Toni): this can be called from a rosservice now, so make sure
  // the sim world is ready before...
  // TODO(Toni): can't we load the mesh for now, this takes very long,
  // instead of re-building it ? The problem of doing so is that we might end
  // with inconsistent tsdf, esdf and mesh... Instead recompute all from tsdf...
  if (build_esdf_batch_) {
    reconstructEsdfOutOfTsdf(true);
  }
  // visualize();  // To visualize esdf and tsdf.

  vxb::Mesh::Ptr walls_mesh = nullptr;
  if (!only_rooms) {
    vxb::MeshLayer::Ptr mesh_test(
        new vxb::MeshLayer(tsdf_layer_->block_size()));
    reconstructMeshOutOfTsdf(mesh_test);

    // Get Semantic PCLs
    SemanticPointCloudMap semantic_pointclouds;
    // Use original colors to extract semantic labels.
    static const vxb::ColorMode& color_mode = vxb::ColorMode::kColor;
    getSemanticPointcloudsFromMesh(
        mesh_test, color_mode, &semantic_pointclouds);
    SemanticMeshMap semantic_meshes;
    getSemanticMeshesFromMesh(mesh_test, color_mode, &semantic_meshes);
    for (const auto& semantic_pcl_it : semantic_pointclouds) {
      const SemanticLabel& semantic_label = semantic_pcl_it.first;
      const SemanticPointCloud::Ptr& semantic_pcl = semantic_pcl_it.second;
      CHECK(semantic_pcl);
      CHECK(semantic_meshes.find(semantic_label) != semantic_meshes.end());
      if (semantic_pcl->empty()) {
        LOG(WARNING) << "Semantic pointcloud for label "
                     << std::to_string(semantic_label) << " is empty.";
        continue;
      }
      // Publish semantic pointcloud
      semantic_pcl_pubs_.publish(semantic_label, *semantic_pcl);

      // Publish semantic mesh
      vxb::Mesh::Ptr semantic_mesh = semantic_meshes.at(semantic_label);
      publishSemanticMesh(semantic_label, *semantic_mesh);

      // Estimate objects in semantic pointcloud.
      // Only for those semantic labels that are not stuff (aka only for
      // things).
      bool is_not_stuff = std::find(stuff_labels_.begin(),
                                    stuff_labels_.end(),
                                    semantic_label) == stuff_labels_.end();
      if (is_not_stuff) {
        // TODO(Toni): at some point we should use the 3D mesh, not the
        // pointcloud.
        // TODO(Toni):  REMOVE FOR FULL OBJECT DISCOVERY
        // static size_t n = 0;
        // if (n < 3u) {
        // Only extract objects that are not unknown.
        if (semantic_label == 2 || semantic_label == 7 || semantic_label == 5 ||
            semantic_label == 8) {
          extractThings(semantic_label, semantic_pcl);
        }
        //  ++n;
        //}
      } else {
        LOG(INFO) << "Skipping object extraction for semantic label: "
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
          walls_mesh = semantic_mesh;
        }
      }
    }
  }

  ////////////////////////// FIND ROOMS ////////////////////////////////////////
  CHECK(scene_graph_);
  LOG(INFO) << "Start Room finding.";
  CHECK(room_finder_);
  IntensityPointCloud::Ptr room_clusters_cloud =
      room_finder_->findRooms(*esdf_layer_, scene_graph_.get());
  CHECK(room_clusters_cloud);
  LOG(INFO) << "Finished Room finding.";

  // Publish cloud
  room_clusters_cloud->header.frame_id = world_frame_;
  color_clustered_pcl_pub_.publish(*room_clusters_cloud);

  //////////////// GRAPH INFERENCE after initial guess /////////////////////////
  LOG(INFO) << "Start Places Segmentation.";
  CHECK_NOTNULL(places_in_rooms_finder_);
  places_in_rooms_finder_->findPlacesRoomConnectivity(scene_graph_.get());
  LOG(INFO) << "Finished Places Segmentation.";

  // Now find the inter room connectivity
  LOG(INFO) << "Start Room Connectivity finder.";
  CHECK(room_connectivity_finder_);
  room_connectivity_finder_->findRoomConnectivity(scene_graph_.get());
  LOG(INFO) << "Finished Room Connectivity finder.";

  // Count the number of rooms to display stats
  countRooms();
  //////////////////////////////////////////////////////////////////////////////

  /////////////////////// Objects-Skeleton /////////////////////////////////////
  // Extract links between objects and skeleton, should be done in scene
  // graph...
  // TODO(TONI): update this function to use the objects/places layer a la
  // places_in_rooms_finder_
  // name it objects_in_places_finder_
  // publishSkeletonToObjectLinks(skeleton_graph_cloud);
  //////////////////////////////////////////////////////////////////////////////

  /////////////////////// Segment Walls Mesh ///////////////////////////////////
  if (walls_mesh) {
    LOG(INFO) << "Clustering walls...";
    vxb::Mesh segmented_walls_mesh;
    // TODO(TONI): this has to be rephrased using the 3D scene-graph!
    CHECK(scene_graph_);
    enclosing_wall_finder_->findWalls(
        *walls_mesh, *scene_graph_, &segmented_walls_mesh);
    // 19 overrides the old walls, use another one!
    publishSemanticMesh(19, segmented_walls_mesh);
    // Publish exploded semantic walls
    publishExplodedWalls(segmented_walls_mesh);
    LOG(INFO) << "Done clustering walls...";
  } else {
    LOG(WARNING) << "Not segmenting walls mesh bcs no walls mesh found.";
  }
  //////////////////////////////////////////////////////////////////////////////

  ///////////////////////////////// BUILDINGS //////////////////////////////////
  CHECK(scene_graph_);
  building_finder_->findBuildings(scene_graph_.get());
  //////////////////////////////////////////////////////////////////////////////

  LOG(INFO) << "Visualizing Scene Graph";
  CHECK(scene_graph_);
  scene_graph_visualizer_.visualize(*scene_graph_);

  LOG(INFO) << "Visualizing Human Pose Graphs";
  // dynamic_scene_graph_.visualizePoseGraphs();

  LOG(INFO) << "Visualizing Human Skeletons";
  // dynamic_scene_graph_.visualizeJoints();

  if (!scene_graph_output_path_.empty()) {
    LOG(INFO) << "Saving Scene-Graph to file: "
              << scene_graph_output_path_.c_str();
    save(*scene_graph_, scene_graph_output_path_);
    LOG(INFO) << "Done saving Scene-Graph to file";
  }
}

void SceneGraphBuilder::countRooms() const {
  LOG(INFO) << "COUNTING ROOMS NODES";
  SceneGraphLayer rooms_layer(LayerId::kRoomsLayerId);
  CHECK(scene_graph_);
  CHECK(scene_graph_->getLayerSafe(LayerId::kRoomsLayerId, &rooms_layer));
  LOG(INFO) << "There are: " << rooms_layer.getNumberOfNodes();

  // Print room id and number of places
  // for (const auto& kv : room_count) {
  //   LOG(INFO) << "ROOM ID: " << kv.first << " with N vertices: " <<
  //   kv.second;
  // }
}

void SceneGraphBuilder::publishSkeletonToObjectLinks(
    const ColorPointCloud::Ptr& skeleton_graph_pcl) {
  CHECK(scene_graph_);
  // TODO(TONI): bad, you should add the skeleton to the scene graph instead...
  // In the meantime:

  pcl::KdTreeFLANN<Point> kdtree;
  CHECK(skeleton_graph_pcl);
  PointCloud::Ptr skeleton_graph_pcl_converted(new PointCloud);
  pcl::copyPointCloud(*skeleton_graph_pcl, *skeleton_graph_pcl_converted);
  kdtree.setInputCloud(skeleton_graph_pcl_converted);

  // 1. Loop over the objects in the database_
  std::vector<SceneGraphNode> scene_nodes;
  scene_graph_->getAllSceneNodes(&scene_nodes);
  visualization_msgs::MarkerArray obj_to_skeleton_markers;
  for (const SceneGraphNode& node : scene_nodes) {
    const NodeAttributes& attributes = node.attributes_;
    const SemanticLabel& semantic_label = attributes.semantic_label_;
    if (node.layer_id_ == LayerId::kObjectsLayerId) {
      // We have an object, get its centroid.
      NodePosition centroid = attributes.position_;
      pcl::PointXYZ centroid_point(centroid.x, centroid.y, centroid.z);

      // Compute nearest skeleton vertex
      static constexpr int K = 1;
      std::vector<int> nn_indices(K);
      std::vector<float> nn_squared_distances(K);
      if (kdtree.nearestKSearch(
              centroid_point, K, nn_indices, nn_squared_distances) > 0) {
        // Found the nearest neighbor
        CHECK_GT(nn_indices.size(), 0);
        CHECK_GT(nn_squared_distances.size(), 0);
        CHECK_EQ(nn_indices.size(), nn_squared_distances.size());
        Point pcl_point = skeleton_graph_pcl_converted->at(nn_indices.at(0));
        // Pcl points leave in a diff layer than skeleton, separate them!
        // PCL points live at the first layer
        centroid.z += scene_graph_visualizer_.getLayerStepZ();
        // Skeleton points live at the second layer;
        pcl_point.z += skeleton_z_level_;
        visualization_msgs::Marker marker =
            scene_graph_visualizer_.getLineFromPointToPoint(
                NodePosition(pcl_point.x, pcl_point.y, pcl_point.z),
                centroid,
                attributes.color_,
                0.04,
                std::to_string(semantic_label));
        obj_to_skeleton_markers.markers.push_back(marker);
      } else {
        LOG(WARNING) << "Didn't find NN! Not linking this centroid...";
      }
    }
  }

  edges_obj_skeleton_pub_.publish(obj_to_skeleton_markers);
}

void SceneGraphBuilder::publishSemanticMesh(const SemanticLabel& semantic_label,
                                            const vxb::Mesh& semantic_mesh) {
  visualization_msgs::Marker mesh_msg;
  mesh_msg.header.frame_id = world_frame_;
  bool is_walls =
      std::find(walls_labels_.begin(), walls_labels_.end(), semantic_label) !=
      walls_labels_.end();
  // Publish twice: one at the mesh level, the other at the object level.
  // Except for walls, those should be at skeleton level.
  CHECK(scene_graph_);
  utils::fillMarkerWithMesh(
      semantic_mesh,
      vxb::ColorMode::kLambertColor,
      &mesh_msg,
      is_walls ? 10 : scene_graph_visualizer_.getLayerStepZ());
  semantic_mesh_2_pubs_.publish(semantic_label, mesh_msg);
  visualization_msgs::Marker new_mesh_msg;
  new_mesh_msg.header.frame_id = world_frame_;
  utils::fillMarkerWithMesh(
      semantic_mesh, vxb::ColorMode::kLambertColor, &new_mesh_msg, 0.0);
  semantic_mesh_pubs_.publish(semantic_label, new_mesh_msg);
}

void SceneGraphBuilder::publishExplodedWalls(const vxb::Mesh& segmented_walls,
                                             const double& explosion_factor) {
  // TODO
}

void SceneGraphBuilder::extractThings(
    const SemanticLabel& semantic_label,
    const SemanticPointCloud::Ptr& semantic_pcl) {
  CHECK(scene_graph_);
  LOG(INFO) << "Extracting objects for label: "
            << std::to_string(semantic_label);
  CHECK(object_finder_);
  CHECK(semantic_pcl);
  if (semantic_pcl->empty()) {
    LOG(WARNING) << "Extracting things for semantic label "
                 << std::to_string(semantic_label)
                 << " failed because the semantic pointcloud is empty.";
    return;
  }

  Centroids centroids;
  ObjectPointClouds object_pcls;
  // TODO(Toni): do not publish right away the semantic pcl...
  ObjectFinder<ColorPoint>::BoundingBoxes bounding_boxes;
  color_clustered_pcl_pub_.publish(object_finder_->findObjects(
      semantic_pcl, &centroids, &object_pcls, &bounding_boxes));

  // Call object registration server to get registrated point clouds
  ObjectPointClouds registrated_object_pcls =
      objectDatabaseActionCall(object_pcls, std::to_string(semantic_label));

  const auto& n_centroids = centroids.size();
  CHECK_EQ(n_centroids, object_pcls.size());
  CHECK_EQ(n_centroids, bounding_boxes.size());

  // Create semantic instance for each centroid
  NodeId object_instance_id = 0;
  for (size_t idx = 0u; idx < n_centroids; ++idx) {
    // Create SceneNode out of centroids
    SceneGraphNode scene_node;
    NodeAttributes& attributes = scene_node.attributes_;
    attributes.semantic_label_ = semantic_label;
    const auto& color =
        semantic_config_.semantic_label_to_color_->getColorFromSemanticLabel(
            semantic_label);
    attributes.color_ = NodeColor(color.r, color.g, color.b);
    attributes.name_ =
        std::to_string(semantic_label) + std::to_string(object_instance_id);
    scene_node.node_id_ = next_object_id_;
    scene_node.layer_id_ = LayerId::kObjectsLayerId;
    pcl::PointXYZ centroid_point;
    centroids.at(idx).get(centroid_point);  // Calculates centroid
    attributes.position_ =
        NodePosition(centroid_point.x, centroid_point.y, centroid_point.z);
    attributes.pcl_ = object_pcls.at(idx);
    CHECK(attributes.pcl_);
    attributes.bounding_box_ = bounding_boxes.at(idx);

    // Add to database
    CHECK(scene_node.attributes_.pcl_) << "Pcl not initialized!";
    CHECK(scene_node.layer_id_ == LayerId::kObjectsLayerId);
    scene_graph_->addSceneNode(scene_node);
    ++object_instance_id;
    ++next_object_id_;
  }
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

  CHECK(scene_graph_);
  // LOG(WARNING) << "Clearing Scene Graph";
  // scene_graph_->clear();

  scene_graph_visualizer_.updateEdgeAlpha(config.edge_alpha);
  scene_graph_visualizer_.visualize(*scene_graph_);

  LOG(INFO) << "Object Finder params have been updated. "
               "Run scene graph reconstruction ros service to see the effects.";
}

/**
 * Function to handle registration with object db
 * @param object_pcls
 * @param semantic_label
 */
ObjectPointClouds SceneGraphBuilder::objectDatabaseActionCall(
    const ObjectPointClouds& object_pcls,
    const std::string semantic_label) {
  LOG(INFO) << "Sending object point clouds to object database.";
  // For storing all registrated object point clouds
  ObjectPointClouds registrated_object_pcls;

  // Query object database for all point clouds
  // Object pcls is a vector of pointers to pcl::XYZRGB point clouds
  for (const auto& color_pcl : object_pcls) {
    if (color_pcl->points.size() == 0) {
      LOG(INFO) << "Empty point cloud given to object database client.";
      registrated_object_pcls.push_back(color_pcl);
      continue;
    }
    LOG(INFO) << "Object pcl size:" << color_pcl->points.size();

    // Create goal for object db
    object_db::ObjectRegistrationGoal c_goal;
    c_goal.semantic_label = semantic_label;

    // Get current color
    const auto& color_r = color_pcl->points[0].r;
    const auto& color_g = color_pcl->points[0].g;
    const auto& color_b = color_pcl->points[0].b;

    // Convert colored point cloud to sensor msg point cloud
    for (size_t p_idx = 0; p_idx < color_pcl->size(); ++p_idx) {
      geometry_msgs::Point32 c_point;
      c_point.x = color_pcl->points[p_idx].x;
      c_point.y = color_pcl->points[p_idx].y;
      c_point.z = color_pcl->points[p_idx].z;
      c_goal.dst.points.push_back(c_point);
    }

    // Send goal to object databse action server
    object_db_client_->sendGoal(c_goal);

    // Deal with the result
    bool finished = object_db_client_->waitForResult(ros::Duration(30));
    bool aborted = object_db_client_->getState() ==
                   actionlib::SimpleClientGoalState::ABORTED;
    if (aborted) {
      LOG(INFO) << "Object database aborted.";
      registrated_object_pcls.push_back(color_pcl);
    } else if (!finished) {
      LOG(INFO) << "Object database did not finish before the timeout.";
      registrated_object_pcls.push_back(color_pcl);
    } else {
      auto result = object_db_client_->getResult();
      auto registrated_object = result->aligned_object;

      // Convert sensor msg point cloud type to colored pcl
      ColorPointCloud::Ptr registrated_pcl(new ColorPointCloud);
      for (const auto& o_point : registrated_object.points) {
        ColorPoint c_point;
        c_point.x = o_point.x;
        c_point.y = o_point.y;
        c_point.z = o_point.z;
        c_point.r = color_r;
        c_point.g = color_g;
        c_point.b = color_b;
        registrated_pcl->points.push_back(c_point);
      }
      registrated_object_pcls.push_back(registrated_pcl);
      LOG(INFO) << "Object database query successful.";
      LOG(INFO) << "Registrated object size: " << registrated_pcl->size();
    }
  }

  if (registrated_object_pcls.size() != object_pcls.size()) {
    LOG(INFO) << "Registrated objects size mismatch!";
  }
  return registrated_object_pcls;
}

}  // namespace kimera
