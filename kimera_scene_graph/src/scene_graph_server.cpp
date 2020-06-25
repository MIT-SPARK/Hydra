#include "kimera_scene_graph/scene_graph_server.h"

#include <voxblox_ros/mesh_pcl.h>
#include <voxblox_skeleton/io/skeleton_io.h>
#include <voxblox_skeleton/ros/skeleton_vis.h>
#include <voxblox_skeleton/skeleton_generator.h>

#include <pcl/common/copy_point.h>

#include <pcl/common/io.h>

#include <pcl/common/transforms.h>
#include <pcl/filters/crop_hull.h>
#include <pcl/surface/concave_hull.h>

#include "kimera_scene_graph/common.h"
#include "kimera_scene_graph/utils/voxblox_to_pcl.h"

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
      semantic_mesh_2_pubs_("mesh_2", nh_private),
      rqt_server_(),
      rqt_callback_(),
      room_finder_(nullptr),
      object_finder_(nullptr),
      wall_finder_(nullptr),
      enclosing_wall_finder_(nullptr),
      building_finder_(nullptr),
      scene_graph_(nh, nh_private),
      reconstruct_scene_graph_srv_(),
      load_map_srv_() {
  // TODO(Toni): remove
  skeleton_z_level_ = 2.0 * scene_graph_.getLayerStepZ();

  // TODO(Toni): put all of this in scene graph reconstructor config!
  // Build object finder
  int object_finder_type = 0;
  nh_private.param(
      "object_finder_type", object_finder_type, object_finder_type);

  // Build finders
  wall_finder_ = kimera::make_unique<WallFinder<ColorPoint>>(world_frame_);
  enclosing_wall_finder_ =
      kimera::make_unique<EnclosingWallFinder>(world_frame_);
  object_finder_ = kimera::make_unique<ObjectFinder<ColorPoint>>(
      world_frame_, static_cast<ObjectFinderType>(object_finder_type));
  room_finder_ = kimera::make_unique<RoomFinder>(nh_private, world_frame_);
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

  // ROS publishers.
  sparse_graph_pub_ = nh_private_.advertise<visualization_msgs::MarkerArray>(
      "sparse_graph", 1, true);
  segmented_sparse_graph_pub_ =
      nh_private_.advertise<visualization_msgs::MarkerArray>(
          "segmented_sparse_graph", 1, true);
  color_clustered_pcl_pub_ =
      nh_private_.advertise<IntensityPointCloud>("clustered_pcls", 1, true);
  walls_clustered_pcl_pub_ = nh_private_.advertise<IntensityPointCloud>(
      "walls_clustered_pcls", 1, true);
  esdf_truncated_pub_ =
      nh_private_.advertise<IntensityPointCloud>("esdf_truncated", 1, true);
  room_centroids_pub_ =
      nh_private_.advertise<ColorPointCloud>("room_centroids", 1, true);
  room_layout_pub_ =
      nh_private_.advertise<ColorPointCloud>("room_layout", 1, true);
  mesh_pub_ =
      nh_private_.advertise<voxblox_msgs::Mesh>("semantic_mesh", 1, true);
  polygon_mesh_pub_ = nh_private_.advertise<pcl_msgs::PolygonMesh>(
      "polygon_semantic_mesh", 1, true);
  edges_obj_skeleton_pub_ =
      nh_private_.advertise<visualization_msgs::MarkerArray>(
          "obj_skeleton_edges", 1, true);

  // Publish the skeleton.
  visualization_msgs::MarkerArray marker_array;
  vxb::visualizeSkeletonGraph(
      sparse_skeleton_graph_, world_frame_, &marker_array);
  sparse_graph_pub_.publish(marker_array);

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

  // Build Object database action client
  LOG(INFO) << "Creating object database client.";
  object_db_client_ = kimera::make_unique<ObjectDBClient>("object_db", true);
  LOG(INFO) << "Waiting for object database server.";
  object_db_client_->waitForServer();
  LOG(INFO) << "Object database server connected.";
}

bool SceneGraphSimulationServer::sceneGraphReconstructionServiceCall(
    std_srvs::SetBool::Request& request,
    std_srvs::SetBool::Response& response) {
  LOG(INFO) << "Requested scene graph reconstruction.";
  sceneGraphReconstruction(request.data);
  return true;
}

void SceneGraphSimulationServer::reconstructMeshOutOfTsdf(
    vxb::MeshLayer::Ptr mesh_layer) {
  vxb::MeshIntegratorConfig mesh_config;
  CHECK(tsdf_test_);
  // TODO(Toni): mind that the parent server already builds the mesh (but that
  // is for the ground-truth sdf, not for the currently loaded one I believe.)
  vxb::MeshIntegrator<vxb::TsdfVoxel> mesh_integrator_test(
      mesh_config, tsdf_test_.get(), mesh_layer.get());
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

void SceneGraphSimulationServer::reconstructEsdfOutOfTsdf(
    const bool& save_to_file) {
  LOG(INFO) << "Building ESDF layer.";
  esdf_integrator_->setFullEuclidean(true);
  esdf_integrator_->updateFromTsdfLayerBatch();
  LOG(INFO) << "Saving ESDF layer.";
  if (save_to_file) {
    esdf_test_->saveToFile("/home/tonirv/tesse_esdf.vxblx");
  }
  LOG(INFO) << "Done building ESDF layer.";
}

void SceneGraphSimulationServer::sceneGraphReconstruction(
    const bool& only_rooms) {
  // Also generate test mesh
  // TODO(Toni): this can be called from a rosservice now, so make sure
  // the sim world is ready before...
  // TODO(Toni): can't we load the mesh for now, this takes very long,
  // instead of re-building it ? The problem of doing so is that we might end
  // with inconsistent tsdf, esdf and mesh... Instead recompute all from tsdf...
  // Reconstruct ESDF layer (TODO:Toni) save to file and reload...
  if (build_esdf_batch_) {
    reconstructEsdfOutOfTsdf(true);
  }
  // visualize();  // To visualize esdf and tsdf.

  vxb::Mesh::Ptr walls_mesh = nullptr;
  if (!only_rooms) {
    vxb::MeshLayer::Ptr mesh_test(new vxb::MeshLayer(tsdf_test_->block_size()));
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
          static constexpr bool kUseEnclosingWallFinder = true;
          if (!kUseEnclosingWallFinder) {
            LOG(WARNING) << "Clustering walls...";
            Centroids wall_centroids;
            std::vector<ColorPointCloud::Ptr> wall_pcls;
            ObjectFinder<ColorPoint>::BoundingBoxes wall_bounding_boxes;
            walls_clustered_pcl_pub_.publish(
                wall_finder_->findWalls(semantic_pcl,
                                        &wall_centroids,
                                        &wall_pcls,
                                        &wall_bounding_boxes));
            LOG(WARNING) << "Done clustering walls...";
          } else {
            walls_mesh = semantic_mesh;
          }
        }
      }
    }
  }

  ////////////////////////// FIND ROOMS ////////////////////////////////////////
  LOG(INFO) << "Finding room layout.";
  CHECK(room_finder_);
  Centroids room_centroids;
  std::vector<ColorPointCloud::Ptr> room_pcls;
  IntensityPointCloud::Ptr esdf_pcl(new IntensityPointCloud);
  createDistancePointcloudFromEsdfLayerSlice(
      *esdf_test_, 2, visualization_slice_level_, &*esdf_pcl);
  IntensityPointCloud::Ptr room_clusters_cloud =
      room_finder_->findRooms(esdf_pcl, &room_centroids, &room_pcls);

  // Publish cloud
  room_clusters_cloud->header.frame_id = world_frame_;
  color_clustered_pcl_pub_.publish(*room_clusters_cloud);

  // Publish truncated ESDF to see wall layout:
  IntensityPointCloud::Ptr esdf_truncated(new IntensityPointCloud);
  esdf_truncated = passThroughFilter1D<IntensityPoint>(
      esdf_pcl, "intensity", -1.0, kEsdfTruncation);
  esdf_truncated->header.frame_id = world_frame_;
  IntensityPointCloud::Ptr esdf_truncated_z_shift(new IntensityPointCloud);
  Eigen::Affine3f z_esdf = Eigen::Affine3f::Identity();
  z_esdf.translation() << 0.0, 0.0, skeleton_z_level_;
  pcl::transformPointCloud(*esdf_truncated, *esdf_truncated_z_shift, z_esdf);
  esdf_truncated_pub_.publish(*esdf_truncated_z_shift);

  //////////////// GRAPH INFERENCE after initial guess /////////////////////////
  // Show the graph before any inference
  visualization_msgs::MarkerArray marker_array;
  vxb::visualizeSkeletonGraph(sparse_skeleton_graph_,
                              world_frame_,
                              &marker_array,
                              vxb::SkeletonGraphVizType::kRoomId,
                              skeleton_z_level_);
  segmented_sparse_graph_pub_.publish(marker_array);

  // Create cloud of the graph, make sure indices are 1-to-1
  std::vector<int64_t> vertex_ids;
  std::map<int, int64_t> cloud_to_graph_ids;
  ColorPointCloud::Ptr skeleton_graph_cloud = createPclCloudFromSkeleton(
      sparse_skeleton_graph_, &cloud_to_graph_ids, &vertex_ids);

  // Publish centroids
  // Create semantic instance for each centroid
  InstanceId instance_id = 1;  // do not init to 0 (special null id).
  ColorPointCloud::Ptr room_layout_pcl(new ColorPointCloud);
  for (size_t idx = 0; idx < room_centroids.size(); ++idx) {
    // Create SceneNode out of centroids
    SceneNode room_instance;
    room_instance.attributes_.semantic_label_ = kRoomSemanticLabel;
    room_instance.attributes_.color_ = kRoomColor;
    room_instance.attributes_.instance_id_ = instance_id;
    room_instance.id_ =
        std::to_string(kRoomSemanticLabel) + std::to_string(instance_id);
    room_centroids.at(idx).get(room_instance.attributes_.position_);
    CHECK_LT(idx, room_pcls.size());

    // Associate room to sparse graph...
    // Get section of the sparse graph that falls inside this room_pcl
    const auto& room_pcl = room_pcls.at(idx);
    CHECK(room_pcl);

    // Find concave hull of pcl first
    std::vector<pcl::Vertices> concave_polygon;
    pcl::ConcaveHull<ColorPoint> concave_hull_adapter;
    static constexpr double kAlpha = 0.30;
    concave_hull_adapter.setAlpha(kAlpha);
    concave_hull_adapter.setInputCloud(room_pcl);
    concave_hull_adapter.setKeepInformation(true);
    concave_hull_adapter.setDimension(2);
    ColorPointCloud::Ptr concave_hull_pcl(new ColorPointCloud);
    concave_hull_adapter.reconstruct(*concave_hull_pcl, concave_polygon);

    pcl::KdTreeFLANN<ColorPoint> kdtree;
    kdtree.setInputCloud(concave_hull_pcl);
    static constexpr int K = 1;  // find nearest-neighbor
    std::vector<int> nn_indices(K);
    std::vector<float> nn_squared_distances(K);

    // Don't show concave hull in pcl_, too much clutter with edges.
    // room_instance.attributes_.pcl_ = concave_hull_pcl;
    room_instance.attributes_.pcl_.reset(new ColorPointCloud);
    *room_layout_pcl =
        *room_layout_pcl + *concave_hull_pcl;  // concatenate the clouds.

    // Create cropper to know which vertices are outside the room
    pcl::CropHull<ColorPoint> hull_cropper;
    hull_cropper.setHullIndices(concave_polygon);
    hull_cropper.setHullCloud(concave_hull_pcl);
    hull_cropper.setDim(2);
    // because we can only get removed indices, tell the algorithm to
    // remove inside.
    hull_cropper.setCropOutside(true);

    hull_cropper.setInputCloud(skeleton_graph_cloud);
    std::vector<int> removed_indices;
    hull_cropper.filter(removed_indices);

    // Loop over graph and associate room to the removed vertices
    // and label edges according to their type.
    std::vector<int64_t> edge_ids;
    for (const int& removed_index : removed_indices) {
      CHECK_LE(removed_index, cloud_to_graph_ids.size());
      vxb::SkeletonVertex& vertex =
          sparse_skeleton_graph_.getVertex(cloud_to_graph_ids[removed_index]);
      // Identify this vertex as belonging to the room
      if (vertex.room_id != 0) {
        // This vertex is already labeled!
        LOG(WARNING) << "Vertex already labeled when finding rooms.";
        continue;
      }
      vertex.room_id = instance_id;
      ColorPoint point;
      point.x = vertex.point.x();
      point.y = vertex.point.y();
      point.z = vertex.point.z();
      const vxb::Color& color = getRoomColor(vertex.room_id);
      point.r = color.r;
      point.g = color.g;
      point.b = color.b;
      // Just append new points to the room (correspond to vertices of
      // sparse graph.
      CHECK(room_instance.attributes_.pcl_);
      room_instance.attributes_.color_ << color.r, color.g, color.b;
      room_instance.attributes_.pcl_->push_back(point);
      edge_ids.insert(
          edge_ids.end(), vertex.edge_list.begin(), vertex.edge_list.end());
    }

    // "Dilate" the labels, for every null vertex, if one of its neighbors
    // is a room and they are closer than esdf truncation distance,
    // change its id to room x.
    // Label edges according to whether they are transition or not
    // FLOODFILL LIKE CRAZY
    while (edge_ids.size() > 0) {
      // Depth first.
      auto edge_idx = edge_ids.back();  // copy (bcs we delete next)
      edge_ids.pop_back();
      vxb::SkeletonEdge& edge = sparse_skeleton_graph_.getEdge(edge_idx);
      vxb::SkeletonVertex& vtx_start =
          sparse_skeleton_graph_.getVertex(edge.start_vertex);
      vxb::SkeletonVertex& vtx_end =
          sparse_skeleton_graph_.getVertex(edge.end_vertex);
      bool is_start_in_room = false;
      if (vtx_start.room_id == instance_id) {
        is_start_in_room = true;
      } else if (vtx_end.room_id == instance_id) {
        is_start_in_room = false;
      } else {
        LOG(ERROR) << "At least one vertex of the edge should belong to this "
                      "room.";
        continue;
      }
      if (vtx_start.room_id == vtx_end.room_id) {
        // We have a self-contained edge, label the edge as non-transition
        edge.transition_edge = false;
      } else {
        // We have a transition edge, from one room to unknown.
        // Search for its nearest neighbor to the current pcl of the room
        // do the serach in 2D, because the truncation of the esdf is done
        // in 2D...
        ColorPoint search_point;
        if (is_start_in_room) {
          // If start is in room, we are trying to label the end.
          search_point.x = vtx_end.point.x();
          search_point.y = vtx_end.point.y();
          search_point.z = 0.0;
        } else {
          search_point.x = vtx_start.point.x();
          search_point.y = vtx_start.point.y();
          search_point.z = 0.0;
        }

        float esdf_distance_approx = 0;
        if (kdtree.nearestKSearch(
                search_point, K, nn_indices, nn_squared_distances) > 0) {
          // Found the nearest neighbor
          CHECK_GT(nn_indices.size(), 0);
          CHECK_GT(nn_squared_distances.size(), 0);
          CHECK_EQ(nn_indices.size(), nn_squared_distances.size());
          // Don't take the squared distances, since we need to project
          // in 2D!
          const ColorPoint& pcl_point = concave_hull_pcl->at(nn_indices.at(0));
          esdf_distance_approx =
              std::sqrt(std::pow(search_point.x - pcl_point.x, 2) +
                        std::pow(search_point.y - pcl_point.y, 2));
        } else {
          LOG(WARNING) << "Didn't find NN! Not labeling this vertex...";
          continue;
        }

        float kEpsilon = 0.10 * kEsdfTruncation;  // allow for 10% extra.
        if (esdf_distance_approx < kEsdfTruncation + kEpsilon) {
          // The node is inside the esdf truncation, so label the
          // unknown vertex as in the room and add its neighbors to the
          // queue.
          if (is_start_in_room) {
            vtx_end.room_id = instance_id;
            edge_ids.insert(edge_ids.end(),
                            vtx_end.edge_list.begin(),
                            vtx_end.edge_list.end());
          } else {
            vtx_start.room_id = instance_id;
            edge_ids.insert(edge_ids.end(),
                            vtx_start.edge_list.begin(),
                            vtx_start.edge_list.end());
          }
          CHECK_EQ(vtx_end.room_id, vtx_start.room_id);
          edge.transition_edge = false;
        } else {
          // That is indeed a transition edge!
          // Store what kind of transition
          // If we transit to NULL, that is perhaps a corridor or door...
          edge.transition_edge = true;
        }
      }
    }

    // Visualize new skeleton for each room.
    visualization_msgs::MarkerArray marker_array;
    vxb::visualizeSkeletonGraph(sparse_skeleton_graph_,
                                world_frame_,
                                &marker_array,
                                vxb::SkeletonGraphVizType::kRoomId,
                                skeleton_z_level_);
    segmented_sparse_graph_pub_.publish(marker_array);

    // Add the room to the database
    scene_graph_.addSceneNode(room_instance);
    instance_id++;
  }
  Eigen::Affine3f z_shift = Eigen::Affine3f::Identity();
  z_shift.translation() << 0.0, 0.0, 10.0;
  room_layout_pcl->header.frame_id = world_frame_;
  ColorPointCloud::Ptr transformed_pcl(new ColorPointCloud);
  pcl::transformPointCloud(*room_layout_pcl, *transformed_pcl, z_shift);
  room_layout_pub_.publish(*transformed_pcl);

  // Finally, do majority voting on the unknown folks...
  // Get all vertices that do not know in which room they are.
  std::list<int64_t> undecided_vertices_ids;
  for (const auto& vertex_id : vertex_ids) {
    undecided_vertices_ids.push_back(vertex_id);
  }
  std::unordered_map<int64_t, int> loopy_vertex;
  while (undecided_vertices_ids.size() > 0) {
    // Breadth first.
    int64_t vertex_id = undecided_vertices_ids.front();  // copy
    undecided_vertices_ids.pop_front();
    vxb::SkeletonVertex& vertex = sparse_skeleton_graph_.getVertex(vertex_id);
    std::map<int, int64_t> room_votes;
    if (vertex.room_id == 0) {
      // Bad one, it has no clue where it is.

      // Check that it has neigbors
      if (vertex.edge_list.size() == 0) {
        // This folk has no neighbors :O
        LOG(WARNING) << "Graph vertex with id " << vertex_id
                     << " has no neighbors!";
        continue;
      }

      // Ask its neighbors where it is!
      for (const int64_t& edge_id : vertex.edge_list) {
        const vxb::SkeletonEdge& edge = sparse_skeleton_graph_.getEdge(edge_id);

        // Remove self-edges
        if (edge.end_vertex == vertex_id && edge.start_vertex == vertex_id) {
          LOG(ERROR) << "Self-edges are not allowed! Removing edge.\n"
                     << "Offending edge id: " << edge.edge_id << '\n'
                     << "Vertex id: " << vertex_id << '\n'
                     << "Start vertex id: " << edge.start_vertex << '\n'
                     << "End vertex id: " << edge.end_vertex;
          // Remove from graph, at this point edge is a dangling ref!
          sparse_skeleton_graph_.removeEdge(edge_id);
          continue;
        }

        // Get neighbor voxel for this edge
        vxb::SkeletonVertex* neighbor_vertex = nullptr;
        if (edge.end_vertex != vertex_id) {
          // Hello nice neighbor!
          neighbor_vertex = &sparse_skeleton_graph_.getVertex(edge.end_vertex);
        } else if (edge.start_vertex != vertex_id) {
          // Oh the end_vertex was the current vertex, hi actual
          // neighbor!
          neighbor_vertex =
              &sparse_skeleton_graph_.getVertex(edge.start_vertex);
        } else {
          LOG(ERROR) << "The edge does not belong to this vtx! "
                     << "This shouldn't happen.\n"
                     << "Vertex id: " << vertex_id << '\n'
                     << "Start vertex id: " << edge.start_vertex << '\n'
                     << "End vertex id: " << edge.end_vertex;
          // Forget about this edge for now... but this should not
          // happen!
          continue;
        }
        CHECK(neighbor_vertex);

        // So, neighbor, tell me, in which room are you?
        auto it = room_votes.find(neighbor_vertex->room_id);
        if (it == room_votes.end()) {
          // First time we see this room id.
          room_votes[neighbor_vertex->room_id] = 1;
        } else {
          // Ok, keep adding the votes for this room.
          it->second += 1;
        }
      }

      // Find max room id.
      int max_room_id = 0;
      int64_t max_room_count = 0;
      for (const auto& kv : room_votes) {
        if (kv.first != 0) {
          // Only count those neighbors that know where they are
          if (kv.second > max_room_count) {
            max_room_id = kv.first;
            max_room_count = kv.second;
          }
        }
      }

      if (max_room_id == 0) {
        // Wow, none of our neighbors knows where it is!
        // (do not think it has no neighbors, we check this above).

        // Keep track of the id of this guy, to avoid infinite loops
        // which may happen e.g. if a subgraph of two vertices with
        // unknown rooms, then we will endlessly loop.
        if (loopy_vertex.find(vertex_id) == loopy_vertex.end()) {
          // First time we see this bad one, add to map
          loopy_vertex[vertex_id] = 1;
        } else {
          // You again!?
          CHECK_GT(loopy_vertex[vertex_id], 0);
          // Typically we will revisit the same guy twice.
          static constexpr int kMaxVertexRevisits = 4;
          if (loopy_vertex[vertex_id] > kMaxVertexRevisits) {
            // If we have revisited this voxel too many times, ignore it.
            VLOG(1) << "Detected loop! Discarding vertex with id: " << vertex_id
                    << "\n Position: " << vertex.point.transpose();
            continue;
          } else {
            loopy_vertex[vertex_id] += 1;
          }
        }

        // Re-add this guy to the list, perhaps when we label the rest
        // of rooms we figure out this guy...
        VLOG(1) << "Re-ADD vertex: " << vertex_id;
        undecided_vertices_ids.push_back(vertex_id);
      } else {
        // Found the most likely room id!
        vertex.room_id = max_room_id;

        // Update all the edges of this vertex as non/transition.
        for (const int64_t& edge_id : vertex.edge_list) {
          vxb::SkeletonEdge& edge = sparse_skeleton_graph_.getEdge(edge_id);
          const auto& vtx_1 =
              sparse_skeleton_graph_.getVertex(edge.start_vertex);
          const auto& vtx_2 = sparse_skeleton_graph_.getVertex(edge.end_vertex);
          if (vtx_1.room_id == vtx_2.room_id) {
            // This edge now belongs fully to the same room.
            edge.transition_edge = false;
            CHECK_NE(vtx_1.room_id, 0) << "Can't be! We know its room!";
          } else {
            CHECK_EQ(edge.transition_edge, true);
          }
        }
      }
    } else {
      // Good one, it knows where it is.
      continue;
    }
  }

  // Visualize new skeleton.
  visualization_msgs::MarkerArray new_marker_array;
  vxb::visualizeSkeletonGraph(sparse_skeleton_graph_,
                              world_frame_,
                              &new_marker_array,
                              vxb::SkeletonGraphVizType::kRoomId,
                              skeleton_z_level_);
  segmented_sparse_graph_pub_.publish(new_marker_array);
  //////////////////////////////////////////////////////////////

  // Now find the inter room connectivity
  /// Loop over all edges, if two vertices of different rooms are
  /// connected together, add such connection to the scene-graph
  std::vector<int64_t> edge_ids;
  sparse_skeleton_graph_.getAllEdgeIds(&edge_ids);
  for (const auto& edge_id : edge_ids) {
    vxb::SkeletonEdge& edge = sparse_skeleton_graph_.getEdge(edge_id);
    const auto& vtx_1 = sparse_skeleton_graph_.getVertex(edge.start_vertex);
    const auto& vtx_2 = sparse_skeleton_graph_.getVertex(edge.end_vertex);
    if (vtx_1.room_id != vtx_2.room_id) {
      NodeId room1 =
          std::to_string(kRoomSemanticLabel) + std::to_string(vtx_1.room_id);
      NodeId room2 =
          std::to_string(kRoomSemanticLabel) + std::to_string(vtx_2.room_id);
      scene_graph_.addEdge(room1, room2);
    }
  }
  LOG(INFO) << "Done finding room layout.";


  LOG(INFO) << "COUNTING ROOMS NODES";
  std::vector<int64_t> all_vtx_ids;
  sparse_skeleton_graph_.getAllVertexIds(&all_vtx_ids);
  std::map<int, int> room_count;
  for (const auto& id : all_vtx_ids) {
    const vxb::SkeletonVertex& vertex = sparse_skeleton_graph_.getVertex(id);
    const auto& room_id = vertex.room_id;
    auto it = room_count.find(room_id);
    if (it != room_count.end()) {
      room_count[vertex.room_id] += 1;
    } else {
      // First time we see this
      room_count[vertex.room_id] = 1;
    }
  }

  for (const auto& kv : room_count) {
    LOG(INFO) << "ROOM ID: " << kv.first
              << " with N vertices: " << kv.second;
  }

  //////////////////////////////////////////////////////////////////////////////

  /////////////////////// Objects-Skeleton /////////////////////////////////////
  // Extract links between objects and skeleton, should be done in scene
  // graph...
  publishSkeletonToObjectLinks(skeleton_graph_cloud);
  //////////////////////////////////////////////////////////////////////////////

  /////////////////////// Segment Walls Mesh ///////////////////////////////////
  if (walls_mesh) {
    LOG(WARNING) << "Clustering walls...";
    vxb::Mesh segmented_walls_mesh;
    enclosing_wall_finder_->findWalls(
        *walls_mesh, sparse_skeleton_graph_, &segmented_walls_mesh);
    // 19 overrides the old walls, use another one!
    publishSemanticMesh(19, segmented_walls_mesh);
    LOG(WARNING) << "Done clustering walls...";
  } else {
    LOG(WARNING) << "Not segmenting walls mesh bcs no walls mesh found.";
  }
  //////////////////////////////////////////////////////////////////////////////

  ///////////////////////////////// BUILDINGS //////////////////////////////////
  SceneNode building_instance;
  building_finder_->findBuildings(scene_graph_, &building_instance);
  scene_graph_.addSceneNode(building_instance);
  //////////////////////////////////////////////////////////////////////////////

  LOG(INFO) << "Visualizing Scene Graph";
  scene_graph_.visualize();
}

void SceneGraphSimulationServer::publishSkeletonToObjectLinks(
    const ColorPointCloud::Ptr& skeleton_graph_pcl) {
  // TODO(TONI): bad, you should add the skeleton to the scene graph instead...
  // In the meantime:

  pcl::KdTreeFLANN<Point> kdtree;
  CHECK(skeleton_graph_pcl);
  PointCloud::Ptr skeleton_graph_pcl_converted(new PointCloud);
  pcl::copyPointCloud(*skeleton_graph_pcl, *skeleton_graph_pcl_converted);
  kdtree.setInputCloud(skeleton_graph_pcl_converted);

  // 1. Loop over the objects in the database_
  std::vector<const SceneNode*> scene_nodes;
  scene_graph_.getAllSceneNodes(&scene_nodes);
  visualization_msgs::MarkerArray obj_to_skeleton_markers;
  for (const auto& node : scene_nodes) {
    const NodeAttributes& attributes = node->attributes_;
    const SemanticLabel& semantic_label = attributes.semantic_label_;
    if (semantic_label != kRoomSemanticLabel &&
        semantic_label != kBuildingSemanticLabel) {
      // We have an object, get its centroid.
      NodePosition centroid = attributes.position_;

      // Compute nearest skeleton vertex
      static constexpr int K = 1;
      std::vector<int> nn_indices(K);
      std::vector<float> nn_squared_distances(K);
      if (kdtree.nearestKSearch(centroid, K, nn_indices, nn_squared_distances) >
          0) {
        // Found the nearest neighbor
        CHECK_GT(nn_indices.size(), 0);
        CHECK_GT(nn_squared_distances.size(), 0);
        CHECK_EQ(nn_indices.size(), nn_squared_distances.size());
        Point pcl_point = skeleton_graph_pcl_converted->at(nn_indices.at(0));
        // Pcl points leave in a diff layer than skeleton, separate them!
        // PCL points live at the first layer
        centroid.z += scene_graph_.getLayerStepZ();
        // Skeleton points live at the second layer;
        pcl_point.z += skeleton_z_level_;
        visualization_msgs::Marker marker =
            scene_graph_.getLineFromPointToPoint(
                pcl_point,
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

void SceneGraphSimulationServer::publishSemanticMesh(
    const SemanticLabel& semantic_label,
    const vxb::Mesh& semantic_mesh) {
  visualization_msgs::Marker mesh_msg;
  mesh_msg.header.frame_id = world_frame_;
  bool is_walls =
      std::find(walls_labels_.begin(), walls_labels_.end(), semantic_label) !=
      walls_labels_.end();
  // Publish twice: one at the mesh level, the other at the object level.
  // Except for walls, those should be at skeleton level.
  vxb::fillMarkerWithMesh(
      semantic_mesh,
      vxb::ColorMode::kLambertColor,
      &mesh_msg,
      is_walls ? skeleton_z_level_ : scene_graph_.getLayerStepZ());
  semantic_mesh_2_pubs_.publish(semantic_label, mesh_msg);
  visualization_msgs::Marker new_mesh_msg;
  new_mesh_msg.header.frame_id = world_frame_;
  vxb::fillMarkerWithMesh(
      semantic_mesh, vxb::ColorMode::kLambertColor, &new_mesh_msg, 0.0);
  semantic_mesh_pubs_.publish(semantic_label, new_mesh_msg);
}

void SceneGraphSimulationServer::extractThings(
    const SemanticLabel& semantic_label,
    const SemanticPointCloud::Ptr& semantic_pcl) {
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
  InstanceId instance_id = 0;
  for (size_t idx = 0; idx < n_centroids; ++idx) {
    // Create SceneNode out of centroids
    SceneNode scene_node;
    NodeAttributes& attributes = scene_node.attributes_;
    attributes.semantic_label_ = semantic_label;
    const auto& color =
        semantic_config_.semantic_label_to_color_->getColorFromSemanticLabel(
            semantic_label);
    attributes.color_ << color.r, color.g, color.b;
    attributes.instance_id_ = instance_id;
    scene_node.id_ =
        std::to_string(semantic_label) + std::to_string(instance_id);
    centroids.at(idx).get(attributes.position_);  // Calculates centroid
    attributes.pcl_ = object_pcls.at(idx);
    attributes.bounding_box_ = bounding_boxes.at(idx);

    // Add to database
    CHECK(scene_node.attributes_.pcl_) << "Pcl not initialized!";
    scene_graph_.addSceneNode(scene_node);
    ++instance_id;
  }
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
    ColorPointCloud::Ptr pointcloud) {
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

/**
 * Function to handle registration with object db
 * @param object_pcls
 * @param semantic_label
 */
ObjectPointClouds SceneGraphSimulationServer::objectDatabaseActionCall(
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
