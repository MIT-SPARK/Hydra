#pragma once

#include <glog/logging.h>

#include <ros/ros.h>

#include <voxblox_skeleton/ros/skeleton_vis.h>
#include <voxblox_skeleton/skeleton.h>

#include <kimera_semantics/common.h>

#include <pcl/filters/crop_hull.h>
#include <pcl/surface/concave_hull.h>

#include "kimera_scene_graph/common.h"
#include "kimera_scene_graph/scene_node.h"
#include "kimera_scene_graph/utils/kimera_to_voxblox.h"

namespace kimera {

class PlacesRoomConnectivityFinder {
 public:
  PlacesRoomConnectivityFinder(const ros::NodeHandle& nh_private,
                               const float& skeleton_z_level,
                               const std::string& world_frame)
      : nh_private_(nh_private),
        segmented_sparse_graph_pub_(),
        skeleton_z_level_(skeleton_z_level),
        world_frame_(world_frame) {
    segmented_sparse_graph_pub_ =
        nh_private_.advertise<visualization_msgs::MarkerArray>(
            "segmented_sparse_graph", 1, true);
  }
  ~PlacesRoomConnectivityFinder() = default;

 public:
  void findPlacesRoomConnectivity(SceneGraph* scene_graph) {
    CHECK_NOTNULL(scene_graph);

    // Get relevant layers: rooms and places
    SceneGraphLayer room_layer(LayerId::kRoomsLayerId);
    CHECK(scene_graph->getLayerSafe(LayerId::kRoomsLayerId, &room_layer));
    SceneGraphLayer* places_layer =
        scene_graph->getLayerMutable(LayerId::kPlacesLayerId);
    CHECK_NOTNULL(places_layer);

    // Convert scene graph layer to skeleton graph? use only for visualization
    // perhaps.
    LOG(INFO) << "Convert places layer to skeleton.";
    vxb::SparseSkeletonGraph non_segmented_places_skeleton;
    places_layer->convertLayerToSkeleton(&non_segmented_places_skeleton);
    LOG(INFO) << "Done converting places layer to skeleton.";

    // Show the graph before any inference
    LOG(INFO) << "Publish places graph before any segmentation.";
    visualization_msgs::MarkerArray marker_array;
    vxb::visualizeSkeletonGraph(non_segmented_places_skeleton,
                                world_frame_,
                                &marker_array,
                                vxb::SkeletonGraphVizType::kRoomId,
                                skeleton_z_level_);
    segmented_sparse_graph_pub_.publish(marker_array);
    LOG(INFO) << "Done publishing places graph before any segmentation.";

    // Create cloud of the graph, make sure indices are 1-to-1
    std::vector<NodeId> vertex_ids;
    std::map<int, NodeId> cloud_to_layer_ids;
    ColorPointCloud::Ptr places_graph_cloud =
        places_layer->convertLayerToPcl(&cloud_to_layer_ids, &vertex_ids);
    CHECK_EQ(places_graph_cloud->size(), places_layer->getNumberOfNodes());

    // Init kd-tree
    pcl::KdTreeFLANN<ColorPoint> kdtree;

    // Loop over room layer in scene graph, and use the pcl_ attribute
    for (const std::pair<NodeId, SceneGraphNode>& room :
         room_layer.getNodeIdMap()) {
      LOG(INFO) << "Segmenting places for room with id: " << room.first;

      ////////////////////// Associate room to sparse graph...
      // Get section of the sparse graph that falls inside this room_pcl
      NodePcl::Ptr room_pcl = room.second.attributes_.pcl_;
      CHECK(room_pcl);
      CHECK(!room_pcl->empty());

      // Find concave hull of pcl first
      LOG(INFO) << "Finding concave hull of pcl.";
      pcl::ConcaveHull<ColorPoint> concave_hull_adapter;
      static constexpr double kAlpha = 0.30;
      concave_hull_adapter.setAlpha(kAlpha);
      concave_hull_adapter.setKeepInformation(true);
      concave_hull_adapter.setDimension(2);
      concave_hull_adapter.setInputCloud(room_pcl);
      ColorPointCloud::Ptr concave_hull_pcl(new ColorPointCloud);
      std::vector<pcl::Vertices> concave_polygon;
      concave_hull_adapter.reconstruct(*concave_hull_pcl, concave_polygon);
      LOG(INFO) << "Done finding concave hull of pcl.";

      // Setup kd-tree
      kdtree.setInputCloud(concave_hull_pcl);

      // Create cropper to know which vertices are outside the room
      LOG(INFO) << "Finding indices of places inside room.";
      pcl::CropHull<ColorPoint> hull_cropper;
      hull_cropper.setHullIndices(concave_polygon);
      hull_cropper.setHullCloud(concave_hull_pcl);
      hull_cropper.setDim(2);
      // because we can only get removed indices, tell the algorithm to
      // remove inside.
      hull_cropper.setCropOutside(true);

      hull_cropper.setInputCloud(places_graph_cloud);
      // These indices correspond to nodes inside the current room.
      std::vector<int> removed_indices_inside_room;
      hull_cropper.filter(removed_indices_inside_room);
      LOG(INFO) << "Done finding indices of places inside room.";

      // Loop over graph and associate room to the removed vertices
      // and label edges according to their type.
      LOG(INFO) << "Starting Initial linking of places to rooms.";
      std::vector<EdgeId> edge_ids;
      for (const int& inside_room_place_index : removed_indices_inside_room) {
        CHECK_LE(inside_room_place_index, cloud_to_layer_ids.size());
        CHECK_NOTNULL(places_layer);
        SceneGraphNode& place_node = places_layer->getNodeMutable(
            cloud_to_layer_ids[inside_room_place_index]);
        if (place_node.hasParent()) {
          // This vertex is already labeled!
          LOG(WARNING) << "Vertex already labeled when finding rooms.";
          continue;
        }
        // Identify this vertex as belonging to the room by updating its parent
        // edge. (but this is an inter layer edge, use scene graph!
        linkPlaceToRoom(room.first, place_node.node_id_, scene_graph);

        // Color the place node with the same color of the room it is in.
        place_node.attributes_.color_ =
            utils::vxbColorToNodeColor(getRoomColor(room.second.node_id_));
        std::vector<EdgeId> siblings =
            getEdgeIdsInEdgeIdMap(place_node.siblings_edge_map_);
        edge_ids.insert(edge_ids.end(), siblings.begin(), siblings.end());
      }
      LOG(INFO) << "Done with initial linking of places to rooms.";

      // "Dilate" the labels, for every null vertex, if one of its neighbors
      // is a room and they are closer than esdf truncation distance,
      // change its id to room x.
      // Label edges according to whether they are transition or not
      // FLOODFILL LIKE CRAZY
      LOG(INFO) << "Starting floodfill for labelling unlabelled places.";
      while (edge_ids.size() > 0) {
        // Depth first.
        auto edge_idx = edge_ids.back();  // copy (bcs we delete next)
        edge_ids.pop_back();
        SceneGraphLayer* place_layer =
            scene_graph->getLayerMutable(LayerId::kPlacesLayerId);
        CHECK_NOTNULL(place_layer);
        SceneGraphEdge& edge = place_layer->getIntraLayerEdgeMutable(edge_idx);
        SceneGraphNode& vtx_start =
            place_layer->getNodeMutable(edge.start_node_id_);
        SceneGraphNode& vtx_end =
            place_layer->getNodeMutable(edge.end_node_id_);

        CHECK(edge.start_layer_id_ == LayerId::kPlacesLayerId) << edge.print();
        CHECK(edge.end_layer_id_ == LayerId::kPlacesLayerId) << edge.print();
        CHECK(vtx_start.layer_id_ == LayerId::kPlacesLayerId)
            << vtx_start.print();
        CHECK(vtx_end.layer_id_ == LayerId::kPlacesLayerId) << vtx_end.print();
        CHECK(
            !vtx_start.parent_edge_.isEdgeValid() ||
            vtx_start.parent_edge_.start_layer_id_ == LayerId::kRoomsLayerId)
            << vtx_start.print();

        // Both nodes' parent edge, if valid, should be consistent with the fact
        // that the nodes are in the places layer.
        CHECK(!vtx_start.parent_edge_.isEdgeValid() ||
              vtx_start.parent_edge_.end_layer_id_ == LayerId::kPlacesLayerId)
            << vtx_start.print();
        CHECK(!vtx_end.parent_edge_.isEdgeValid() ||
              vtx_end.parent_edge_.end_layer_id_ == LayerId::kPlacesLayerId)
            << vtx_end.print();

        NodeId current_room_id = room.second.node_id_;
        NodeId vtx_start_room_id = vtx_start.parent_edge_.start_node_id_;
        NodeId vtx_end_room_id = vtx_end.parent_edge_.start_node_id_;

        bool is_start_place_inside_room = false;
        if (vtx_start_room_id == current_room_id) {
          is_start_place_inside_room = true;
        } else if (vtx_end_room_id == current_room_id) {
          is_start_place_inside_room = false;
        } else {
          LOG(ERROR) << "At least one vertex of the edge should belong to this "
                        "room...";
          continue;
        }

        if (vtx_start_room_id == vtx_end_room_id) {
          // We have a self-contained edge, label the edge as non-transition
          // edge.transition_edge = false;
        } else {
          // We have a transition edge, from one room to unknown.
          // Search for its nearest neighbor to the current pcl of the room
          // do the search in 2D, because the truncation of the esdf is done
          // in 2D...
          ColorPoint search_point;
          search_point.z = 0.0;
          if (is_start_place_inside_room) {
            // If start is in room, we are trying to label the end.
            search_point.x = vtx_end.attributes_.position_.x;
            search_point.y = vtx_end.attributes_.position_.y;
          } else {
            search_point.x = vtx_start.attributes_.position_.x;
            search_point.y = vtx_start.attributes_.position_.y;
          }

          float esdf_distance_approx = 0;
          static constexpr int K = 1;  // find nearest-neighbor
          std::vector<int> nn_indices(K);
          std::vector<float> nn_squared_distances(K);
          if (kdtree.nearestKSearch(
                  search_point, K, nn_indices, nn_squared_distances) > 0) {
            // Found the nearest neighbor
            CHECK_GT(nn_indices.size(), 0);
            CHECK_GT(nn_squared_distances.size(), 0);
            CHECK_EQ(nn_indices.size(), nn_squared_distances.size());
            // Don't take the squared distances, since we need to project in 2D!
            const ColorPoint& pcl_point =
                concave_hull_pcl->at(nn_indices.at(0));
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
            // unknown vertex as being in the room and add its neighbors to the
            // queue.
            std::vector<EdgeId> siblings;
            if (is_start_place_inside_room) {
              linkPlaceToRoom(room.first, vtx_end.node_id_, scene_graph);
              siblings = getEdgeIdsInEdgeIdMap(vtx_end.siblings_edge_map_);
            } else {
              linkPlaceToRoom(room.first, vtx_start.node_id_, scene_graph);
              siblings = getEdgeIdsInEdgeIdMap(vtx_start.siblings_edge_map_);
            }
            // edge_ids.insert(edge_ids.end(), siblings.begin(),
            // siblings.end());
            CHECK_EQ(vtx_end.parent_edge_.start_node_id_,
                     vtx_start.parent_edge_.start_node_id_);
          } else {
            // That is indeed a transition edge!
            // Store what kind of transition
            // If we transit to NULL, that is perhaps a corridor or door...
          }
        }
      }
      LOG(INFO) << "Finished floodfill for labelling unlabelled places.";

      // Visualize new skeleton for each room.
      vxb::SparseSkeletonGraph semi_segmented_places_skeleton;
      CHECK_NOTNULL(places_layer);
      places_layer->convertLayerToSkeleton(&semi_segmented_places_skeleton);
      visualization_msgs::MarkerArray marker_array;
      vxb::visualizeSkeletonGraph(semi_segmented_places_skeleton,
                                  world_frame_,
                                  &marker_array,
                                  vxb::SkeletonGraphVizType::kRoomId,
                                  skeleton_z_level_);
      segmented_sparse_graph_pub_.publish(marker_array);
    }

    // Finally, do majority voting on the unknown folks...
    // Get all vertices that do not know in which room they are.
    std::list<NodeId> undecided_vertices_ids;
    for (const auto& vertex_id : vertex_ids) {
      undecided_vertices_ids.push_back(vertex_id);
    }
    std::unordered_map<NodeId, int> loopy_vertex;
    while (undecided_vertices_ids.size() > 0u) {
      // Breadth first.
      NodeId vertex_id = undecided_vertices_ids.front();  // copy
      undecided_vertices_ids.pop_front();
      SceneGraphNode& place_node = places_layer->getNodeMutable(vertex_id);
      std::map<int, int64_t> room_votes;
      if (!place_node.hasParent()) {
        // Bad one, it has no clue in which room it is.

        // Check that it has neighbors
        if (place_node.hasSiblings()) {
          // This folk has no neighbors :O
          LOG(WARNING) << "Graph vertex with id " << vertex_id
                       << " has no neighbors!";
          continue;
        }

        // Ask its neighbors where it is!
        for (const std::pair<EdgeId, SceneGraphEdge>& edge_kv :
             place_node.siblings_edge_map_) {
          const SceneGraphEdge& edge =
              places_layer->getIntraLayerEdge(edge_kv.first);

          // Check for self-edges
          LOG_IF(FATAL, edge.end_node_id_ == edge.start_node_id_)
              << "Self-edges are not allowed! Removing edge.\n"
              << edge.print();

          // Get neighbor voxel for this edge
          SceneGraphNode* neighbor_vertex = nullptr;
          if (edge.end_node_id_ != vertex_id) {
            // Hello nice neighbor!
            neighbor_vertex = &places_layer->getNodeMutable(edge.end_node_id_);
          } else if (edge.start_node_id_ != vertex_id) {
            // Oh the end_vertex was the current vertex, hi actual
            // neighbor!
            neighbor_vertex =
                &places_layer->getNodeMutable(edge.start_node_id_);
          } else {
            LOG(ERROR) << "The edge does not belong to this vtx! "
                       << "This shouldn't happen.\n"
                       << "Vertex id: " << vertex_id << '\n'
                       << edge.print();
            // Forget about this edge for now... but this should not
            // happen!
            continue;
          }
          CHECK(neighbor_vertex);

          // So, neighbor, tell me, in which room are you?
          auto neighbor_room_id = neighbor_vertex->parent_edge_.start_node_id_;
          auto it = room_votes.find(neighbor_room_id);
          if (it == room_votes.end()) {
            // First time we see this room id.
            room_votes[neighbor_room_id] = 1;
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
              VLOG(1) << "Detected loop! Discarding vertex with id: "
                      << vertex_id
                      << "\n Position: " << place_node.attributes_.position_;
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
          linkPlaceToRoom(max_room_id, place_node.node_id_, scene_graph);
        }
      } else {
        // Good one, it knows where it is.
        continue;
      }
    }

    // Visualize new skeleton for all rooms.
    vxb::SparseSkeletonGraph segmented_places_skeleton;
    CHECK_NOTNULL(places_layer);
    places_layer->convertLayerToSkeleton(&segmented_places_skeleton);
    visualization_msgs::MarkerArray new_marker_array;
    vxb::visualizeSkeletonGraph(segmented_places_skeleton,
                                world_frame_,
                                &new_marker_array,
                                vxb::SkeletonGraphVizType::kRoomId,
                                skeleton_z_level_);
    segmented_sparse_graph_pub_.publish(new_marker_array);
  }

  void linkPlaceToRoom(const NodeId& room_id,
                       const NodeId& place_id,
                       SceneGraph* scene_graph) {
    CHECK_NOTNULL(scene_graph);
    SceneGraphEdge parent_edge;
    parent_edge.start_node_id_ = room_id;
    parent_edge.start_layer_id_ = LayerId::kRoomsLayerId;
    parent_edge.end_node_id_ = place_id;
    parent_edge.end_layer_id_ = LayerId::kPlacesLayerId;
    scene_graph->addEdge(&parent_edge);
  }

 private:
  ros::NodeHandle nh_private_;
  ros::Publisher segmented_sparse_graph_pub_;
  std::string world_frame_;

  // TODO(Toni): remove
  float skeleton_z_level_;
};

}  // namespace kimera
