#include "kimera_scene_graph/connectivity_utils.h"
#include "kimera_scene_graph/pcl_conversion.h"

#include <glog/logging.h>

#include <pcl/filters/crop_hull.h>
#include <pcl/search/kdtree.h>
#include <pcl/surface/concave_hull.h>

#include <kimera_dsg/node_attributes.h>

namespace kimera {

using Node = SceneGraph::Node;

void findRoomConnectivity(SceneGraph* scene_graph) {
  CHECK_NOTNULL(scene_graph);
  CHECK(scene_graph->hasLayer(to_underlying(KimeraDsgLayers::PLACES)));
  const SceneGraphLayer& places_layer =
      *(scene_graph->getLayer(to_underlying(KimeraDsgLayers::PLACES)));

  for (const auto& edge : places_layer.edges) {
    const Node& source = *(scene_graph->getNode(edge.second.source));
    if (!source.hasParent()) {
      continue;  // we could warn here, but covered by room-places connectivity
    }

    const Node& target = *(scene_graph->getNode(edge.second.target));
    if (!target.hasParent()) {
      continue;  // we could warn here, but covered by room-places connectivity
    }

    // TODO(nathan) theoretically, parent edges don't have to be in the same
    // layer

    NodeId source_room = *(source.getParent());
    NodeId target_room = *(source.getParent());
    if (source_room != target_room) {
      // the graph takes care of filtering repeated edges
      scene_graph->insertEdge(source_room, target_room);
    }
  }
}

// TODO(nathan) this is hardcoded for 2 dimensions: does this cause problems
// with multiple floors?
std::vector<int> getPlaceIndicesInsideRoom(pcl::KdTreeFLANN<ColorPoint>& kdtree,
                                           ColorPointCloud::Ptr room_pcl,
                                           ColorPointCloud::Ptr places_pcl) {
  CHECK(room_pcl);
  CHECK(places_pcl);
  // Find concave hull of pcl first
  VLOG(2) << "Finding concave hull of room.";
  pcl::ConcaveHull<ColorPoint> concave_hull_adapter;
  static constexpr double kAlpha = 0.30;
  concave_hull_adapter.setAlpha(kAlpha);
  concave_hull_adapter.setKeepInformation(true);
  concave_hull_adapter.setDimension(2);
  concave_hull_adapter.setInputCloud(room_pcl);
  ColorPointCloud::Ptr concave_hull_pcl(new ColorPointCloud());
  std::vector<pcl::Vertices> concave_polygon;
  concave_hull_adapter.reconstruct(*concave_hull_pcl, concave_polygon);
  VLOG(2) << "Done finding concave hull of room.";

  // Setup kd-tree
  kdtree.setInputCloud(concave_hull_pcl);

  // Create cropper to know which vertices are outside the room
  VLOG(2) << "Finding indices of places inside room.";
  pcl::CropHull<ColorPoint> hull_cropper;
  hull_cropper.setHullIndices(concave_polygon);
  hull_cropper.setHullCloud(concave_hull_pcl);
  hull_cropper.setDim(2);
  // because we can only get removed indices, tell the algorithm to
  // remove inside.
  hull_cropper.setCropOutside(true);

  hull_cropper.setInputCloud(places_pcl);
  // These indices correspond to nodes inside the current room.
  std::vector<int> removed_indices_inside_room;
  hull_cropper.filter(removed_indices_inside_room);
  VLOG(2) << "Done finding indices of places inside room.";
  return removed_indices_inside_room;
}

struct NNResult {
  bool valid;
  float distance_nn_2d;
};

NNResult getDistanceToRoom(const pcl::KdTreeFLANN<ColorPoint>& kdtree,
                           const Eigen::Vector3d& position) {
  // the search is 2D, because the truncation of the esdf is 2D
  ColorPoint search_point;
  search_point.x = position(0);
  search_point.y = position(1);
  search_point.z = 0.0;

  static constexpr int K = 1;
  std::vector<int> nn_indices(K);
  std::vector<float> nn_distances(K);
  if (kdtree.nearestKSearch(search_point, K, nn_indices, nn_distances) == 0) {
    return {false, -1.0f};
  }

  // TODO(nathan) reconsider these checks
  CHECK_GT(nn_indices.size(), 0u);
  CHECK_GT(nn_distances.size(), 0u);
  // Don't take the squared distances, since we need to project in 2D!
  const ColorPoint& pcl_point = kdtree.getInputCloud()->at(nn_indices.at(0));
  float esdf_distance_approx =
      std::sqrt(std::pow(search_point.x - pcl_point.x, 2) +
                std::pow(search_point.y - pcl_point.y, 2));

  return {true, esdf_distance_approx};
}

void findPlacesRoomConnectivity(SceneGraph* scene_graph,
                                float esdf_truncation_distance) {
  CHECK_NOTNULL(scene_graph);

  const SceneGraphLayer& room_layer =
      *(scene_graph->getLayer(to_underlying(KimeraDsgLayers::ROOMS)));
  const SceneGraphLayer& places_layer =
      *(scene_graph->getLayer(to_underlying(KimeraDsgLayers::PLACES)));

  // Create cloud of the graph, make sure indices are 1-to-1
  PclLayer<ColorPointCloud> places_pcl =
      convertLayerToPcl<ColorPointCloud>(places_layer);

  // Init kd-tree
  pcl::KdTreeFLANN<ColorPoint> kdtree;
  for (const auto& id_node_pair : room_layer.nodes) {
    const NodeId room_id = id_node_pair.first;
    LOG(INFO) << "Segmenting places for room with id: "
              << NodeSymbol(room_id).getLabel();

    ColorPointCloud::Ptr room_pcl =
        id_node_pair.second->attributes<RoomNodeAttributes>().points;
    CHECK(room_pcl);
    CHECK(!room_pcl->empty());

    // Get section of the sparse graph that falls inside this room_pcl
    std::vector<int> removed_indices_inside_room =
        getPlaceIndicesInsideRoom(kdtree, room_pcl, places_pcl.cloud);

    // add edges between room and places nodes within the room
    VLOG(2) << "Starting Initial linking of places to rooms.";
    for (const auto& inside_room_place_index : removed_indices_inside_room) {
      // TODO(nathan) propagate room color to place once we add casting
      // TODO(nathan) consider duplicate parent warning
      scene_graph->insertEdge(
          room_id, places_pcl.cloud_to_layer_ids.at(inside_room_place_index));
    }
    VLOG(2) << "Done with initial linking of places to rooms.";

    // handle adding connections between the concave hull and the room extents
    VLOG(2) << "Labeling place neighbors that are within truncation distance";
    for (const auto& inside_room_place_index : removed_indices_inside_room) {
      const NodeId place_id =
          places_pcl.cloud_to_layer_ids.at(inside_room_place_index);
      CHECK(scene_graph->hasNode(place_id));
      const Node& place_node = *(scene_graph->getNode(place_id));
      for (const auto& sibling_id : place_node.siblings) {
        const Node& sibling_node = *(scene_graph->getNode(sibling_id));

        std::optional<NodeId> sibling_room = sibling_node.getParent();
        if (sibling_room && *sibling_room == room_id) {
          continue;  // intra-room edge
        }

        if (sibling_room && *sibling_room != room_id) {
          // TODO(nathan) lots of comments about transition edges, but not sure
          // if we have to do something specific
          continue;  // transition edge, skip
        }

        NNResult result =
            getDistanceToRoom(kdtree, sibling_node.attributes().position);
        if (!result.valid) {
          LOG(WARNING) << "Failed to find nearest neighbor to place "
                       << sibling_id << " w.r.t. room " << room_id;
        }

        const float truncation_threshold = 1.1f * esdf_truncation_distance;
        if (result.distance_nn_2d < truncation_threshold) {
          // TODO(nathan) this used to increment the queue. May consider this
          // (with visited) in the future
          scene_graph->insertEdge(room_id, sibling_id);
        }
      }
    }
    VLOG(2) << "Finished labeling places inside truncation distance";
  }

  std::list<NodeId> unlabeled_nodes;
  for (const auto& id_node_pair : places_layer.nodes) {
    if (!id_node_pair.second->hasParent() &&
        id_node_pair.second->hasSiblings()) {
      unlabeled_nodes.push_back(id_node_pair.first);
    }
  }

  std::unordered_map<NodeId, int> num_visits;
  while (unlabeled_nodes.size() > 0u) {
    NodeId node_id = unlabeled_nodes.front();
    unlabeled_nodes.pop_front();

    if (num_visits.count(node_id)) {
      num_visits[node_id]++;
    } else {
      num_visits[node_id] = 1;
    }

    static constexpr int kMaxVertexRevisits = 4;
    if (num_visits.at(node_id) == kMaxVertexRevisits) {
      VLOG(2) << "Failed to determine room for vertex " << node_id << "after "
              << kMaxVertexRevisits << " tries!";
      continue;
    }

    const Node& node = *(scene_graph->getNode(node_id));

    std::map<NodeId, size_t> room_votes;
    for (const auto& sibling_id : node.siblings) {
      const Node& sibling_node = *(scene_graph->getNode(sibling_id));
      if (!sibling_node.hasParent()) {
        continue;  // another undecided node
      }

      NodeId room_id = *(sibling_node.getParent());
      if (room_votes.count(room_id)) {
        room_votes[room_id]++;
      } else {
        room_votes[room_id] = 1;
      }
    }

    if (room_votes.empty()) {
      // retry the node once more nodes have been labeled
      unlabeled_nodes.push_back(node_id);
      continue;
    }

    auto best_room = std::max_element(room_votes.begin(),
                                      room_votes.end(),
                                      [](const auto& lhs, const auto& rhs) {
                                        return lhs.second < rhs.second;
                                      });
    CHECK(best_room != room_votes.end());
    scene_graph->insertEdge(best_room->first, node_id);
  }

  for (const auto& id_place_pair : places_layer.nodes) {
    const Node& place = *id_place_pair.second;
    if (!place.hasParent()) {
      continue;
    }
    const Node& parent = *(scene_graph->getNode(*(place.getParent())));
    place.attributes<SemanticNodeAttributes>().color =
        parent.attributes<SemanticNodeAttributes>().color;
  }
}

std::optional<NodeId> findNearestPlace(
    const pcl::KdTreeFLANN<ColorPoint>& kd_tree,
    const PclLayer<ColorPointCloud>& pcl_layer,
    const Eigen::Vector3d& position) {
  ColorPoint query_point;
  query_point.x = position(0);
  query_point.y = position(1);
  query_point.z = position(2);

  std::vector<int> nn_indices(1);
  std::vector<float> nn_distances(1);
  if (kd_tree.nearestKSearch(query_point, 1, nn_indices, nn_distances) == 0) {
    return std::nullopt;
  }

  return pcl_layer.cloud_to_layer_ids.at(nn_indices.at(0));
}

void findObjectPlaceConnectivity(SceneGraph* scene_graph) {
  CHECK_NOTNULL(scene_graph);

  CHECK(scene_graph->hasLayer(to_underlying(KimeraDsgLayers::OBJECTS)));
  const SceneGraphLayer& objects_layer =
      *(scene_graph->getLayer(to_underlying(KimeraDsgLayers::OBJECTS)));

  CHECK(scene_graph->hasLayer(to_underlying(KimeraDsgLayers::PLACES)));
  const SceneGraphLayer& places_layer =
      *(scene_graph->getLayer(to_underlying(KimeraDsgLayers::PLACES)));
  CHECK_GE(places_layer.numNodes(), 0u) << "Empty places layer";

  PclLayer<ColorPointCloud> pcl_places =
      convertLayerToPcl<ColorPointCloud>(places_layer);

  pcl::KdTreeFLANN<ColorPoint> kd_tree;
  kd_tree.setInputCloud(pcl_places.cloud);

  // Iterate over the nodes of objects layer
  for (const auto& id_node_pair : objects_layer.nodes) {
    if (id_node_pair.second->hasParent()) {
      LOG(WARNING) << "Object " << id_node_pair.first
                   << "has a parent already!";
      continue;
    }

    VLOG(5) << "Finding place of object with id: " << id_node_pair.first;

    std::optional<NodeId> nearest_place = findNearestPlace(
        kd_tree, pcl_places, id_node_pair.second->attributes().position);
    if (!nearest_place) {
      LOG(WARNING) << "No place found for object with id: "
                   << id_node_pair.first;
      continue;
    }

    scene_graph->insertEdge(*nearest_place, id_node_pair.first);
  }
}

}  // namespace kimera
