#include "kimera_dsg_builder/dsg_backend.h"
#include "kimera_dsg_builder/pcl_conversion.h"

#include <kimera_dsg/node_attributes.h>
#include <pcl/search/kdtree.h>

#include <glog/logging.h>

namespace kimera {

using kimera_pgmo::DeformationGraph;
using kimera_pgmo::DeformationGraphPtr;
using kimera_pgmo::KimeraPgmo;
using Node = SceneGraph::Node;

DsgBackend::DsgBackend(const ros::NodeHandle nh,
                       const std::map<LayerId, char>& layer_id_map,
                       LayerId mesh_layer_id)
    : nh_(nh), initialized_(false), layer_id_map_(layer_id_map) {
  // TODO(nathan) check ordering here
  // TODO(nathan) make symbols from the map
  DynamicSceneGraph::LayerIds layer_ids;
  for (const auto& id_key_pair : layer_id_map) {
    if (id_key_pair.first == mesh_layer_id) {
      LOG(ERROR) << "Found duplicate layer id " << id_key_pair.first
                 << " with mesh: " << mesh_layer_id
                 << ". Backend will remain uninitialized!";
      return;
    }

    layer_ids.push_back(id_key_pair.first);
  }

  graph_.reset(new DynamicSceneGraph(layer_ids, mesh_layer_id));
  pgmo_.reset(new KimeraPgmo());

  //graph_->setMesh(pgmo_->getOptimizedMeshPtr());
  CHECK(graph_->hasLayer(graph_->getMeshLayerId()))
      << "Uninitialized mesh from PGMO";

  pgmo_->initialize(nh);
  DeformationGraphPtr deformations = pgmo_->getDeformationGraphPtr();
  deformations->storeOnlyNoOptimization();
}

void DsgBackend::addLoopClosure() {
  // add to deformation graph
}

gtsam::Pose3 getPose(const DynamicSceneGraph& graph, NodeId node) {
  return gtsam::Pose3(gtsam::Rot3(), gtsam::Vector3(graph.getPosition(node)));
}

kimera_pgmo::Vertices getNearestMeshVertices(
    const pcl::KdTreeFLANN<pcl::PointXYZ>& kdtree,
    const Eigen::Vector3d& curr_position,
    size_t num_neighbors) {
  pcl::PointXYZ search_point;
  search_point.x = curr_position(0);
  search_point.y = curr_position(1);
  search_point.z = curr_position(2);

  std::vector<int> indices(num_neighbors);
  std::vector<float> distances(num_neighbors);
  int num_found =
      kdtree.nearestKSearch(search_point, num_neighbors, indices, distances);

  kimera_pgmo::Vertices to_return;
  for (int i = 0; i < num_found; ++i) {
    to_return.push_back(indices.at(i));
  }

  return to_return;
}

void fillKdTreeFromDeformationGraph(const DeformationGraphPtr& deformations,
                                    pcl::KdTreeFLANN<pcl::PointXYZ>& kd_tree) {
  // TODO(nathan) this is messy
  pcl::PointCloud<pcl::PointXYZ>::Ptr vertices(
      new pcl::PointCloud<pcl::PointXYZ>());
  std::vector<gtsam::Point3> control_points =
      deformations->getInitialPositionsVertices('v');
  for (const auto& point : control_points) {
    pcl::PointXYZ pcl_point;
    pcl_point.x = point.x();
    pcl_point.y = point.y();
    pcl_point.z = point.z();
    vertices->push_back(pcl_point);
  }

  kd_tree.setInputCloud(vertices);
}

void DsgBackend::addPlacesToDeformationGraph() {
  CHECK(graph_->hasLayer(KimeraDsgLayers::PLACES));
  const SceneGraphLayer& places = *(graph_->getLayer(KimeraDsgLayers::PLACES));

  if (places.nodes().empty()) {
    LOG(WARNING) << "Attempting to add places to deformation graph with empty "
                    "places layer";
    return;
  }

  DeformationGraphPtr deformations = pgmo_->getDeformationGraphPtr();

  pcl::KdTreeFLANN<pcl::PointXYZ> kd_tree;
  fillKdTreeFromDeformationGraph(deformations, kd_tree);

  std::queue<NodeId> frontier;
  frontier.push(places.nodes().begin()->first);
  std::set<NodeId> visited;
  while (!frontier.empty()) {
    const Node& node = graph_->getNode(frontier.front()).value();
    frontier.pop();
    visited.insert(node.id);

    gtsam::Pose3 curr_pose = getPose(*graph_, node.id);
    if (!deformations->getGtsamValues().exists(node.id)) {
      deformations->addNewNode(node.id, curr_pose, false);
    }

    bool is_min_neighbor = true;
    const double curr_distance =
        node.attributes<PlaceNodeAttributes>().distance;
    for (const auto& sibling : node.siblings()) {
      const Node& sibling_node = graph_->getNode(sibling).value();
      const double sibling_distance =
          sibling_node.attributes<PlaceNodeAttributes>().distance;

      if (sibling_distance < curr_distance) {
        is_min_neighbor = false;
      }

      if (visited.count(sibling)) {
        continue;
      }

      frontier.push(sibling);

      // TODO(nathan) we need to fix this
      gtsam::Pose3 sibling_pose = getPose(*graph_, sibling);
      deformations->addNewBetween(
          node.id, sibling, curr_pose.between(sibling_pose));
    }

    if (!is_min_neighbor) {
      continue;  // only exterior nodes connect to mesh directly
    }

    kimera_pgmo::Vertices mesh_neighbors =
        getNearestMeshVertices(kd_tree, curr_pose.translation(), 5);
    // TODO(nathan) don't hard-code the key
    deformations->addNodeValence(node.id, mesh_neighbors, 'v');
  }
}

void DsgBackend::optimize() {
  addPlacesToDeformationGraph();

  DeformationGraphPtr deformations = pgmo_->getDeformationGraphPtr();
  deformations->optimize();

  deformGraph();
}

void DsgBackend::deformGraph() {
  CHECK(graph_->hasLayer(KimeraDsgLayers::PLACES));
  const SceneGraphLayer& places = *(graph_->getLayer(KimeraDsgLayers::PLACES));

  DeformationGraphPtr deformations = pgmo_->getDeformationGraphPtr();
  gtsam::Values new_values = deformations->getGtsamValues();
  for (const auto& id_node_pair : places.nodes()) {
    if (!new_values.exists(id_node_pair.first)) {
    // TODO(nathan) use deformPoints for places if we don't add all of them to the graph
      VLOG(1) << "Place " << NodeSymbol(id_node_pair.first).getLabel()
              << " was not in deformation graph";
      continue;
    }

    auto pose = new_values.at<gtsam::Pose3>(id_node_pair.first);
    id_node_pair.second->attributes().position = pose.translation();
  }

  // TODO(nathan) move objects either using deformPoints or new mesh vertices
  // TODO(nathan) compute new room centroid
  // TODO(nathan) compute new building centroid
  // TODO(nathan) structure?
}

}  // namespace kimera
