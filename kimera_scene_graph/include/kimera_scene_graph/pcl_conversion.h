#pragma once
#include <kimera_dsg/scene_graph_layer.h>

namespace kimera {

template <typename CloudType>
struct PclLayer {
  typename CloudType::Ptr cloud;
  std::map<size_t, NodeId> cloud_to_layer_ids;
};

template <typename CloudType>
PclLayer<CloudType> convertLayerToPcl(const SceneGraphLayer& layer) {
  PclLayer<CloudType> to_return;
  to_return.cloud.reset(new CloudType());
  to_return.cloud->resize(layer.numNodes());

  size_t next_point_id = 0u;
  for (const auto& id_node_pair : layer.nodes) {
    Eigen::Vector3d position = id_node_pair.second->attributes().position;
    typename CloudType::PointType point;
    point.x = position(0);
    point.y = position(1);
    point.z = position(2);
    to_return.cloud->at(next_point_id) = point;
    to_return.cloud_to_layer_ids[next_point_id] = id_node_pair.first;
    next_point_id++;
  }

  return to_return;
}

}  // namespace kimera
