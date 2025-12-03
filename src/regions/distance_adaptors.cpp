#include "hydra/regions/distance_adaptors.h"

#include <glog/logging.h>

namespace hydra {

template <typename Attr>
double getDistance(const SceneGraphNode& node) {
  const auto attrs = node.tryAttributes<Attr>();
  if (!attrs) {
    LOG(WARNING) << "Node " << NodeSymbol(node.id).str()
                 << " does not have correct attribute type!";
    return 0.0;
  }

  return attrs->distance;
}

double FreespaceDistanceAdaptor::distance(const SceneGraphNode& node) const {
  return getDistance<PlaceNodeAttributes>(node);
}

double FreespaceDistanceAdaptor::distance(const SceneGraphEdge& edge) const {
  return edge.info->weight;
}

double TraversabilityDistanceAdaptor::distance(const SceneGraphNode& node) const {
  return getDistance<TraversabilityNodeAttributes>(node);
}

double TraversabilityDistanceAdaptor::distance(const SceneGraphEdge& edge) const {
  return edge.info->weight;
}

}  // namespace hydra
