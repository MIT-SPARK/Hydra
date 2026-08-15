#pragma once
#include <spark_dsg/scene_graph_layer.h>

#include <Eigen/Dense>
#include <list>
#include <map>

#include "clio/scene_graph_types.h"

namespace clio {

struct ClusteringWorkspace {
  using NodeEmbeddings = std::map<NodeId, Eigen::VectorXf>;
  std::map<size_t, Eigen::VectorXf> features;
  std::map<size_t, NodeId> node_lookup;
  std::map<NodeId, size_t> order;
  std::map<EdgeKey, double> edges;
  std::vector<size_t> assignments;

  ClusteringWorkspace(const spark_dsg::SceneGraphLayer& layer,
                      const NodeEmbeddings& node_embeddings);

  ClusteringWorkspace(const spark_dsg::SceneGraphLayer& layer,
                      const std::vector<NodeId>& nodes);

  ClusteringWorkspace(const spark_dsg::SceneGraphLayer& layer);

  size_t size() const;

  size_t featureDim() const;

  std::list<EdgeKey> addMerge(EdgeKey to_merge);

  std::vector<std::vector<NodeId>> getClusters() const;
};

}  // namespace clio
