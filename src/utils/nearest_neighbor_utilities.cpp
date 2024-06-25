/* -----------------------------------------------------------------------------
 * Copyright 2022 Massachusetts Institute of Technology.
 * All Rights Reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  1. Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *
 *  2. Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Research was sponsored by the United States Air Force Research Laboratory and
 * the United States Air Force Artificial Intelligence Accelerator and was
 * accomplished under Cooperative Agreement Number FA8750-19-2-1000. The views
 * and conclusions contained in this document are those of the authors and should
 * not be interpreted as representing the official policies, either expressed or
 * implied, of the United States Air Force or the U.S. Government. The U.S.
 * Government is authorized to reproduce and distribute reprints for Government
 * purposes notwithstanding any copyright notation herein.
 * -------------------------------------------------------------------------- */
#include "hydra/utils/nearest_neighbor_utilities.h"

#include <glog/logging.h>

#include <nanoflann.hpp>

namespace hydra {

using nanoflann::KDTreeSingleIndexAdaptor;
using nanoflann::KDTreeSingleIndexDynamicAdaptor;
using nanoflann::L2_Simple_Adaptor;

struct GraphKdTreeAdaptor {
  GraphKdTreeAdaptor(const SceneGraphLayer& layer, const std::vector<NodeId>& nodes)
      : layer(layer), nodes(nodes) {}

  inline size_t kdtree_get_point_count() const { return nodes.size(); }

  inline double kdtree_get_pt(const size_t idx, const size_t dim) const {
    return layer.getPosition(nodes[idx])(dim);
  }

  template <class T>
  bool kdtree_get_bbox(T&) const {
    return false;
  }

  const SceneGraphLayer& layer;
  std::vector<NodeId> nodes;
};

struct NearestNodeFinder::Detail {
  using Dist = L2_Simple_Adaptor<double, GraphKdTreeAdaptor>;
  using KDTree = KDTreeSingleIndexAdaptor<Dist, GraphKdTreeAdaptor, 3, size_t>;

  Detail(const SceneGraphLayer& layer, const std::vector<NodeId>& nodes)
      : adaptor(layer, nodes) {
    kdtree.reset(new KDTree(3, adaptor));
    kdtree->buildIndex();
  }

  ~Detail() = default;

  GraphKdTreeAdaptor adaptor;
  std::unique_ptr<KDTree> kdtree;
};

NearestNodeFinder::NearestNodeFinder(const SceneGraphLayer& layer,
                                     const std::vector<NodeId>& nodes)
    : num_nodes(nodes.size()), internals_(new Detail(layer, nodes)) {}

NearestNodeFinder::NearestNodeFinder(const SceneGraphLayer& layer,
                                     const std::unordered_set<NodeId>& nodes)
    : num_nodes(nodes.size()) {
  std::vector<NodeId> node_vector(nodes.begin(), nodes.end());
  VLOG(10) << "Making node finder with " << nodes.size() << " nodes";
  internals_.reset(new Detail(layer, node_vector));
}

NearestNodeFinder::~NearestNodeFinder() {}

NearestNodeFinder::Ptr NearestNodeFinder::fromLayer(const SceneGraphLayer& layer,
                                                    const Filter& filter) {
  std::unordered_set<NodeId> layer_nodes;
  for (const auto& [node_id, node] : layer.nodes()) {
    if (filter(*node)) {
      layer_nodes.insert(node_id);
    }
  }

  if (layer_nodes.empty()) {
    return nullptr;
  }

  return std::make_unique<NearestNodeFinder>(layer, layer_nodes);
}

void NearestNodeFinder::find(const Eigen::Vector3d& position,
                             size_t num_to_find,
                             bool skip_first,
                             const NearestNodeFinder::Callback& callback) {
  const size_t limit = skip_first ? num_to_find + 1 : num_to_find;
  std::vector<size_t> nn_indices(limit);
  std::vector<double> distances(limit);

  size_t num_found = internals_->kdtree->knnSearch(
      position.data(), limit, nn_indices.data(), distances.data());

  size_t i = skip_first ? 1 : 0;
  for (; i < num_found; ++i) {
    callback(internals_->adaptor.nodes[nn_indices[i]], nn_indices[i], distances[i]);
  }
}

size_t NearestNodeFinder::findRadius(const Eigen::Vector3d& position,
                                     double radius,
                                     bool skip_first,
                                     const NearestNodeFinder::Callback& callback) {
  std::vector<nanoflann::ResultItem<size_t, double>> neighbors;
  size_t num_found = internals_->kdtree->radiusSearch(
      position.data(), radius, neighbors, nanoflann::SearchParameters());

  size_t i = skip_first ? 1 : 0;
  for (; i < num_found; ++i) {
    const auto idx = neighbors[i].first;
    callback(internals_->adaptor.nodes[idx], idx, neighbors[i].second);
  }

  if (num_found == 0) {
    return num_found;
  } else {
    return num_found - (skip_first ? 1 : 0);
  }
}

size_t makeSemanticNodeFinders(const SceneGraphLayer& layer,
                               SemanticNodeFinders& finders,
                               bool use_active) {
  std::map<SemanticLabel, std::unordered_set<NodeId>> label_node_map;
  size_t total = 0;
  for (const auto& id_node_pair : layer.nodes()) {
    auto& attrs = id_node_pair.second->attributes<SemanticNodeAttributes>();
    if (!use_active && attrs.is_active) {
      continue;
    }

    ++total;
    auto iter = label_node_map.find(attrs.semantic_label);
    if (iter == label_node_map.end()) {
      iter = label_node_map.insert({attrs.semantic_label, {}}).first;
    }

    iter->second.insert(id_node_pair.first);
  }

  // creating nodefinders
  finders.clear();
  for (const auto& label_ids_pair : label_node_map) {
    finders.emplace(label_ids_pair.first,
                    std::make_unique<NearestNodeFinder>(layer, label_ids_pair.second));
  }

  return total;
}

struct PointNeighborSearch::Detail {
  // Nanoflann interface.
  explicit Detail(const std::vector<Eigen::Vector3f>& points)
      : points_(points),
        tree_(3, *this, nanoflann::KDTreeSingleIndexAdaptorParams(10)) {
    tree_.buildIndex();
  }

  std::size_t kdtree_get_point_count() const { return points_.size(); }

  float kdtree_get_pt(const size_t idx, const size_t dim) const {
    if (dim == 0)
      return points_[idx].x();
    else if (dim == 1)
      return points_[idx].y();
    else
      return points_[idx].z();
  }

  template <class BBOX>
  bool kdtree_get_bbox(BBOX&) const {
    return false;
  }

  const std::vector<Eigen::Vector3f>& points_;
  KDTreeSingleIndexAdaptor<L2_Simple_Adaptor<float, Detail>, Detail, 3> tree_;
};

PointNeighborSearch::PointNeighborSearch(const std::vector<Eigen::Vector3f>& points) {
  internals_ = std::make_unique<Detail>(points);
}

PointNeighborSearch::~PointNeighborSearch(){};

bool PointNeighborSearch::search(const Eigen::Vector3f& query_point,
                                 float& distance_squared,
                                 size_t& index) const {
  nanoflann::KNNResultSet<float> resultSet(1);
  float query_point_arr[3] = {query_point.x(), query_point.y(), query_point.z()};
  resultSet.init(&index, &distance_squared);
  return internals_->tree_.findNeighbors(resultSet, &query_point_arr[0]);
}

}  // namespace hydra
