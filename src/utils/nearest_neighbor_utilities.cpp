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
    : internals_(new Detail(layer, nodes)) {}

NearestNodeFinder::NearestNodeFinder(const SceneGraphLayer& layer,
                                     const std::unordered_set<NodeId>& nodes) {
  std::vector<NodeId> node_vector(nodes.begin(), nodes.end());
  VLOG(10) << "Making node finder with " << nodes.size() << " nodes";
  internals_.reset(new Detail(layer, node_vector));
}

NearestNodeFinder::~NearestNodeFinder() {}

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

}  // namespace hydra
