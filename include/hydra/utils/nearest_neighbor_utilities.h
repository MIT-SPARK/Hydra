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
#pragma once
#include <memory>
#include <unordered_set>
#include <vector>

#include "hydra/common/dsg_types.h"

namespace hydra {

class NearestNodeFinder {
 public:
  using Callback = std::function<void(NodeId, size_t, double)>;
  using Filter = std::function<bool(const SceneGraphNode&)>;
  using Ptr = std::unique_ptr<NearestNodeFinder>;

  NearestNodeFinder(const SceneGraphLayer& layer, const std::vector<NodeId>& nodes);

  NearestNodeFinder(const SceneGraphLayer& layer,
                    const std::unordered_set<NodeId>& nodes);

  virtual ~NearestNodeFinder();

  static Ptr fromLayer(const SceneGraphLayer& layer, const Filter& filter);

  void find(const Eigen::Vector3d& position,
            size_t num_to_find,
            bool skip_first,
            const Callback& callback);

  size_t findRadius(const Eigen::Vector3d& position,
                    double radius_m,
                    bool skip_first,
                    const Callback& callback);

  const size_t num_nodes;

 private:
  struct Detail;
  std::unique_ptr<Detail> internals_;
};

using SemanticNodeFinders =
    std::map<SemanticNodeAttributes::Label, std::unique_ptr<NearestNodeFinder>>;

size_t makeSemanticNodeFinders(const SceneGraphLayer& layer,
                               SemanticNodeFinders& finders,
                               bool use_active = false);

/**
 * @brief Helper class to perform nearest neigbor search on a set of points using
 * KD-Trees.
 */
class PointNeighborSearch {
 public:
  explicit PointNeighborSearch(const std::vector<Eigen::Vector3f>& points);
  virtual ~PointNeighborSearch();

  // Lookup.
  /**
   * @brief Find the nearest neighbor of the query point.
   * @param query_point The query point.
   * @param distance_squared The output squared distance to the nearest neighbor.
   * @param index The output index of the nearest neighbor in the tree data.
   * @returns True if a nearest neighbor was found, false otherwise.
   */
  bool search(const Eigen::Vector3f& query_point,
              float& distance_squared,
              size_t& index) const;

 private:
  struct Detail;
  std::unique_ptr<Detail> internals_;
};

}  // namespace hydra
