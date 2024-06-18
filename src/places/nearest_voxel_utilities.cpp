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
#include "hydra/places/nearest_voxel_utilities.h"

#include <nanoflann.hpp>

#include "hydra/places/graph_extractor_utilities.h"

namespace hydra::places {

using nanoflann::KDTreeSingleIndexAdaptor;
using nanoflann::KDTreeSingleIndexDynamicAdaptor;
using nanoflann::L2_Simple_Adaptor;

struct VoxelKdTreeAdaptor {
  explicit VoxelKdTreeAdaptor(const GlobalIndices& indices) : indices(indices) {}

  inline size_t kdtree_get_point_count() const { return indices.size(); }

  inline GlobalIndex::Scalar kdtree_get_pt(const size_t idx, const size_t dim) const {
    return indices[idx](dim);
  }

  template <class T>
  bool kdtree_get_bbox(T&) const {
    return false;
  }

  GlobalIndices indices;
};

struct NearestVoxelFinder::Detail {
  using Dist = L2_Simple_Adaptor<GlobalIndex::Scalar, VoxelKdTreeAdaptor>;
  using KDTree = KDTreeSingleIndexAdaptor<Dist, VoxelKdTreeAdaptor, 3, size_t>;

  explicit Detail(const GlobalIndices& indices) : adaptor(indices) {
    kdtree.reset(new KDTree(3, adaptor));
    kdtree->buildIndex();
  }

  ~Detail() = default;

  VoxelKdTreeAdaptor adaptor;
  std::unique_ptr<KDTree> kdtree;
};

NearestVoxelFinder::NearestVoxelFinder(const GlobalIndices& indices)
    : internals_(new Detail(indices)) {}

NearestVoxelFinder::~NearestVoxelFinder() {}

void NearestVoxelFinder::find(const GlobalIndex& index,
                              size_t num_to_find,
                              const NearestVoxelFinder::Callback& callback) {
  std::vector<size_t> nn_indices(num_to_find);
  std::vector<GlobalIndex::Scalar> distances(num_to_find);


  size_t num_found = internals_->kdtree->knnSearch(index.data(),
                                                   num_to_find,
                                                   nn_indices.data(),
                                                   distances.data());

  for (size_t i = 0; i < num_found; ++i) {
    callback(internals_->adaptor.indices[nn_indices[i]], nn_indices[i], distances[i]);
  }
}

FurthestIndexResult findFurthestIndexFromLine(const GlobalIndices& indices,
                                              const GlobalIndex& start,
                                              const GlobalIndex& end,
                                              size_t number_source_edges) {
  FurthestIndexResult result;

  GlobalIndices line = makeBresenhamLine(start, end);
  if (line.empty()) {
    return result;
  }

  NearestVoxelFinder nearest_voxel_finder(indices);
  for (const auto& line_idx : line) {
    nearest_voxel_finder.find(
        line_idx, 1, [&](const GlobalIndex& index, size_t nn_index, GlobalIndex::Scalar distance) {
          if (distance > result.distance || !result.valid) {
            result.valid = true;
            result.distance = distance;
            result.from_source = nn_index < number_source_edges;
            result.index = index;
          }
        });
  }

  return result;
}

}  // namespace hydra::places
