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

#include <kimera_pgmo/mesh_delta.h>
#include <spark_dsg/bounding_box.h>
#include <spatial_hash/hash.h>

#include "hydra/backend/update_functions.h"

namespace hydra {

class MeshLabelClustering {
 public:
  struct BlockInfo {
    BlockInfo();
    BlockInfo(const Eigen::Vector3f& pos, float resolution);

    Eigen::VectorXi counts;
    spark_dsg::BoundingBox bbox;
  };

  using SpatialBlockInfoMap = spatial_hash::LongIndexHashMap<BlockInfo>;
  using BlockCallback =
      std::function<void(const spatial_hash::LongIndex&, const BlockInfo&)>;

  MeshLabelClustering(float resolution);

  void clear();

  template <typename Vertices>
  void update(const Vertices& vertices, const std::function<bool(size_t)>& is_active);

  void update(const kimera_pgmo::MeshDelta& delta);

  void processSeen(const BlockCallback& callback, bool clear_seen = true);

  void updateBlockTracking(MeshLabelClustering::SpatialBlockInfoMap& info_map,
                           const spatial_hash::LongIndex& index,
                           uint32_t label);

 private:
  BlockInfo infoFromIndex(const spatial_hash::LongIndex& idx) const;

  float resolution_;
  float index_scale_;

  SpatialBlockInfoMap archived_;
  SpatialBlockInfoMap active_;
  spatial_hash::LongIndexSet seen_;
};

template <typename Vertices>
void MeshLabelClustering::update(const Vertices& vertices,
                                 const std::function<bool(size_t)>& is_active) {
  active_.clear();

  const auto num_local = kimera_pgmo::traits::num_vertices(vertices);
  for (size_t i = 0; i < num_local; ++i) {
    kimera_pgmo::traits::VertexTraits traits;
    const auto pos = kimera_pgmo::traits::get_vertex(vertices, i, &traits);
    if (!traits.label) {
      continue;
    }

    const spatial_hash::LongIndex block_index(std::round(pos.x() * index_scale_),
                                              std::round(pos.y() * index_scale_),
                                              std::round(pos.z() * index_scale_));
    updateBlockTracking(is_active(i) ? active_ : archived_, block_index, *traits.label);
    seen_.insert(block_index);
  }
}

struct UpdateMeshClustersFunctor : public UpdateFunctor {
  struct Config {
    //! Spatial resolution for clustering
    float resolution = 0.5f;
    //! Layer ID to update
    spark_dsg::LayerId layer = 1;
    //! Layer name to update
    std::string layer_name = "MESH_CLUSTERS";
    //! Partition to update
    PartitionId partition = 0;
    //! Node symbol prefix
    char prefix = 'l';
  } const config;

  explicit UpdateMeshClustersFunctor(const Config& config);

  void call(const DynamicSceneGraph& unmerged,
            SharedDsgInfo& dsg,
            const UpdateInfo::ConstPtr& info) const override;

 private:
  mutable NodeSymbol next_node_id_;
  mutable MeshLabelClustering clustering_;
  mutable spatial_hash::LongIndexHashMap<NodeId> node_id_map_;
};

void declare_config(UpdateMeshClustersFunctor::Config& config);

}  // namespace hydra
