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
#include "hydra/frontend/place_mesh_connector.h"

#include <glog/logging.h>
#include <glog/stl_logging.h>

#include <nanoflann.hpp>

namespace hydra {

using nanoflann::KDTreeSingleIndexAdaptor;
using nanoflann::L2_Simple_Adaptor;

struct MeshDeltaAdaptor {
  MeshDeltaAdaptor(const kimera_pgmo::MeshDelta& delta)
      : delta(delta), indices(*delta.getActiveIndices()) {}

  inline size_t kdtree_get_point_count() const { return indices.size(); }

  inline double kdtree_get_pt(const size_t idx, const size_t dim) const {
    const auto cloud_idx = delta.getLocalIndex(indices.at(idx));
    CHECK_LT(cloud_idx, delta.vertex_updates->size());
    const auto& p = delta.vertex_updates->at(cloud_idx);
    return dim == 0 ? p.x : (dim == 1 ? p.y : p.z);
  }

  template <class T>
  bool kdtree_get_bbox(T&) const {
    return false;
  }

  const kimera_pgmo::MeshDelta& delta;
  pcl::Indices indices;
};

struct PlaceMeshConnector::Detail {
  using Dist = L2_Simple_Adaptor<double, MeshDeltaAdaptor>;
  using KDTree = KDTreeSingleIndexAdaptor<Dist, MeshDeltaAdaptor, 3, size_t>;

  Detail(const kimera_pgmo::MeshDelta& delta) : adaptor(delta) {
    kdtree.reset(new KDTree(3, adaptor));
    kdtree->buildIndex();
  }

  ~Detail() = default;

  std::optional<size_t> find(const Eigen::Vector3d& position) const {
    size_t index;
    double distance;
    size_t num_found = kdtree->knnSearch(position.data(), 1, &index, &distance);
    if (!num_found) {
      return std::nullopt;
    }

    return adaptor.indices.at(index);
  }

  MeshDeltaAdaptor adaptor;
  std::unique_ptr<KDTree> kdtree;
};

PlaceMeshConnector::PlaceMeshConnector(const kimera_pgmo::MeshDelta::Ptr& delta)
    : delta_(delta), internals_(new Detail(*CHECK_NOTNULL(delta))) {}

PlaceMeshConnector::~PlaceMeshConnector() {}

size_t PlaceMeshConnector::addConnections(const SceneGraphLayer& places,
                                          const DeformationMapping& mapping) const {
  const auto has_labels = delta_->hasSemantics();

  size_t num_missing = 0;
  for (const auto& id_node_pair : places.nodes()) {
    auto& attrs = id_node_pair.second->attributes<PlaceNodeAttributes>();
    // TODO(nathan) archive logic should live here if we actually track mesh vertices
    if (!attrs.is_active) {
      continue;
    }

    attrs.deformation_connections.clear();
    attrs.pcl_mesh_connections.clear();
    attrs.mesh_vertex_labels.clear();

    if (attrs.voxblox_mesh_connections.empty()) {
      ++num_missing;
      continue;
    }

    for (auto& vertex : attrs.voxblox_mesh_connections) {
      const Eigen::Vector3d pos = Eigen::Map<const Eigen::Vector3d>(vertex.voxel_pos);
      const auto nearest = internals_->find(pos);
      if (!nearest) {
        continue;
      }

      const auto local_idx = delta_->getLocalIndex(*nearest);
      // assign mesh vertex to relevant fields
      vertex.vertex = *nearest;
      attrs.pcl_mesh_connections.push_back(*nearest);

      // assign (potentially valid) deformation connection
      attrs.deformation_connections.push_back(mapping.at(local_idx));

      if (has_labels) {
        const auto label = delta_->semantic_updates.at(local_idx);
        attrs.mesh_vertex_labels.push_back(label);
        vertex.label = label;
      }
    }
  }

  return num_missing;
}

}  // namespace hydra
