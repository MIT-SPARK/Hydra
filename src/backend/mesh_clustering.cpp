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

#include "hydra/backend/mesh_clustering.h"

#include <config_utilities/config.h>
#include <config_utilities/factory.h>
#include <config_utilities/validation.h>

#include "hydra/common/global_info.h"
#include "hydra/utils/pgmo_mesh_traits.h"
#include "hydra/utils/timing_utilities.h"

namespace hydra {
namespace {

const auto registration =
    config::RegistrationWithConfig<UpdateFunctor,
                                   UpdateMeshClustersFunctor,
                                   UpdateMeshClustersFunctor::Config>(
        "UpdateMeshClustersFunctor");

}

using timing::ScopedTimer;

struct ActiveMeshWrapper {
  ActiveMeshWrapper(const spark_dsg::Mesh& mesh, size_t start_idx)
      : mesh(mesh), start_idx(start_idx) {}

  const spark_dsg::Mesh& mesh;
  const size_t start_idx;
};

size_t pgmoNumVertices(const ActiveMeshWrapper& wrapper) {
  const auto total = kimera_pgmo::traits::num_vertices(wrapper.mesh);
  return total > wrapper.start_idx ? total - wrapper.start_idx : 0;
}

kimera_pgmo::traits::Pos pgmoGetVertex(const ActiveMeshWrapper& wrapper,
                                       size_t i,
                                       kimera_pgmo::traits::VertexTraits* traits) {
  return kimera_pgmo::traits::get_vertex(wrapper.mesh, i + wrapper.start_idx, traits);
}

using spatial_hash::LongIndex;
using BlockInfo = MeshLabelClustering::BlockInfo;

BlockInfo::BlockInfo()
    : counts(Eigen::VectorXi::Zero(GlobalInfo::instance().getTotalLabels())) {}

BlockInfo::BlockInfo(const Eigen::Vector3f& pos, float resolution)
    : counts(Eigen::VectorXi::Zero(GlobalInfo::instance().getTotalLabels())),
      bbox(Eigen::Vector3f::Constant(resolution), pos) {}

MeshLabelClustering::MeshLabelClustering(float resolution)
    : resolution_(resolution), index_scale_(1.0f / resolution) {}

void MeshLabelClustering::clear() {
  archived_.clear();
  active_.clear();
  seen_.clear();
}

void MeshLabelClustering::update(const kimera_pgmo::MeshDelta& delta) {
  update(delta, [&](size_t idx) { return idx >= delta.getNumArchivedVertices(); });
}

void MeshLabelClustering::processSeen(const BlockCallback& callback, bool clear_seen) {
  for (const auto& idx : seen_) {
    auto info = infoFromIndex(idx);
    auto archive_iter = archived_.find(idx);
    if (archive_iter != archived_.end()) {
      info.counts += archive_iter->second.counts;
    }

    auto active_iter = active_.find(idx);
    if (active_iter != active_.end()) {
      info.counts += active_iter->second.counts;
    }

    callback(idx, info);
  }

  if (clear_seen) {
    seen_.clear();
  }
}

BlockInfo MeshLabelClustering::infoFromIndex(const LongIndex& idx) const {
  const Eigen::Vector3f pos = resolution_ * idx.cast<float>();
  return {pos, resolution_};
}

void MeshLabelClustering::updateBlockTracking(SpatialBlockInfoMap& info_map,
                                              const LongIndex& index,
                                              uint32_t label) {
  auto iter = info_map.find(index);
  if (iter == info_map.end()) {
    iter = info_map.emplace(index, infoFromIndex(index)).first;
  }

  auto& info = iter->second;
  if (label > info.counts.size()) {
    LOG_FIRST_N(WARNING, 5) << "Found label " << label << " outside label space!";
    return;
  }

  info.counts(label) += 1;
}

UpdateMeshClustersFunctor::UpdateMeshClustersFunctor(const Config& config)
    : config(config::checkValid(config)),
      next_node_id_(config.prefix, 0),
      clustering_(config.resolution) {}

void UpdateMeshClustersFunctor::call(const DynamicSceneGraph&,
                                     SharedDsgInfo& dsg,
                                     const UpdateInfo::ConstPtr& info) const {
  auto mesh = dsg.graph->mesh();
  if (!mesh) {
    LOG(WARNING) << "No mesh to cluster!";
    return;
  }

  ScopedTimer spin_timer("backend/cluster_mesh", info ? info->timestamp_ns : 0);

  // set layer name
  dsg.graph->addLayer(config.layer, config.partition, config.layer_name);

  // TODO(nathan) fix this partial processing
  const auto num_archived = info ? info->num_archived_vertices : 0;
  const auto num_previous = info ? info->num_previous_archived_vertices : 0;
  if (info && info->loop_closure_detected) {
    clustering_.clear();
    clustering_.update(*mesh, [num_archived](size_t i) { return i > num_archived; });
  } else {
    ActiveMeshWrapper wrapper(*mesh, num_previous);
    clustering_.update(wrapper,
                       [&](size_t i) { return i + num_previous > num_archived; });
  }

  clustering_.processSeen(
      [&](const LongIndex& idx, const BlockInfo& info) {
        auto iter = node_id_map_.find(idx);
        if (iter == node_id_map_.end()) {
          iter = node_id_map_.emplace(idx, next_node_id_).first;
          ++next_node_id_;
        }

        auto attrs = std::make_unique<SemanticNodeAttributes>();
        attrs->position = info.bbox.world_P_center.cast<double>();
        attrs->bounding_box = info.bbox;
        attrs->semantic_feature = info.counts.cast<float>();
        dsg.graph->addOrUpdateNode(
            config.layer, iter->second, std::move(attrs), config.partition);
      },
      true);
}

void declare_config(UpdateMeshClustersFunctor::Config& config) {
  using namespace config;
  name("UpdateMeshClustersFunctor::Config");
  field(config.resolution, "resolution");
  field(config.layer, "layer");
  field(config.layer_name, "layer_name");
  field(config.partition, "partition");
  check(config.resolution, GT, 0.0, "resolution");
}

}  // namespace hydra
