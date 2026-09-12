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
#include "hydra/frontend/mesh_segmenter.h"

#include <config_utilities/config.h>
#include <config_utilities/types/conversions.h>
#include <config_utilities/types/enum.h>
#include <config_utilities/validation.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <tuple>
#include <unordered_map>

#include "hydra/common/global_info.h"
#include "hydra/frontend/mesh_clustering.h"
#include "hydra/frontend/mesh_connection_updater.h"
#include "hydra/utils/mesh_deduplication.h"
#include "hydra/utils/timing_utilities.h"

namespace hydra {
namespace {
const auto registration =
    config::RegistrationWithConfig<GraphBuilderFunctor,
                                   MeshSegmenter,
                                   MeshSegmenter::Config>("MeshSegmenter");
using Cell = std::array<int64_t, 3>;
struct CellHash {
  size_t operator()(const Cell& key) const {
    size_t result = 0;
    for (const auto v : key) {
      result ^= std::hash<int64_t>{}(v) + 0x9e3779b9 + (result << 6) + (result >> 2);
    }
    return result;
  }
};
Cell cellFor(const Eigen::Vector3f& point, double resolution) {
  return {static_cast<int64_t>(std::floor(point.x() / resolution)),
          static_cast<int64_t>(std::floor(point.y() / resolution)),
          static_cast<int64_t>(std::floor(point.z() / resolution))};
}
template <typename Callback>
void neighbors(const Cell& cell, const Callback& callback) {
  for (int x = -1; x <= 1; ++x) {
    for (int y = -1; y <= 1; ++y) {
      for (int z = -1; z <= 1; ++z) {
        callback(Cell{cell[0] + x, cell[1] + y, cell[2] + z});
      }
    }
  }
}
}  // namespace

void declare_config(MeshSegmenter::Config& config) {
  using namespace config;
  name("MeshSegmenterConfig");
  field(config.layer_id, "layer_id");
  field(config.clustering, "clustering", false);
  field(config.association_tolerance, "association_tolerance");
  field(config.vertex_merge_tolerance_m, "vertex_merge_tolerance_m", "m");
  check(config.vertex_merge_tolerance_m, GE, 0.0, "vertex_merge_tolerance_m");
  checkCondition(std::isfinite(config.vertex_merge_tolerance_m),
                 "finite merge tolerance");
  field(config.min_overlap_ratio, "min_overlap_ratio");
  check(config.association_tolerance, GT, 0.0, "association_tolerance");
  check(config.min_overlap_ratio, GT, 0.0, "min_overlap_ratio");
  check(config.min_overlap_ratio, LE, 1.0, "min_overlap_ratio");
  enum_field(config.bounding_box_type,
             "bounding_box_type",
             {{spark_dsg::BoundingBox::Type::INVALID, "INVALID"},
              {spark_dsg::BoundingBox::Type::AABB, "AABB"},
              {spark_dsg::BoundingBox::Type::OBB, "OBB"},
              {spark_dsg::BoundingBox::Type::RAABB, "RAABB"}});
  field(config.sinks, "sinks");
}

MeshSegmenter::MeshSegmenter(const Config& config, const std::set<uint32_t>& labels)
    : config(config::checkValid(config)),
      next_node_id_('O', 0),
      labels_(labels),
      sinks_(Sink::instantiate(config.sinks)) {}

MeshSegmenter::MeshSegmenter(const Config& config)
    : MeshSegmenter(config, GlobalInfo::instance().labelspace().object_labels) {}

void MeshSegmenter::call(const ActiveWindowOutput& msg,
                         SharedDsgInfo&,
                         FrontendOutput&,
                         const VolumetricWindow*) {
  update(msg);
}

void MeshSegmenter::callPostUpdate(SharedDsgInfo& dsg,
                                   FrontendOutput& output,
                                   const MeshUpdateInfo& info) {
  timing::ScopedTimer timer("object/connections", output.timestamp_ns);
  connections_.updateObjects(*this, info, *dsg.graph);
}

void MeshSegmenter::update(const ActiveWindowOutput& input) {
  merges_.clear();
  retired_blocks_.clear();
  // Finalized objects are owned by the graph after the previous connection pass.
  for (auto it = objects_.begin(); it != objects_.end();) {
    if (!it->second.is_active) {
      it = objects_.erase(it);
    } else {
      it->second.archived_vertices.clear();
      ++it;
    }
  }

  {
    timing::ScopedTimer timer("object/detection", input.timestamp_ns);
    std::unordered_set<const MeshBlock*> archived;
    for (const auto& index : input.archived) {
      const auto it = blocks_.find(index);
      if (it != blocks_.end()) {
        archived.insert(it->second.get());
        retired_blocks_.push_back(it->second);
      }
    }
    for (auto& [id, object] : objects_) {
      if (!object.is_active || archived.empty()) {
        continue;
      }
      MeshVertexDeduplicator points(config.vertex_merge_tolerance_m);
      std::vector<ObjectMeshVertex> remaining;
      object.points.clear();
      for (const auto& ref : object.vertices) {
        if (archived.count(ref.block)) {
          object.archived_vertices.push_back(ref);
          object.has_archived = true;
        } else {
          remaining.push_back(ref);
          const auto& p = ref.block->pos(ref.vertex);
          if (points.add(p) == object.points.size()) {
            object.points.push_back(p);
          }
        }
      }
      object.vertices = std::move(remaining);
      // Do this before replacement blocks are considered, including reentry at
      // the same coordinates in this packet.
      if (object.vertices.empty()) {
        object.is_active = false;
      }
    }
    for (const auto& index : input.archived) {
      blocks_.erase(index);
    }
    for (const auto& block : input.map().getMeshLayer()) {
      blocks_[block.index] = input.map().getMeshLayer().getBlockPtr(block.index);
    }
    cluster(input.timestamp_ns);
  }
}

struct MeshSegmenter::Detection {
  spark_dsg::Mesh mesh{true, false, true, false};
  std::vector<std::vector<ObjectMeshVertex>> sources;
  LabelIndices labels;
  std::vector<Cluster> clusters;
  std::vector<uint32_t> cluster_labels;
};

MeshSegmenter::Detection MeshSegmenter::prepareSamples() const {
  // Visit blocks in a fixed order so representative selection does not depend
  // on hash-map iteration or the order in which updated blocks arrived.
  std::vector<MeshBlock::ConstPtr> blocks;
  for (const auto& [index, block] : blocks_) {
    blocks.push_back(block);
  }
  std::sort(blocks.begin(), blocks.end(), [](const auto& a, const auto& b) {
    return std::array<int, 3>{a->index.x(), a->index.y(), a->index.z()} <
           std::array<int, 3>{b->index.x(), b->index.y(), b->index.z()};
  });
  MeshVertexDeduplicator deduplicator(config.vertex_merge_tolerance_m);
  std::vector<std::vector<ObjectMeshVertex>> groups;
  for (const auto& block : blocks) {
    if (!block->has_labels) {
      continue;
    }
    for (size_t i = 0; i < block->numVertices(); ++i) {
      const auto& point = block->pos(i);
      const auto label = block->label(i);
      if (!point.allFinite() || !labels_.count(label)) {
        continue;
      }
      const auto index = deduplicator.add(point, label);
      if (index == groups.size()) {
        groups.emplace_back();
      }
      // Keep every source reference for compression mapping and archival.
      groups[index].push_back({block.get(), i});
    }
  }
  const auto key = [](const auto& refs) {
    const auto& ref = refs.front();
    const auto& point = ref.block->pos(ref.vertex);
    return std::tuple{ref.block->label(ref.vertex), point.x(), point.y(), point.z()};
  };
  std::sort(groups.begin(), groups.end(), [&](const auto& a, const auto& b) {
    return key(a) < key(b);
  });

  Detection detection;
  auto& [mesh, sources, labels, clusters, cluster_labels] = detection;
  mesh.resizeVertices(groups.size());
  for (size_t i = 0; i < groups.size(); ++i) {
    const auto& ref = groups[i].front();
    const auto label = ref.block->label(ref.vertex);
    mesh.setPos(i, ref.block->pos(ref.vertex));
    mesh.setLabel(i, label);
    mesh.setColor(i, ref.block->color(ref.vertex));
    labels[label].push_back(i);
  }
  sources = std::move(groups);

  return detection;
}

void MeshSegmenter::cluster(uint64_t timestamp_ns) {
  timing::ScopedTimer timer("object/preparation", timestamp_ns);
  auto detection = prepareSamples();
  auto& [mesh, sources, labels, clusters, cluster_labels] = detection;
  timer.reset("object/clustering");
  LabelClusters display;
  auto extraction = config.clustering;
  // Existing objects may retain a small active fragment; creation is filtered below.
  extraction.min_cluster_size = 1;
  for (const auto& [label, indices] : labels) {
    const auto components = clustering::findClusters(extraction, mesh, indices);
    for (const auto& component : components) {
      Cluster cluster;
      cluster.indices = component;
      for (const auto i : component) {
        cluster.centroid += mesh.pos(i).cast<double>();
      }
      cluster.centroid /= component.size();
      display[label].push_back(cluster);
      clusters.push_back(std::move(cluster));
      cluster_labels.push_back(label);
    }
  }
  timer.reset("object/sinks");
  Sink::callAll(sinks_, timestamp_ns, mesh, labels, display);

  timer.reset("object/association");
  associate(timestamp_ns, detection);
}

void MeshSegmenter::associate(uint64_t timestamp_ns, const Detection& detection) {
  const auto& [mesh, sources, labels, clusters, cluster_labels] = detection;
  struct PreviousPoint {
    spark_dsg::NodeId id;
    Eigen::Vector3f point;
    uint32_t label;
  };
  std::unordered_map<Cell, std::vector<PreviousPoint>, CellHash> old_grid;
  for (const auto& [id, object] : objects_) {
    if (!object.is_active) {
      continue;
    }
    for (const auto& p : object.points) {
      old_grid[cellFor(p, config.association_tolerance)].push_back(
          {id, p, object.label});
    }
  }
  struct Match {
    double score;
    size_t cluster;
    spark_dsg::NodeId id;
  };
  std::vector<Match> matches;
  const auto association_squared =
      config.association_tolerance * config.association_tolerance;
  for (size_t c = 0; c < clusters.size(); ++c) {
    std::map<spark_dsg::NodeId, size_t> overlaps;
    for (const auto i : clusters[c].indices) {
      std::set<spark_dsg::NodeId> seen;
      neighbors(cellFor(mesh.pos(i), config.association_tolerance),
                [&](const Cell& cell) {
                  const auto it = old_grid.find(cell);
                  if (it == old_grid.end()) {
                    return;
                  }
                  for (const auto& old : it->second) {
                    if (old.label == cluster_labels[c] &&
                        (mesh.pos(i) - old.point).cast<double>().squaredNorm() <=
                            association_squared) {
                      seen.insert(old.id);
                    }
                  }
                });
      for (const auto id : seen) {
        ++overlaps[id];
      }
    }
    for (const auto& [id, count] : overlaps) {
      const double score = static_cast<double>(count) / clusters[c].indices.size();
      if (score >= config.min_overlap_ratio) {
        matches.push_back({score, c, id});
      }
    }
  }
  std::sort(matches.begin(), matches.end(), [](const auto& a, const auto& b) {
    if (a.score != b.score) {
      return a.score > b.score;
    }
    if (a.id != b.id) {
      return a.id < b.id;
    }
    return a.cluster < b.cluster;
  });
  std::map<size_t, spark_dsg::NodeId> assigned;
  std::set<spark_dsg::NodeId> used;
  for (const auto& match : matches) {
    if (!assigned.count(match.cluster) && used.insert(match.id).second) {
      assigned[match.cluster] = match.id;
    }
  }
  // A previous object joining an already assigned component is merged only if
  // it was not assigned to a different child in this update.
  for (const auto& match : matches) {
    if (used.count(match.id) || !assigned.count(match.cluster)) {
      continue;
    }
    const auto into = assigned.at(match.cluster);
    auto& target = objects_.at(into);
    auto& from = objects_.at(match.id);
    target.has_archived |= from.has_archived;
    target.archived_vertices.insert(target.archived_vertices.end(),
                                    from.archived_vertices.begin(),
                                    from.archived_vertices.end());
    merges_.emplace_back(match.id, into);
    objects_.erase(match.id);
    used.insert(match.id);
  }

  for (auto& [id, object] : objects_) {
    if (object.is_active && !used.count(id)) {
      object.is_active = false;
      object.vertices.clear();
      object.points.clear();
    }
  }
  for (size_t c = 0; c < clusters.size(); ++c) {
    auto it = assigned.find(c);
    if (it == assigned.end()) {
      if (clusters[c].indices.size() < config.clustering.min_cluster_size) {
        continue;
      }
      const spark_dsg::NodeId id = next_node_id_;
      ++next_node_id_;
      Object object;
      object.id = id;
      object.label = cluster_labels[c];
      objects_.emplace(id, std::move(object));
      it = assigned.emplace(c, id).first;
    }
    auto& object = objects_.at(it->second);
    object.timestamp_ns = timestamp_ns;
    object.is_active = true;
    object.vertices.clear();
    object.points.clear();
    for (const auto i : clusters[c].indices) {
      object.points.push_back(mesh.pos(i));
      object.vertices.insert(
          object.vertices.end(), sources[i].begin(), sources[i].end());
    }
  }
}

std::unordered_set<spark_dsg::NodeId> MeshSegmenter::getActiveNodes() const {
  std::unordered_set<spark_dsg::NodeId> result;
  for (const auto& [id, object] : objects_) {
    if (object.is_active) {
      result.insert(id);
    }
  }
  return result;
}
}  // namespace hydra
