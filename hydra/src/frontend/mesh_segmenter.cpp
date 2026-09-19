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
#include <numeric>
#include <tuple>
#include <unordered_map>

#include "hydra/common/global_info.h"
#include "hydra/frontend/mesh_clustering.h"
#include "hydra/frontend/mesh_sample_grouping.h"
#include "hydra/utils/mesh_utilities.h"
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
  checkCondition(std::isfinite(config.association_tolerance),
                 "finite association tolerance");
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
                         SharedDsgInfo& dsg,
                         FrontendOutput&,
                         const VolumetricWindow*) {
  update(msg);
  std::lock_guard<std::mutex> lock(dsg.mutex);
  updateNodes(*dsg.graph);
}

void MeshSegmenter::callMeshUpdate(SharedDsgInfo& dsg,
                                   FrontendOutput& output,
                                   const MeshUpdateInfo& info) {
  timing::ScopedTimer timer("object/connections", output.timestamp_ns);
  if (!info.correspondence) {
    throw std::logic_error("Missing compressor correspondence");
  }
  for (auto it = tracked_nodes_.begin(); it != tracked_nodes_.end();) {
    auto& attrs = dsg.graph->getNode(*it).attributes<spark_dsg::ObjectNodeAttributes>();
    info.offsets.remapVertexIndices(attrs.mesh_connections);
    const auto object = objects_.find(*it);
    if (object != objects_.end() && object->second.has_active_support) {
      updateConnections(object->second, info, attrs);
    }
    attrs.mesh_connections.sort();
    attrs.mesh_connections.unique();
    attrs.is_active = !attrs.mesh_connections.empty() &&
                      attrs.mesh_connections.back() >= info.offsets.archived_vertices;
    if (!attrs.is_active &&
        (object == objects_.end() || !object->second.has_active_support)) {
      it = tracked_nodes_.erase(it);
    } else {
      ++it;
    }
  }
}

void MeshSegmenter::updateConnections(const Object& object,
                                      const MeshUpdateInfo& info,
                                      spark_dsg::ObjectNodeAttributes& attrs) const {
  const auto& correspondence = *info.correspondence;
  std::set<size_t> current;
  std::set<size_t> history;
  const auto retain = [&](const Eigen::Vector3f& point) {
    const auto it = correspondence.retained.find(correspondence.grid.toIndex(point));
    if (it == correspondence.retained.end()) {
      return;
    }
    for (const auto index : it->second) {
      history.insert(info.offsets.toGlobalVertex(index));
    }
  };
  for (const auto& ref : object.vertices) {
    const auto index = correspondence.find(ref.block->pos(ref.vertex));
    if (index) {
      current.insert(info.offsets.toGlobalVertex(*index));
    } else {
      // A frozen vertex can still support an unchanged block. Preserve an
      // existing connection without assigning it to a newly detected object.
      retain(ref.block->pos(ref.vertex));
    }
  }

  for (const auto& point : object.archived_points) {
    retain(point);
  }
  attrs.mesh_connections.remove_if([&](size_t index) {
    return index >= info.offsets.prev_archived_vertices && !history.count(index) &&
           !current.count(index);
  });
  attrs.mesh_connections.insert(
      attrs.mesh_connections.end(), current.begin(), current.end());
}

void MeshSegmenter::updateGeometry(const Object& object,
                                   spark_dsg::ObjectNodeAttributes& attrs) const {
  spark_dsg::Mesh samples;
  MeshSampleGrouping deduplicator(config.vertex_merge_tolerance_m);
  const auto append = [&](const auto& points) {
    for (const auto& point : points) {
      if (deduplicator.add(point) == samples.numVertices()) {
        samples.points.push_back(point);
      }
    }
  };
  append(object.archived_points);
  append(object.points);
  std::vector<size_t> indices(samples.numVertices());
  std::iota(indices.begin(), indices.end(), 0);
  updateObjectGeometry(samples, attrs, &indices, config.bounding_box_type);
}

void MeshSegmenter::updateNodes(spark_dsg::SceneGraph& graph) {
  for (const auto& [from, into] : merges_) {
    if (graph.hasNode(from) && graph.hasNode(into)) {
      const auto& source =
          graph.getNode(from).attributes<spark_dsg::ObjectNodeAttributes>();
      auto& target = graph.getNode(into).attributes<spark_dsg::ObjectNodeAttributes>();
      target.mesh_connections.insert(target.mesh_connections.end(),
                                     source.mesh_connections.begin(),
                                     source.mesh_connections.end());
    }
    graph.removeNode(from);
    tracked_nodes_.erase(from);
  }
  for (const auto& [id, object] : objects_) {
    if (!object.has_active_support && object.archived_points.empty()) {
      graph.removeNode(id);
      tracked_nodes_.erase(id);
      continue;
    }
    if (!graph.hasNode(id)) {
      auto attrs = std::make_unique<spark_dsg::ObjectNodeAttributes>();
      attrs->semantic_label = object.label;
      graph.emplaceNode(config.layer_id, id, std::move(attrs));
      tracked_nodes_.insert(id);
    }
    auto& attrs = graph.getNode(id).attributes<spark_dsg::ObjectNodeAttributes>();
    attrs.last_update_time_ns = object.timestamp_ns;
    updateGeometry(object, attrs);
  }
}

void MeshSegmenter::update(const ActiveWindowOutput& input) {
  merges_.clear();
  // Finalized objects are owned by the graph after the previous connection pass.
  for (auto it = objects_.begin(); it != objects_.end();) {
    if (!it->second.has_active_support) {
      it = objects_.erase(it);
    } else {
      ++it;
    }
  }

  timing::ScopedTimer timer("object/detection", input.timestamp_ns);
  archiveBlocks(input.archived);
  for (const auto& block : input.map().getMeshLayer()) {
    blocks_[block.index] = input.map().getMeshLayer().getBlockPtr(block.index);
  }
  cluster(input.timestamp_ns);
}

void MeshSegmenter::archiveBlocks(const spatial_hash::BlockIndices& indices) {
  if (indices.empty()) {
    return;
  }

  const spatial_hash::IndexSet archived(indices.begin(), indices.end());
  for (auto& [id, object] : objects_) {
    if (!object.has_active_support) {
      continue;
    }

    MeshSampleGrouping history(config.vertex_merge_tolerance_m);
    for (const auto& point : object.archived_points) {
      history.add(point);
    }
    MeshSampleGrouping active(config.vertex_merge_tolerance_m);
    std::vector<ObjectMeshVertex> remaining;
    object.points.clear();
    for (const auto& ref : object.vertices) {
      const auto& point = ref.block->pos(ref.vertex);
      if (archived.count(ref.block->index)) {
        if (history.add(point) == object.archived_points.size()) {
          object.archived_points.push_back(point);
        }
      } else {
        remaining.push_back(ref);
        if (active.add(point) == object.points.size()) {
          object.points.push_back(point);
        }
      }
    }
    object.vertices = std::move(remaining);
    // Archive before replacements, including reentry at the same block index.
    object.has_active_support = !object.vertices.empty();
  }
  for (const auto& index : indices) {
    blocks_.erase(index);
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
  MeshSampleGrouping deduplicator(config.vertex_merge_tolerance_m);
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
      groups[index].push_back({block, i});
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

auto MeshSegmenter::findMatches(const Detection& detection) const
    -> std::vector<Match> {
  const auto& [mesh, sources, labels, clusters, cluster_labels] = detection;
  struct PreviousPoint {
    spark_dsg::NodeId id;
    Eigen::Vector3f point;
    uint32_t label;
  };
  std::unordered_map<Cell, std::vector<PreviousPoint>, CellHash> old_grid;
  for (const auto& [id, object] : objects_) {
    if (!object.has_active_support) {
      continue;
    }
    for (const auto& p : object.points) {
      old_grid[cellFor(p, config.association_tolerance)].push_back(
          {id, p, object.label});
    }
  }
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
      const auto score = static_cast<double>(count) / clusters[c].indices.size();
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
  return matches;
}

void MeshSegmenter::associate(uint64_t timestamp_ns, const Detection& detection) {
  const auto& [mesh, sources, labels, clusters, cluster_labels] = detection;
  const auto matches = findMatches(detection);
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
    MeshSampleGrouping history(config.vertex_merge_tolerance_m);
    for (const auto& point : target.archived_points) {
      history.add(point);
    }
    for (const auto& point : from.archived_points) {
      if (history.add(point) == target.archived_points.size()) {
        target.archived_points.push_back(point);
      }
    }
    merges_.emplace_back(match.id, into);
    objects_.erase(match.id);
    used.insert(match.id);
  }

  for (auto& [id, object] : objects_) {
    if (object.has_active_support && !used.count(id)) {
      object.has_active_support = false;
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
    object.has_active_support = true;
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
    if (object.has_active_support) {
      result.insert(id);
    }
  }
  return result;
}
}  // namespace hydra
