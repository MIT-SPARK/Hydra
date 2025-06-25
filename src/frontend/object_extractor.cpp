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
#include "hydra/frontend/object_extractor.h"

#include <config_utilities/config.h>
#include <config_utilities/types/enum.h>
#include <config_utilities/validation.h>
#include <glog/logging.h>
#include <glog/stl_logging.h>
#include <spark_dsg/bounding_box_extraction.h>
#include <spark_dsg/printing.h>

#include "hydra/utils/mesh_utilities.h"
#include "hydra/utils/timing_utilities.h"

namespace hydra {
namespace {

inline bool objectsMatch(const ObjectNodeAttributes& lhs,
                         const ObjectNodeAttributes& rhs) {
  return lhs.bounding_box.contains(rhs.position) ||
         rhs.bounding_box.contains(lhs.position);
}

inline bool objectsMatch(const ObjectNodeAttributes& attrs,
                         const MeshSegmenter::Cluster& cluster) {
  return attrs.bounding_box.contains(cluster.centroid);
}

inline void updateAttributes(const MeshSegmenter::Cluster& cluster,
                             uint64_t timestamp,
                             ObjectNodeAttributes& attrs) {
  attrs.last_update_time_ns = timestamp;
  attrs.is_active = true;

  updateObjectGeometry(*graph.mesh(), attrs);
}

ObjectNodeAttributes::Ptr createAttributes(const MeshSegmenter::Cluster& cluster,
                                           uint32_t label,
                                           uint64_t timestamp) {
  if (cluster.points.empty()) {
    LOG(ERROR) << "Encountered empty cluster with label" << static_cast<int>(label)
               << " @ " << timestamp << "[ns]";
    return nullptr;
  }

  auto attrs = std::make_unique<ObjectNodeAttributes>();
  attrs->last_update_time_ns = timestamp;
  attrs->is_active = true;
  attrs->semantic_label = label;

  updateObjectGeometry(*graph.mesh(), *attrs, nullptr, config.bounding_box_type);
  return attrs;
}

}  // namespace

using timing::ScopedTimer;

void declare_config(ObjectExtractor::Config& config) {
  using namespace config;
  name("MeshSegmenterConfig");
  field(config.layer_id, "layer_id");
  enum_field(config.bounding_box_type,
             "bounding_box_type",
             {{spark_dsg::BoundingBox::Type::INVALID, "INVALID"},
              {spark_dsg::BoundingBox::Type::AABB, "AABB"},
              {spark_dsg::BoundingBox::Type::OBB, "OBB"},
              {spark_dsg::BoundingBox::Type::RAABB, "RAABB"}});
  field(config.mesh_segmenter, "mesh_segmenter", false);
  field(config.sinks, "sinks");
}

// TODO(nathan) move node ID to not be here
ObjectExtractor::ObjectExtractor(const Config& config, const std::set<uint32_t>& labels)
    : config(config::checkValid(config)),
      sinks_(Sink::instantiate(config.sinks)),
      segmenter_(config.mesh_segmenter.with_labels(labels)),
      next_node_id_('O', 0) {
  for (const auto& label : labels) {
    active_nodes_[label] = std::map<NodeId, ObjectNodeAttributes::Ptr>();
  }
}

void ObjectExtractor::detect(uint64_t timestamp_ns, const VolumetricMap& map) {
  ScopedTimer timer("frontend/object_detection", timestamp_ns, true, 1, false);

  // clean up archived nodes and cleared points
  updateOldNodes(map);

  // detect and associate new clusters into the current objects
  const auto new_clusters = segmenter_.segment(map.getMeshLayer());
  for (const auto& [label, clusters] : new_clusters) {
    auto& active = active_nodes_.at(label);
    for (const auto& cluster : clusters) {
      bool matched = false;
      for (const auto& [node_id, attrs] : active) {
        if (objectsMatch(*attrs, cluster)) {
          matched = true;
          updateAttributes(cluster, timestamp_ns, *attrs);
          break;
        }
      }

      if (!matched) {
        active.emplace(next_node_id_, createAttributes(cluster, label, timestamp_ns));
        ++next_node_id_;
      }
    }
  }

  // resolve overlapping nodes
  mergeActiveNodes();

  Sink::callAll(sinks_, timestamp_ns, *this);
}

void ObjectExtractor::updateGraph(uint64_t timestamp_ns, DynamicSceneGraph& graph) {
  ScopedTimer timer("frontend/object_graph_update", timestamp_ns);
  for (const auto& node_id : removed_nodes_) {
    graph.removeNode(node_id);
  }

  removed_nodes_.clear();

  for (const auto& [label, nodes] : active_nodes_) {
    for (const auto& [node_id, attrs] : nodes) {
      graph.addOrUpdateNode(config.layer_id, node_id, attrs->clone());
    }
  }
}

void ObjectExtractor::updateOldNodes(const VolumetricMap& map) {
  for (auto& [label, label_nodes] : active_nodes_) {
    auto iter = label_nodes.begin();
    while (iter != label_nodes.end()) {
      const auto node_id = iter->first;
      auto& attrs = *iter->second;

      // remap and prune mesh connections
      VLOG(20) << "[Object Extractor] Updating node " << NodeSymbol(node_id).str();
      // TODO(nathan)
      const auto is_active = false;
      VLOG(20) << "[Object Extractor] After update: " << std::boolalpha << is_active;
      if (attrs.mesh_connections.size() < config.min_cluster_size) {
        removed_nodes_.insert(node_id);
        continue;
      }

      attrs.is_active = is_active;
      if (!attrs.is_active) {
        iter = label_nodes.erase(iter);
      } else {
        ++iter;
      }
    }
  }
}

void ObjectExtractor::mergeActiveNodes() {
  for (auto& [label, curr_active] : active_nodes_) {
    std::set<NodeId> merged_nodes;
    for (const auto& [node_id, attrs] : curr_active) {
      if (merged_nodes.count(node_id)) {
        continue;
      }

      std::list<NodeId> to_merge;
      for (const auto& [other_id, other_attrs] : curr_active) {
        if (node_id == other_id) {
          continue;
        }

        if (merged_nodes.count(other_id)) {
          continue;
        }

        if (objectsMatch(*attrs, *other_attrs)) {
          to_merge.push_back(other_id);
        }
      }

      for (const auto& other_id : to_merge) {
        merged_nodes.insert(other_id);
      }

      if (!to_merge.empty()) {
        updateObjectGeometry(*graph.mesh(), attrs);
      }
    }
  }
}

}  // namespace hydra
