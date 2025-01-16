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
#include "hydra/backend/update_places_functor.h"

#include <config_utilities/config.h>
#include <config_utilities/validation.h>
#include <glog/logging.h>
#include <gtsam/geometry/Pose3.h>
#include <hydra/common/global_info.h>
#include <kimera_pgmo/deformation_graph.h>
#include <spark_dsg/printing.h>

#include "hydra/utils/timing_utilities.h"

namespace hydra {

using timing::ScopedTimer;
using MergeId = std::optional<NodeId>;

void declare_config(UpdatePlacesFunctor::Config& config) {
  using namespace config;
  name("UpdatePlacesFunctor::Config");
  field(config.pos_threshold_m, "pos_threshold_m", "m");
  field(config.distance_tolerance_m, "distance_tolerance_m", "m");
  field(config.num_control_points, "num_control_points");
  field(config.control_point_tolerance_s, "control_point_tolerance_s", "s");
  field(config.merge_proposer, "merge_proposer");
}

struct AttributeMap {
  std::vector<NodeAttributes*> attributes;

  void push_back(NodeAttributes* attrs) { attributes.push_back(attrs); }

  void sort() {
    std::sort(
        attributes.begin(), attributes.end(), [](const auto& lhs, const auto& rhs) {
          return lhs->last_update_time_ns < rhs->last_update_time_ns;
        });
  }
};

size_t pgmoNumVertices(const AttributeMap& map) { return map.attributes.size(); }

kimera_pgmo::traits::Pos pgmoGetVertex(const AttributeMap& map,
                                       size_t i,
                                       kimera_pgmo::traits::VertexTraits* traits) {
  const auto& attrs = map.attributes[i];
  if (traits) {
    traits->stamp = attrs->last_update_time_ns;
  }

  return attrs->position.cast<float>();
}

uint64_t pgmoGetVertexStamp(const AttributeMap& map, size_t i) {
  return map.attributes[i]->last_update_time_ns;
}

void pgmoSetVertex(AttributeMap& map,
                   size_t i,
                   const kimera_pgmo::traits::Pos& pos,
                   const kimera_pgmo::traits::VertexTraits&) {
  map.attributes[i]->position = pos.cast<double>();
}

UpdatePlacesFunctor::UpdatePlacesFunctor(const Config& config)
    : config(config::checkValid(config)), merge_proposer(config.merge_proposer) {}

// drops any isolated place nodes that would cause an inderminate system error
void UpdatePlacesFunctor::filterMissing(DynamicSceneGraph& graph,
                                        const std::list<NodeId> missing_nodes) const {
  if (missing_nodes.empty()) {
    return;
  }

  VLOG(5) << "[Places Layer]: could not update "
          << displayNodeSymbolContainer(missing_nodes);

  for (const auto& node_id : missing_nodes) {
    if (!graph.hasNode(node_id)) {
      continue;
    }

    const auto& node = graph.getNode(node_id);
    if (!node.attributes().is_active && !node.hasSiblings()) {
      VLOG(2) << "[Places Layer]: removing node " << NodeSymbol(node_id).getLabel();
      graph.removeNode(node_id);
    }
  }
}

size_t UpdatePlacesFunctor::updateFromValues(const LayerView& view,
                                             SharedDsgInfo& dsg,
                                             const UpdateInfo::ConstPtr& info) const {
  if (!info->places_values) {
    return 0;
  }

  size_t num_changed = 0;
  const auto& places_values = *info->places_values;
  for (const auto& node : view) {
    ++num_changed;
    auto& attrs = node.attributes<PlaceNodeAttributes>();
    if (!places_values.exists(node.id)) {
      VLOG(10) << "[Hydra Backend] missing place " << NodeSymbol(node.id).getLabel()
               << " from places factors.";
      // this happens for the GT version
      continue;
    }

    // TODO(nathan) consider updating distance via parents + deformation graph
    attrs.position = places_values.at<gtsam::Pose3>(node.id).translation();
    dsg.graph->setNodeAttributes(node.id, attrs.clone());
  }

  // TODO(nathan) fix this
  // filterMissing(*dsg.graph, missing_nodes);
  return num_changed;
}

size_t UpdatePlacesFunctor::interpFromValues(const LayerView& view,
                                             SharedDsgInfo& dsg,
                                             const UpdateInfo::ConstPtr& info) const {
  if (!info->deformation_graph) {
    return 0;
  }

  const auto this_robot = GlobalInfo::instance().getRobotPrefix().id;

  std::map<char, AttributeMap> nodes;
  for (const auto& node : view) {
    size_t robot_id = this_robot;
    if (info->node_to_robot_id) {
      auto iter = info->node_to_robot_id->find(node.id);
      if (iter == info->node_to_robot_id->end()) {
        LOG(WARNING) << "Node " << NodeSymbol(node.id) << " does not belong to robot";
      } else {
        robot_id = iter->second;
      }
    }

    const auto prefix = kimera_pgmo::GetVertexPrefix(robot_id);
    auto robot_attrs = nodes.find(prefix);
    if (robot_attrs == nodes.end()) {
      robot_attrs = nodes.emplace(prefix, AttributeMap{}).first;
    }

    auto& attrs = node.attributes();
    auto cache_iter = cached_pos_.find(node.id);
    if (cache_iter == cached_pos_.end()) {
      cache_iter = cached_pos_.emplace(node.id, attrs.position).first;
    } else if (attrs.is_active) {
      // update cache if node is still active
      cache_iter->second = attrs.position;
    }

    // set the position of the node to the original position before deformation
    attrs.position = cache_iter->second;
    robot_attrs->second.push_back(&attrs);
  }

  auto& dgraph = *info->deformation_graph;
  for (auto& [prefix, attributes] : nodes) {
    if (!dgraph.hasVertexKey(prefix)) {
      continue;
    }

    const auto& control_points = dgraph.getInitialPositionsVertices(prefix);
    if (control_points.size() < config.num_control_points) {
      continue;
    }

    attributes.sort(); // make sure attributes are sorted by timestamp
    std::vector<std::set<size_t>> vertex_graph_map_deformed;
    kimera_pgmo::deformation::deformPoints(attributes,
                                           vertex_graph_map_deformed,
                                           attributes,
                                           prefix,
                                           control_points,
                                           dgraph.getVertexStamps(prefix),
                                           dgraph.getGtsamValues(),
                                           config.num_control_points,
                                           config.control_point_tolerance_s,
                                           nullptr);
  }

  for (const auto& node : view) {
    dsg.graph->setNodeAttributes(node.id, node.attributes().clone());
  }

  return 0;
}

void UpdatePlacesFunctor::call(const DynamicSceneGraph& unmerged,
                               SharedDsgInfo& dsg,
                               const UpdateInfo::ConstPtr& info) const {
  ScopedTimer spin_timer("backend/update_places", info->timestamp_ns);

  if (!unmerged.hasLayer(DsgLayers::PLACES)) {
    return;
  }

  const auto new_loopclosure = info->loop_closure_detected;
  const auto& places = unmerged.getLayer(DsgLayers::PLACES);
  active_tracker.clear();  // reset from previous pass
  const auto view = new_loopclosure ? LayerView(places) : active_tracker.view(places);

  size_t num_changed = 0;
  if (!info->places_values || info->places_values->size() == 0) {
    num_changed = interpFromValues(view, dsg, info);

    std::vector<NodeId> to_remove;
    for (const auto& [node_id, pos] : cached_pos_) {
      if (!unmerged.hasNode(node_id)) {
        to_remove.push_back(node_id);
      }
    }

    for (const auto& node_id : to_remove) {
      cached_pos_.erase(node_id);
    }
  } else {
    num_changed = updateFromValues(view, dsg, info);
  }

  VLOG(2) << "[Hydra Backend] Places update: " << num_changed << " nodes";
}

MergeList UpdatePlacesFunctor::findMerges(const DynamicSceneGraph& graph,
                                          const UpdateInfo::ConstPtr& info) const {
  const auto new_lcd = info->loop_closure_detected;
  const auto& places = graph.getLayer(DsgLayers::PLACES);
  // freeze layer view to avoid messing with tracker
  const auto view = new_lcd ? LayerView(places) : active_tracker.view(places, true);

  MergeList proposals;
  merge_proposer.findMerges(
      places,
      view,
      [this](const SceneGraphNode& lhs, const SceneGraphNode& rhs) {
        const auto& lhs_attrs = lhs.attributes<PlaceNodeAttributes>();
        const auto& rhs_attrs = rhs.attributes<PlaceNodeAttributes>();
        if (!lhs_attrs.real_place || !rhs_attrs.real_place) {
          return false;
        }

        const auto distance = (lhs_attrs.position - rhs_attrs.position).norm();
        if (distance > config.pos_threshold_m) {
          return false;
        }

        const auto radii_deviation = std::abs(lhs_attrs.distance - rhs_attrs.distance);
        return radii_deviation <= config.distance_tolerance_m;
      },
      proposals);
  return proposals;
}

}  // namespace hydra
