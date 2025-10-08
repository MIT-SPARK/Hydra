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

#include "hydra/backend/deformation_interpolator.h"

#include <config_utilities/config.h>
#include <config_utilities/validation.h>
#include <glog/logging.h>
#include <hydra/common/global_info.h>
#include <kimera_pgmo/deformation_graph.h>
#include <spark_dsg/node_symbol.h>

namespace hydra {

using spark_dsg::NodeSymbol;

void declare_config(DeformationInterpolator::Config& config) {
  using namespace config;
  name("DeformationInterpolator::Config");
  field(config.num_control_points, "num_control_points");
  field(config.control_point_tolerance_s, "control_point_tolerance_s", "s");
  check(config.num_control_points, GT, 0, "num_control_points");
  check(config.control_point_tolerance_s, GE, 0.0, "control_point_tolerance_s");
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

DeformationInterpolator::DeformationInterpolator(const Config& config)
    : config(config::checkValid(config)) {}

void DeformationInterpolator::interpolateNodePositions(
    const DynamicSceneGraph& unmerged,
    DynamicSceneGraph& dsg,
    const UpdateInfo::ConstPtr& info,
    const LayerView& view) const {
  if (!info->deformation_graph) {
    return;
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

  const auto& dgraph = *info->deformation_graph;
  for (auto& [prefix, attributes] : nodes) {
    attributes.sort();
    dgraph.deformAllPoints(attributes,
                           attributes,
                           prefix,
                           config.num_control_points,
                           config.control_point_tolerance_s);
  }

  // Copy the newly interpolated positions to the merged DSG.
  for (const auto& node : view) {
    auto node_ptr = dsg.findNode(node.id);
    if (node_ptr) {
      node_ptr->attributes().position = node.attributes().position;
    }
  }

  // Update the cached positions.
  for (auto it = cached_pos_.begin(); it != cached_pos_.end();) {
    if (!unmerged.hasNode(it->first)) {
      it = cached_pos_.erase(it);
    } else {
      ++it;
    }
  }
}

}  // namespace hydra
