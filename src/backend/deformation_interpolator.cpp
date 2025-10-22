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
namespace {

std::string printTransform(const Eigen::Isometry3d& tf) {
  Eigen::IOFormat fmt(4, Eigen::DontAlignCols, ", ", "; ", "", "", "[", "]");
  std::stringstream ss;
  ss << "q: " << Eigen::Quaterniond(tf.rotation()).coeffs().format(fmt)
     << ", t: " << tf.translation().format(fmt);
  return ss.str();
}

}  // namespace

using spark_dsg::NodeSymbol;

void declare_config(DeformationInterpolator::Config& config) {
  using namespace config;
  name("DeformationInterpolator::Config");
  field(config.num_control_points, "num_control_points");
  field(config.control_point_tolerance_s, "control_point_tolerance_s", "s");
  check(config.num_control_points, GE, 0, "num_control_points");
  check(config.control_point_tolerance_s, GE, 0.0, "control_point_tolerance_s");
}

NodeCache::Entry* NodeCache::add(NodeId node_id, const NodeAttributes& attrs) {
  uint64_t timestamp_ns = attrs.last_update_time_ns;
  if (timestamp_ns == 0u) {
    const auto derived = dynamic_cast<const KhronosObjectAttributes*>(&attrs);
    if (!derived || derived->last_observed_ns.empty()) {
      LOG(ERROR) << "Unable to find valid timestamp for node "
                 << NodeSymbol(node_id).str();
      return nullptr;
    }

    timestamp_ns = derived->last_observed_ns.back();
  }

  auto iter = nodes.find(node_id);
  if (iter == nodes.end()) {
    return &nodes
                .emplace(node_id,
                         Entry{node_id,
                               attrs.last_update_time_ns,
                               attrs.position.cast<float>()})
                .first->second;
  }

  if (attrs.is_active) {
    iter->second.init_pos = attrs.position.cast<float>();
    iter->second.timestamp = attrs.last_update_time_ns;
  }

  return &iter->second;
}

struct EntryList {
  std::vector<NodeCache::Entry*> entries;

  void sort() {
    std::sort(entries.begin(), entries.end(), [this](const auto& lhs, const auto& rhs) {
      return lhs->timestamp < rhs->timestamp;
    });
  }
};

size_t pgmoNumVertices(const EntryList& entries) { return entries.entries.size(); }

kimera_pgmo::traits::Pos pgmoGetVertex(const EntryList& entries,
                                       size_t i,
                                       kimera_pgmo::traits::VertexTraits* traits) {
  const auto& entry = entries.entries[i];
  if (traits) {
    traits->stamp = entry->timestamp;
  }

  return entry->init_pos;
}

uint64_t pgmoGetVertexStamp(const EntryList& entries, size_t i) {
  return entries.entries[i]->timestamp;
}

DeformationInterpolator::DeformationInterpolator(const Config& config)
    : config(config::checkValid(config)) {}

void DeformationInterpolator::interpolate(const DynamicSceneGraph& unmerged,
                                          DynamicSceneGraph& dsg,
                                          const UpdateInfo::ConstPtr& info,
                                          const LayerView& view) const {
  if (!info->deformation_graph) {
    return;
  }

  auto iter = cache_.nodes.begin();
  while (iter != cache_.nodes.end()) {
    if (!unmerged.hasNode(iter->first)) {
      iter = cache_.nodes.erase(iter);
    } else {
      ++iter;
    }
  }

  const auto this_robot = GlobalInfo::instance().getRobotPrefix().id;
  std::map<size_t, EntryList> robot_entries;
  for (const auto& node : view) {
    size_t robot_id = this_robot;
    if (info->node_to_robot_id) {
      auto iter = info->node_to_robot_id->find(node.id);
      if (iter == info->node_to_robot_id->end()) {
        LOG(WARNING) << "Node " << NodeSymbol(node.id) << " does not belong to a robot";
      } else {
        robot_id = iter->second;
      }
    }

    auto entry = cache_.add(node.id, node.attributes());
    if (!entry) {
      continue;
    }

    auto entries = robot_entries.find(robot_id);
    if (entries == robot_entries.end()) {
      entries = robot_entries.emplace(robot_id, EntryList{}).first;
    }

    entries->second.entries.push_back(entry);
  }

  const auto& dgraph = *info->deformation_graph;
  for (auto& [robot_id, entries] : robot_entries) {
    const auto prefix = kimera_pgmo::GetVertexPrefix(robot_id);
    entries.sort();

    const auto deform_func = [&](const Eigen::Isometry3d& transform, size_t index) {
      auto entry = entries.entries[index];
      LOG(INFO) << "node " << spark_dsg::NodeSymbol(entry->id).str()
                << " -> new: " << printTransform(transform)
                << ", last: " << printTransform(entry->last_transform);

      auto& attrs = dsg.getNode(entry->id).attributes();
      attrs.transform(entry->last_transform.inverse());
      attrs.transform(transform);

      //auto node_ptr = dsg.findNode(entry->id);
      //if (node_ptr) {
        //node_ptr->attributes().transform(entry->last_transform.inverse());
        //node_ptr->attributes().transform(transform);
      //}

      entry->last_transform = transform;
    };

    dgraph.customDeformation(deform_func,
                             entries,
                             prefix,
                             config.num_control_points,
                             config.control_point_tolerance_s);
  }
}

}  // namespace hydra
