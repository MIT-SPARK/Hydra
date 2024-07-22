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
#include "hydra/backend/update_frontiers_functor.h"

#include "hydra/common/global_info.h"
#include "hydra/utils/nearest_neighbor_utilities.h"
#include "hydra/utils/timing_utilities.h"

namespace hydra {

using timing::ScopedTimer;

UpdateFunctor::Hooks UpdateFrontiersFunctor::hooks() const {
  auto my_hooks = UpdateFunctor::hooks();
  my_hooks.cleanup = [this](const UpdateInfo::ConstPtr& info, SharedDsgInfo* dsg) {
    if (dsg) {
      cleanup(info->timestamp_ns, *dsg);
    }
  };

  return my_hooks;
}

MergeList UpdateFrontiersFunctor::call(const DynamicSceneGraph&,
                                       SharedDsgInfo&,
                                       const UpdateInfo::ConstPtr&) const {
  return {};
}

void UpdateFrontiersFunctor::cleanup(uint64_t timestamp_ns, SharedDsgInfo& dsg) const {
  ScopedTimer spin_timer("backend/cleanup_frontiers", timestamp_ns);
  std::unique_lock<std::mutex> lock(dsg.mutex);
  const SceneGraphLayer& places_layer = dsg.graph->getLayer(DsgLayers::PLACES);

  std::unordered_set<NodeId> layer_nodes;
  std::unordered_set<NodeId> frontier_nodes;
  for (const auto& id_node_pair : places_layer.nodes()) {
    auto& attrs = id_node_pair.second->attributes<PlaceNodeAttributes>();
    if (attrs.real_place) {
      layer_nodes.insert(id_node_pair.first);
    } else if (!attrs.is_predicted && !attrs.need_cleanup) {
      frontier_nodes.insert(id_node_pair.first);
    }
  }

  bool have_existing_frontiers = frontier_nodes.size() > 0;

  auto place_finder = std::make_unique<NearestNodeFinder>(places_layer, layer_nodes);
  auto frontier_finder =
      std::make_unique<NearestNodeFinder>(places_layer, frontier_nodes);

  std::set<NodeId> nodes_to_remove;
  std::set<NodeId> nodes_to_copy;
  for (auto& id_node_pair : places_layer.nodes()) {
    auto& attrs = id_node_pair.second->attributes<PlaceNodeAttributes>();
    if (attrs.real_place) {
      continue;
    }

    const auto& prefix = GlobalInfo::instance().getRobotPrefix();
    const auto& agents = dsg.graph->getLayer(DsgLayers::AGENTS, prefix.key);
    NodeSymbol pgmo_key(prefix.key, agents.numNodes() - 1);
    Eigen::Vector3d agent_pos = dsg.graph->getNode(pgmo_key).attributes().position;

    if (have_existing_frontiers && attrs.need_cleanup) {
      NodeId nearest_frontier;
      frontier_finder->find(attrs.position, 1, false, [&](NodeId pid, size_t, double) {
        nearest_frontier = pid;
      });
      const auto& neighbor_attrs =
          dsg.graph->getNode(nearest_frontier).attributes<FrontierNodeAttributes>();
      if ((neighbor_attrs.position - attrs.position).norm() <
          config.frontier_exclusion_radius) {
        nodes_to_remove.insert(id_node_pair.first);
        continue;
      }
    }

    bool close_to_robot =
        (agent_pos - attrs.position).norm() < config.frontier_removal_check_threshold;
    if (attrs.need_cleanup || close_to_robot) {
      attrs.need_cleanup = false;

      std::vector<std::pair<Eigen::Vector3d, double>> nearest_places;
      place_finder->findRadius(
          attrs.position, 5, false, [&](NodeId pid, size_t, double) {
            auto& pattr = dsg.graph->getNode(pid).attributes<PlaceNodeAttributes>();
            nearest_places.push_back({pattr.position, pattr.distance});
          });

      bool remove = false;
      for (auto center_rad : nearest_places) {
        if ((center_rad.first - attrs.position).norm() <= center_rad.second) {
          nodes_to_remove.insert(id_node_pair.first);
          remove = true;
          break;
        }
      }
      if (remove) {
        continue;
      }
    }

    // Some frontiers may end up outside any places even when the environment
    // is fully explored, because the places aren't infinitely dense. If the
    // robot gets close enough to these frontiers, we clear them.
    // NOTE: currently we don't delete these nodes, we actually turn them into
    // anti-frontiers
    if ((agent_pos - attrs.position).norm() < config.frontier_removal_threshold) {
      // nodes_to_remove.insert(id_node_pair.first);
      if (!attrs.active_frontier) {
        attrs.anti_frontier = true;
      } else {
        // If the frontier is active, we need to copy it or it will be deleted by the
        // frontend
        nodes_to_copy.insert(id_node_pair.first);
      }

      continue;
    }
  }

  for (const auto nid : nodes_to_remove) {
    dsg.graph->removeNode(nid);
  }

  for (const auto nid : nodes_to_copy) {
    NodeSymbol new_node_id = next_node_id_++;
    auto cloned_attrs = dsg.graph->getNode(nid).attributes().clone();
    dynamic_cast<PlaceNodeAttributes*>(cloned_attrs.get())->active_frontier = false;
    dsg.graph->emplaceNode(DsgLayers::PLACES, new_node_id, std::move(cloned_attrs));
  }
}

void declare_config(UpdateFrontiersFunctor::Config& config) {
  using namespace config;
  name("FrontierConfig");
  field(config.frontier_removal_threshold, "frontier_removal_threshold");
  field(config.frontier_removal_check_threshold, "frontier_removal_check_threshold");
  field(config.frontier_exclusion_radius, "frontier_exclusion_radius");
}

}  // namespace hydra
