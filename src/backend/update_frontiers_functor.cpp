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

#include <config_utilities/config.h>
#include <spark_dsg/dynamic_scene_graph.h>

#include "hydra/common/global_info.h"
#include "hydra/utils/nearest_neighbor_utilities.h"
#include "hydra/utils/timing_utilities.h"

namespace hydra {

using timing::ScopedTimer;

UpdateFunctor::Hooks UpdateFrontiersFunctor::hooks() const {
  auto my_hooks = UpdateFunctor::hooks();
  my_hooks.cleanup = [this](const UpdateInfo::ConstPtr& info,
                            DynamicSceneGraph& unmerged_dsg,
                            SharedDsgInfo* dsg) {
    if (dsg) {
      cleanup(info->timestamp_ns, unmerged_dsg, *dsg);
    }
  };

  return my_hooks;
}

void UpdateFrontiersFunctor::call(const DynamicSceneGraph&,
                                  SharedDsgInfo&,
                                  const UpdateInfo::ConstPtr&) const {
  return;
}

bool check_against_places(const SharedDsgInfo& dsg,
                          const std::shared_ptr<NearestNodeFinder> place_finder,
                          const spark_dsg::PlaceNodeAttributes& attrs) {
  std::vector<std::pair<Eigen::Vector3d, double>> nearest_places;
  place_finder->findRadius(attrs.position, 5, false, [&](NodeId pid, size_t, double) {
    auto& pattr = dsg.graph->getNode(pid).attributes<PlaceNodeAttributes>();
    nearest_places.push_back({pattr.position, pattr.distance});
  });

  for (auto center_rad : nearest_places) {
    if ((center_rad.first - attrs.position).norm() <= center_rad.second) {
      return true;
    }
  }
  return false;
}

bool point_in_polygon(Eigen::Vector2d pt, std::vector<Eigen::Vector3d> polygon) {
  if (polygon.size() < 3) {
    return false;
  }

  double projection_minimum = 0.0;
  for (int idx = 0; idx < polygon.size(); ++idx) {
    auto x1 = polygon.at(idx);
    auto x2 = polygon.at((idx + 1) % polygon.size());
    auto u = (x2 - x1).head(2);
    if (u.norm() < .00001) {
      LOG(WARNING) << "short edge: " << x2 << x1;
    }
    Eigen::Vector2d normal(-u(1), u(0));
    auto projection = normal.dot(pt - x1.head(2));
    if (projection < projection_minimum) {
      return false;
    }
  }
  return true;
}

bool check_against_2d_places(const SharedDsgInfo& dsg,
                             const std::shared_ptr<NearestNodeFinder> place_2d_finder,
                             const spark_dsg::PlaceNodeAttributes& attrs) {
  std::vector<std::vector<Eigen::Vector3d>> nearest_places;
  place_2d_finder->findRadius(
      attrs.position, 20, false, [&](NodeId pid, size_t, double) {
        auto& pattr = dsg.graph->getNode(pid).attributes<Place2dNodeAttributes>();
        nearest_places.push_back(pattr.boundary);
      });

  for (auto boundary : nearest_places) {
    if (point_in_polygon(attrs.position.head(2), boundary)) {
      return true;
    }
  }
  return false;
}

void UpdateFrontiersFunctor::cleanup(uint64_t timestamp_ns,
                                     DynamicSceneGraph& unmerged_dsg,
                                     SharedDsgInfo& dsg) const {
  ScopedTimer spin_timer("backend/cleanup_frontiers", timestamp_ns);
  if (!dsg.graph->hasLayer(DsgLayers::PLACES)) {
    return;
  }
  std::lock_guard<std::mutex> lock(dsg.mutex);

  for (const auto nid : deleted_frontiers_) {
    dsg.graph->removeNode(nid);
    unmerged_dsg.removeNode(nid);
  }

  const auto& places_layer = dsg.graph->getLayer(DsgLayers::PLACES);
  const auto& places_2d_layer = dsg.graph->getLayer(DsgLayers::MESH_PLACES);

  const auto& prefix = GlobalInfo::instance().getRobotPrefix();
  const auto layer_key = dsg.graph->getLayerKey(DsgLayers::AGENTS);
  if (!layer_key) {
    LOG(ERROR) << "Update frontiers functor could not find agents layer";
    return;
  }
  const auto agents = dsg.graph->findLayer(layer_key->layer, prefix.key);
  if (!agents || agents->numNodes() == 0) {
    return;
  }
  NodeSymbol pgmo_key(prefix.key, agents->numNodes() - 1);
  Eigen::Vector3d agent_pos = dsg.graph->getNode(pgmo_key).attributes().position;
  Eigen::Quaterniond agent_quat =
      dsg.graph->getNode(pgmo_key).attributes<AgentNodeAttributes>().world_R_body;
  Eigen::Vector3d agent_heading = agent_quat * Eigen::Vector3d(1, 0, 0);

  std::unordered_set<NodeId> layer_nodes;
  std::unordered_set<NodeId> frontier_nodes;
  for (const auto& [node_id, node] : places_layer.nodes()) {
    auto attrs = node->tryAttributes<PlaceNodeAttributes>();
    if (!attrs) {
      continue;
    }
    if (attrs->real_place) {
      layer_nodes.insert(node_id);
    } else if (!attrs->is_predicted && !attrs->need_cleanup) {
      frontier_nodes.insert(node_id);
    }
  }

  std::unordered_set<NodeId> layer_2d_nodes;
  for (const auto& [node_id, node] : places_2d_layer.nodes()) {
    layer_2d_nodes.insert(node_id);
  }

  bool have_existing_frontiers = frontier_nodes.size() > 0;

  auto place_finder = std::make_shared<NearestNodeFinder>(places_layer, layer_nodes);
  auto place_2d_finder =
      std::make_shared<NearestNodeFinder>(places_2d_layer, layer_2d_nodes);
  auto frontier_finder =
      std::make_unique<NearestNodeFinder>(places_layer, frontier_nodes);

  std::set<NodeId> nodes_to_remove;
  std::set<NodeId> nodes_to_copy;
  std::set<NodeId> nodes_to_anti;
  for (const auto& [node_id, node] : places_layer.nodes()) {
    auto attrs = node->tryAttributes<PlaceNodeAttributes>();
    if (!attrs || attrs->real_place) {
      continue;
    }

    // Delete frontier if it's too close to an existing frontier.
    if (have_existing_frontiers && attrs->need_cleanup) {
      NodeId nearest_frontier;
      frontier_finder->find(attrs->position, 1, false, [&](NodeId pid, size_t, double) {
        nearest_frontier = pid;
      });
      const auto& neighbor_attrs =
          dsg.graph->getNode(nearest_frontier).attributes<FrontierNodeAttributes>();
      double neighbor_distance = (neighbor_attrs.position - attrs->position).norm();
      if (neighbor_distance < config.frontier_exclusion_radius) {
        nodes_to_remove.insert(node_id);
        continue;
      }
    }

    // Delete frontiers that are inside of existing places.
    bool frontier_check_3d_places = false;
    bool frontier_check_2d_places = true;
    bool remove = false;

    bool close_to_robot =
        (agent_pos - attrs->position).norm() < config.frontier_removal_check_threshold;

    if (attrs->need_cleanup || close_to_robot) {
      attrs->need_cleanup = false;
      auto unmerged_node = unmerged_dsg.findNode(node_id);
      if (unmerged_node) {
        unmerged_node->attributes<FrontierNodeAttributes>().need_cleanup = false;
      }
      if (frontier_check_3d_places) {
        remove = check_against_places(dsg, place_finder, *attrs);
      }

      if (frontier_check_2d_places) {
        remove |= check_against_2d_places(dsg, place_2d_finder, *attrs);
      }
    }

    if (!frontier_check_2d_places && !frontier_check_3d_places) {
      attrs->need_cleanup = false;
    }

    if (remove) {
      nodes_to_remove.insert(node_id);
      continue;
    }

    // Some frontiers may end up outside any places even when the environment
    // is fully explored, because the places aren't infinitely dense. If the
    // robot gets close enough to these frontiers, we clear them.
    // NOTE: currently we don't delete these nodes, we actually turn them into
    // anti-frontiers
    auto disp_to_frontier = attrs->position - agent_pos;
    double distance_to_frontier = disp_to_frontier.norm();
    bool facing_frontier =
        (disp_to_frontier / distance_to_frontier).dot(agent_heading) > 0.5;
    bool near_frontier =
        (attrs->position - agent_pos).norm() < config.frontier_removal_threshold;
    if (facing_frontier && near_frontier) {
      // nodes_to_remove.insert(id_node_pair.first);
      if (!attrs->active_frontier) {
        nodes_to_anti.insert(node_id);
      } else {
        // If the frontier is active, we need to copy it or it will be deleted by the
        // frontend
        nodes_to_copy.insert(node_id);
      }

      continue;
    }
  }

  for (const auto nid : nodes_to_anti) {
    auto node = unmerged_dsg.findNode(nid);
    if (node) {
      node->attributes<PlaceNodeAttributes>().anti_frontier = true;
    }
    auto& merged_attrs = dsg.graph->getNode(nid).attributes<PlaceNodeAttributes>();
    merged_attrs.anti_frontier = true;
  }

  for (const auto nid : nodes_to_remove) {
    deleted_frontiers_.insert(nid);
    dsg.graph->removeNode(nid);
    unmerged_dsg.removeNode(nid);
  }

  // TODO: do we need to also deal with copying the "active" frontiers to the unmerged
  // graph?
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
