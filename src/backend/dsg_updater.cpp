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
#include "hydra/backend/dsg_updater.h"

#include <config_utilities/config.h>
#include <config_utilities/printing.h>
#include <config_utilities/validation.h>
#include <glog/logging.h>
#include <glog/stl_logging.h>
#include <kimera_pgmo/utils/mesh_io.h>

#include "hydra/utils/pgmo_mesh_traits.h"  // IWYU pragma: keep
#include "hydra/utils/timing_utilities.h"

namespace hydra {

using hydra::timing::ScopedTimer;

void declare_config(DsgUpdater::Config& config) {
  using namespace config;
  name("DsgUpdaterConfig");
  field(config.enable_node_merging, "enable_node_merging");
  field(config.enable_exhaustive_merging, "enable_exhaustive_merging");
  field(config.reset_dsg_on_loop_closure, "reset_dsg_on_loop_closure");
  field(config.update_functors, "update_functors");
}

DsgUpdater::DsgUpdater(const Config& config,
                       DynamicSceneGraph::Ptr source,
                       SharedDsgInfo::Ptr target)
    : config(config::checkValid(config)), source_graph_(source), target_dsg_(target) {
  for (const auto& [name, functor] : config.update_functors) {
    update_functors_.emplace(name, functor.create());
    merge_tracker.initializeTracker(name);
  }
}

void DsgUpdater::save(const DataDirectory& output, const std::string& label) const {
  std::lock_guard<std::mutex> graph_lock(target_dsg_->mutex);
  const auto graph_path = output.path(label);
  target_dsg_->graph->save(graph_path / "dsg.json", false);
  target_dsg_->graph->save(graph_path / "dsg_with_mesh.json");

  const auto mesh = target_dsg_->graph->mesh();
  if (mesh && !mesh->empty()) {
    // mesh implements vertex and face traits
    kimera_pgmo::WriteMesh(graph_path / "mesh.ply", *mesh, *mesh);
  }
}

void DsgUpdater::resetBackendDsg(size_t timestamp_ns) {
  ScopedTimer timer("dsg_updater/reset_dsg", timestamp_ns, true, 0, false);
  constexpr bool reset_mesh = false;
  {
    std::unique_lock<std::mutex> graph_lock(target_dsg_->mutex);
    // First reset private graph
    target_dsg_->graph->clear(reset_mesh);
  }

  // TODO(nathan) this might break mesh stuff
  // NOTE(lschmid): This does break mesh stuff (copyMeshDelta and
  // extractObjectAABBs-from mesh object segmentation). If reset mesh functionality is
  // added again "deformation_graph_->setRecalculateVertices();" used to be called here.
  target_dsg_->graph->mergeGraph(*source_graph_);
  target_dsg_->merges.clear();
  merge_tracker.clear();
}

void DsgUpdater::callUpdateFunctions(size_t timestamp_ns, UpdateInfo::ConstPtr info) {
  ScopedTimer spin_timer("dsg_updater/update_layers", timestamp_ns);

  // TODO(nathan) chance that this causes weirdness when we have multiple nodes but no
  // accepted reconciliation merges
  const bool enable_merging =
      info->given_merges.empty() ? config.enable_node_merging : false;

  // merge topological changes to private dsg, respecting merges
  // attributes may be overwritten, but ideally we don't bother
  if (config.reset_dsg_on_loop_closure && info->loop_closure_detected) {
    resetBackendDsg(timestamp_ns);
  }

  GraphMergeConfig merge_config;
  merge_config.previous_merges = &target_dsg_->merges;
  merge_config.update_dynamic_attributes = false;
  target_dsg_->graph->mergeGraph(*source_graph_, merge_config);

  // Nodes occasionally get added to the backend after they've left the active window,
  // which means they never get deformed or updated correctly. This forces them to be
  // active for at least one update
  std::vector<NodeId> active_nodes_to_restore;
  for (auto& [layer_id, layer] : source_graph_->layers()) {
    for (auto& [node_id, node] : layer->nodes()) {
      auto& attrs = node->attributes();
      if (source_graph_->checkNode(node_id) == NodeStatus::NEW && !attrs.is_active) {
        attrs.is_active = true;
        active_nodes_to_restore.push_back(node_id);
      }
    }
  }

  std::list<LayerCleanupFunc> cleanup_hooks;
  for (const auto& [name, functor] : update_functors_) {
    if (!functor) {
      continue;
    }

    const auto hooks = functor->hooks();
    if (hooks.cleanup) {
      cleanup_hooks.push_back(hooks.cleanup);
    }

    functor->call(*source_graph_, *target_dsg_, info);
    if (hooks.find_merges && enable_merging) {
      auto& tracker = merge_tracker.getMergeGroup(name);

      // TODO(nathan) handle given merges
      const auto merges = hooks.find_merges(*source_graph_, info);
      const auto applied =
          tracker.applyMerges(*source_graph_, merges, *target_dsg_, hooks.merge);
      VLOG(1) << "[Backend: " << name << "] Found " << merges.size()
              << " merges (applied " << applied << ")";

      if (config.enable_exhaustive_merging) {
        size_t merge_iter = 0;
        size_t num_applied = 0;
        do {
          if (name == "surface_places") {
            continue;
          }

          const auto new_merges = hooks.find_merges(*target_dsg_->graph, info);
          num_applied = tracker.applyMerges(
              *source_graph_, new_merges, *target_dsg_, hooks.merge);
          VLOG(1) << "[Backend: " << name << "] Found " << new_merges.size()
                  << " merges at pass " << merge_iter << " (" << num_applied
                  << " applied)";
          ++merge_iter;
        } while (num_applied > 0);
      }
      if (info->loop_closure_detected && hooks.merge) {
        LOG(INFO) << "Updating all merge attributes for " << name;
        LOG(INFO) << "Current tracker: " << tracker.print();
        tracker.updateAllMergeAttributes(
            *source_graph_, *target_dsg_->graph, hooks.merge);
      }
    }

    LOG(INFO) << "All merges: " << merge_tracker.print();
    for (const auto& func : cleanup_hooks) {
      func(info, *source_graph_, target_dsg_.get());
    }

    // We reset active flags for all new nodes that were inactive after one update
    for (auto node_id : active_nodes_to_restore) {
      auto node = source_graph_->findNode(node_id);
      if (node) {
        node->attributes().is_active = false;
      }
    }

    // clear new node status
    // TODO(nathan) add API for marking new nodes
    source_graph_->getNewNodes(true);
  }
}

}  // namespace hydra
