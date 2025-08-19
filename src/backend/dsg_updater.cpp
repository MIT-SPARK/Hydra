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

#include "hydra/common/global_info.h"
#include "hydra/common/launch_callbacks.h"
#include "hydra/common/pipeline_queues.h"
#include "hydra/utils/minimum_spanning_tree.h"
#include "hydra/utils/pgmo_mesh_traits.h"
#include "hydra/utils/timing_utilities.h"

namespace hydra {

using hydra::timing::ScopedTimer;
using kimera_pgmo::KimeraPgmoInterface;
using pose_graph_tools::PoseGraph;

void declare_config(DsgUpdater::Config& config) {
  using namespace config;
  name("DsgUpdaterConfig");
  field(config.enable_node_merging, "enable_node_merging");
  field(config.enable_exhaustive_merging, "enable_exhaustive_merging");
  field(config.update_functors, "update_functors");
}

DsgUpdater::DsgUpdater(const Config& config,
                       DynamicSceneGraph::Ptr source,
                       SharedDsgInfo::Ptr target)
    : config(config::checkValid(config)), source_graph_(source), target_dsg_(target) {
  for (const auto& [name, functor] : config.update_functors) {
    update_functors_.emplace(name, functor.create());
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
  {
    // First reset private graph
    target_dsg_->graph->clear();
  }

  // TODO(nathan) this might break mesh stuff
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
  GraphMergeConfig merge_config;
  merge_config.previous_merges = &target_dsg_->merges;
  merge_config.update_dynamic_attributes = false;
  target_dsg_->graph->mergeGraph(*source_graph_, merge_config);

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
      // TODO(nathan) handle given merges
      const auto merges = hooks.find_merges(*source_graph_, info);
      const auto applied =
          merge_tracker.applyMerges(*source_graph_, merges, *target_dsg_, hooks.merge);
      VLOG(1) << "[Backend: " << name << "] Found " << merges.size()
              << " merges (applied " << applied << ")";

      if (config.enable_exhaustive_merging) {
        size_t merge_iter = 0;
        size_t num_applied = 0;
        do {
          const auto new_merges = hooks.find_merges(*target_dsg_->graph, info);
          num_applied = merge_tracker.applyMerges(
              *target_dsg_->graph, new_merges, *target_dsg_, hooks.merge);
          VLOG(1) << "[Backend: " << name << "] Found " << new_merges.size()
                  << " merges at pass " << merge_iter << " (" << num_applied
                  << " applied)";
          ++merge_iter;
        } while (num_applied > 0);
      }
    }
  }

  launchCallbacks(cleanup_hooks, info, target_dsg_.get());
}

}  // namespace hydra
