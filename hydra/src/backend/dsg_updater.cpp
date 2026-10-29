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
#include <spark_dsg/node_attributes.h>
#include <spark_dsg/scene_graph.h>

#include "hydra/common/global_info.h"
#include "hydra/common/launch_callbacks.h"
#include "hydra/utils/image_store.h"
#include "hydra/utils/pgmo_mesh_traits.h"  // IWYU pragma: keep
#include "hydra/utils/timing_utilities.h"

using namespace spark_dsg;

namespace hydra {
namespace {

void findAndApplyMerges(const VerbosityConfig& config,
                        const UpdateFunctor::Hooks& hooks,
                        const UpdateInfo::ConstPtr& info,
                        const SceneGraph& source,
                        SharedDsgInfo& target,
                        MergeTracker& tracker,
                        bool exhaustive) {
  // TODO(nathan) handle given merges
  auto merges = hooks.find_merges(source, info);
  auto applied = tracker.applyMerges(source, merges, target, hooks.merge);
  MLOG(1) << "pass 0: " << merges.size() << " merges (applied " << applied << ")";
  if (!exhaustive) {
    return;
  }

  applied = 0;
  size_t iter = 0;
  do {
    merges = hooks.find_merges(*target.graph, info);
    applied = tracker.applyMerges(source, merges, target, hooks.merge);
    MLOG(1) << "pass " << iter << ": " << merges.size() << " merges (applied "
            << applied << ")";
    ++iter;
  } while (applied > 0);
}

}  // namespace

using hydra::timing::ScopedTimer;

DsgUpdater::Config::Config() : VerbosityConfig("[dsg_updated] ") {}

void declare_config(DsgUpdater::Config& config) {
  using namespace config;
  name("DsgUpdaterConfig");
  base<VerbosityConfig>(config);
  field(config.functor_logging, "functor_logging");
  field(config.enable_node_merging, "enable_node_merging");
  field(config.reset_dsg_on_loop_closure, "reset_dsg_on_loop_closure");
  field(config.update_functors, "update_functors");
  field(config.exhaustive_functors, "exhaustive_functors");
}

DsgUpdater::DsgUpdater(const Config& config,
                       SceneGraph::Ptr source,
                       SharedDsgInfo::Ptr target)
    : config(config::checkValid(config)),
      exhaustive_names_(config.exhaustive_functors.begin(),
                        config.exhaustive_functors.end()),
      image_store_(GlobalInfo::instance().imageStore()),
      source_graph_(source),
      target_dsg_(target) {
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

void DsgUpdater::reset(uint64_t timestamp_ns) {
  ScopedTimer timer("dsg_updater/reset_dsg", timestamp_ns, true, 1, false);
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

void DsgUpdater::handleMeshUpdate(uint64_t timestamp_ns,
                                  const kimera_pgmo::MeshOffsetInfo& offsets) {
  ScopedTimer spin_timer("dsg_updater/update_layers", timestamp_ns);
  std::list<UpdateFunctor::Hooks::MeshUpdateFunc> funcs;
  for (const auto& [name, functor] : update_functors_) {
    if (!functor) {
      continue;
    }

    const auto hooks = functor->hooks();
    if (hooks.mesh_update) {
      funcs.push_back(hooks.mesh_update);
    }
  }

  launchCallbacks(funcs, std::cref(*target_dsg_->graph), offsets);
}

void DsgUpdater::markNewNodes() {
  // TODO(nathan) fix!
  // Nodes occasionally get added to the backend after they've left the active window,
  // which means they never get deformed or updated correctly. This forces them to be
  // active for at least one update
  for (const auto& layer : source_graph_->layers()) {
    for (const auto& node : layer.nodes()) {
      auto& attrs = node.attributes();
      if (source_graph_->checkNode(node.id) == NodeStatus::NEW && !attrs.is_active) {
        attrs.is_active = true;
        active_nodes_to_restore_.push_back(node.id);
      }
    }
  }
}

void DsgUpdater::callUpdateFunctions(size_t timestamp_ns, UpdateInfo::ConstPtr info) {
  ScopedTimer spin_timer("dsg_updater/update_layers", timestamp_ns);
  // merge topological changes to private dsg, respecting merges
  // attributes may be overwritten, but ideally we don't bother
  GraphMergeConfig merge_config;
  merge_config.previous_merges = &target_dsg_->merges;
  target_dsg_->graph->mergeGraph(*source_graph_, merge_config);

  markNewNodes();
  callFunctors(info);
  clearNewNodes();
}

void DsgUpdater::callFunctors(UpdateInfo::ConstPtr info) {
  target_dsg_->new_merges.clear();
  std::list<UpdateFunctor::Hooks::CleanupFunc> cleanup_hooks;
  for (const auto& [name, functor] : update_functors_) {
    if (!functor) {
      continue;
    }

    const auto hooks = functor->hooks();
    if (hooks.cleanup) {
      cleanup_hooks.push_back(hooks.cleanup);
    }

    functor->call(*source_graph_, *target_dsg_, info);
    if (hooks.find_merges && config.enable_node_merging) {
      auto& tracker = merge_tracker.getMergeGroup(name);
      findAndApplyMerges(config.functor_logging.with_name(name),
                         hooks,
                         info,
                         *source_graph_,
                         *target_dsg_,
                         tracker,
                         exhaustive_names_.count(name));
      if (info->full_update && hooks.merge) {
        MLOG(3) << "updating all merge attributes for " << name;
        MLOG(3) << "current tracker: " << tracker.print();
        tracker.updateAllMergeAttributes(
            *source_graph_, *target_dsg_->graph, hooks.merge);
      }
    }
  }

  MLOG(2) << "all merges: " << merge_tracker.print();
  for (const auto& func : cleanup_hooks) {
    func(info, *source_graph_, target_dsg_.get());
  }

  reportMerges();
}

void DsgUpdater::reportMerges() {
  // merges are cumulative (child -> final parent): merging a parent re-parents its
  // children and resetting the backend graph clears them, so both additions and
  // removals are reported relative to the last report
  const auto& merges = target_dsg_->merges;
  auto& updates = target_dsg_->new_merges;
  std::map<NodeId, NodeId> changed;
  for (const auto& [child, parent] : merges) {
    const auto iter = reported_merges_.find(child);
    if (iter == reported_merges_.end() || iter->second != parent) {
      changed.emplace(child, parent);
      updates.push_back({child, parent});
    }
  }

  std::vector<NodeId> removed;
  for (const auto& [child, parent] : reported_merges_) {
    if (!merges.count(child)) {
      removed.push_back(child);
      updates.push_back({child, std::nullopt});
    }
  }

  reported_merges_ = merges;
  if (!updates.empty()) {
    MLOG(1) << "merge updates: " << changed.size() << " changed, " << removed.size()
            << " removed";
  }

  if (image_store_ && !updates.empty()) {
    image_store_->recordMerges(changed, removed);
  }
}

void DsgUpdater::clearNewNodes() {
  // We reset active flags for all new nodes that were inactive after one update
  for (auto node_id : active_nodes_to_restore_) {
    auto node = source_graph_->findNode(node_id);
    if (node) {
      node->attributes().is_active = false;
    }
  }

  // clear new node status
  source_graph_->getNewNodes(true);
}

void DsgUpdater::finalize(UpdateInfo::ConstPtr info) {
  for (const auto& layer : source_graph_->all_layers()) {
    for (const auto& node : layer.nodes()) {
      node.attributes().is_active = false;
    }
  }

  for (const auto& layer : target_dsg_->graph->all_layers()) {
    for (const auto& node : layer.nodes()) {
      node.attributes().is_active = false;
    }
  }

  callFunctors(info);
}

}  // namespace hydra
