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
#include <config_utilities/config.h>
#include <config_utilities/parsing/context.h>
#include <hydra/backend/merge_tracker.h>
#include <hydra/backend/update_functions.h>
#include <spark_dsg/scene_graph.h>

#include <CLI/CLI.hpp>
#include <filesystem>

namespace hydra {

struct AppArgs {
  void add_to_app(CLI::App& app);

  std::filesystem::path scene_graph;
};

void AppArgs::add_to_app(CLI::App& app) {
  app.add_option("scene_graph", scene_graph)
      ->required()
      ->check(CLI::ExistingFile)
      ->description("Graph to reconcile");
}

struct Reconciler {
  struct Config {
    using FunctorConfig = config::VirtualConfig<UpdateFunctor, true>;

    Config();
    //! Update functors that get applied in the specified order
    config::OrderedMap<std::string, FunctorConfig> update_functors;
    //! Names of functors to use exhaustive merging for
    std::vector<std::string> exhaustive_functors;
  } const config;

  void reconcile(const spark_dsg::SceneGraph& graph) const;

  std::vector<std::pair<std::string, std::unique_ptr<UpdateFunctor>>> functors;
};

void Reconciler::reconcile(const spark_dsg::SceneGraph& graph) const {
  const std::set<std::string> exhaustive_names(config.exhaustive_functors.begin(),
                                               config.exhaustive_functors.end());

  auto target = std::make_shared<SharedDsgInfo>(SharedDsgInfo::Config{});
  target->graph = graph.clone();

  auto info = std::make_shared<UpdateInfo>();
  GroupedMergeTracker merge_tracker;
  std::list<UpdateFunctor::Hooks::CleanupFunc> cleanup_hooks;
  for (const auto& [name, functor] : functors) {
    if (!functor) {
      continue;
    }

    const auto hooks = functor->hooks();
    if (hooks.cleanup) {
      cleanup_hooks.push_back(hooks.cleanup);
    }

    functor->call(graph, *target, info);
    if (hooks.find_merges) {
      auto& tracker = merge_tracker.getMergeGroup(name);
      auto merges = hooks.find_merges(graph, info);
      auto applied = tracker.applyMerges(graph, merges, target, hooks.merge);
      if (!exhaustive_names.count(name)) {
        return;
      }

      applied = 0;
      do {
        merges = hooks.find_merges(*target.graph, info);
        applied = tracker.applyMerges(graph, merges, target, hooks.merge);
      } while (applied > 0);
    }

    for (const auto& func : cleanup_hooks) {
      func(info, graph, target);
    }
  }
}

void declare_config(Reconciler::Config& config) {
  using namespace config;
  name("Reconciler::Config");
  field(config.update_functors, "update_functors");
  field(config.exhaustive_functors, "exhaustive_functors");
}

}  // namespace hydra

int main(int argc, char** argv) {
  config::initContext(argc, argv, true);
  config::setConfigSettingsFromContext();

  CLI::App app("Load and reconcile a 3D scene graph");
  argv = app.ensure_utf8(argv);
  app.allow_extras();
  app.get_formatter()->column_width(50);

  hydra::AppArgs args;
  args.add_to_app(app);

  try {
    app.parse(argc, argv);
  } catch (const CLI::ParseError& e) {
    return app.exit(e);
  }

  const auto config = config::fromContext<hydra::Reconciler::Config>();
  auto graph = spark_dsg::SceneGraph::load(args.scene_graph);
  return 0;
}
