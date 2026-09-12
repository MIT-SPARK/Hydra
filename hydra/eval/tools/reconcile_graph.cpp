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
#include <config_utilities/printing.h>
#include <config_utilities/validation.h>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <hydra/backend/merge_tracker.h>
#include <hydra/backend/update_functions.h>
#include <spark_dsg/printing.h>
#include <spark_dsg/scene_graph.h>

#include <CLI/CLI.hpp>
#include <filesystem>

namespace hydra {

struct AppArgs {
  void add_to_app(CLI::App& app);

  std::filesystem::path scene_graph;
  std::string config_ns = "backend";
  std::filesystem::path output;
  int log_level = 0;
  int verbosity = 0;
};

void AppArgs::add_to_app(CLI::App& app) {
  app.add_option("scene_graph", scene_graph)
      ->required()
      ->check(CLI::ExistingFile)
      ->description("Graph to reconcile");
  app.add_option("--namespace", config_ns)->description("Config namespace to use");
  app.add_option("-o,--output", output)->description("File to output to");
  app.add_option("-l,--log-level", log_level)->description("GLOG log level");
  app.add_option("--verbosity", verbosity)->description("GLOG verbosity");
}

struct Reconciler {
  using Graph = spark_dsg::SceneGraph;

  struct Config {
    using FunctorConfig = config::VirtualConfig<UpdateFunctor, true>;

    config::OrderedMap<std::string, FunctorConfig> update_functors;
    std::vector<std::string> exhaustive_functors;
  } const config;

  explicit Reconciler(const Config& config);
  Graph::Ptr reconcile(Graph& graph) const;

  std::vector<std::pair<std::string, std::unique_ptr<UpdateFunctor>>> functors;
};

void declare_config(Reconciler::Config& config) {
  using namespace config;
  name("Reconciler::Config");
  field(config.update_functors, "update_functors");
  field(config.exhaustive_functors, "exhaustive_functors");
}

Reconciler::Reconciler(const Config& config) : config(config::checkValid(config)) {
  std::cout << "Creating reconciler with config:\n"
            << config::toString(config) << std::endl;
  for (const auto& [name, functor] : config.update_functors) {
    functors.emplace_back(name, functor.create());
  }
}

auto Reconciler::reconcile(Graph& graph) const -> Graph::Ptr {
  const std::set<std::string> exhaustive_names(config.exhaustive_functors.begin(),
                                               config.exhaustive_functors.end());

  SharedDsgInfo target(SharedDsgInfo::Config{});
  target.graph = graph.clone();

  auto info = std::make_shared<UpdateInfo>();
  info->loop_closure_detected = true;
  GroupedMergeTracker merge_tracker;
  std::list<UpdateFunctor::Hooks::CleanupFunc> cleanup_hooks;
  for (const auto& [name, functor] : functors) {
    if (!functor) {
      continue;
    }

    merge_tracker.initializeTracker(name);
    const auto hooks = functor->hooks();
    if (hooks.cleanup) {
      cleanup_hooks.push_back(hooks.cleanup);
    }

    functor->call(graph, target, info);
    if (hooks.find_merges) {
      auto& tracker = merge_tracker.getMergeGroup(name);
      auto merges = hooks.find_merges(graph, info);
      auto applied = tracker.applyMerges(graph, merges, target, hooks.merge);
      VLOG(1) << "pass 0: " << merges.size() << " merges (applied " << applied << ")";
      if (!exhaustive_names.count(name)) {
        continue;
      }

      applied = 0;
      size_t iter = 0;
      do {
        merges = hooks.find_merges(*target.graph, info);
        applied = tracker.applyMerges(graph, merges, target, hooks.merge);
        VLOG(1) << "pass " << iter << ": " << merges.size() << " merges (applied "
                << applied << ")";
        ++iter;
      } while (applied > 0);
    }

    for (const auto& func : cleanup_hooks) {
      func(info, graph, &target);
    }
  }

  return target.graph;
}

void printCounts(spark_dsg::SceneGraph& graph) {
  for (const auto& [layer_id, layer] : graph.layers()) {
    std::cout << "Layer " << layer->id << ": " << layer->numNodes() << std::endl;
  }

  for (const auto& [layer_id, partitions] : graph.layer_partitions()) {
    for (const auto& [key, layer] : partitions) {
      std::cout << "Layer " << layer->id << ": " << layer->numNodes() << std::endl;
    }
  }
}

void resetActive(spark_dsg::SceneGraph& graph) {
  for (const auto& [layer_id, layer] : graph.layers()) {
    for (const auto& [node_id, node] : layer->nodes()) {
      node->attributes().is_active = false;
    }
  }

  for (const auto& [layer_id, partitions] : graph.layer_partitions()) {
    for (const auto& [key, layer] : partitions) {
      for (const auto& [node_id, node] : layer->nodes()) {
        node->attributes().is_active = false;
      }
    }
  }
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

  FLAGS_minloglevel = 0;
  FLAGS_logtostderr = 1;
  FLAGS_colorlogtostderr = 1;
  google::InitGoogleLogging(argv[0]);

  auto output = args.output;
  if (output.empty()) {
    const auto filename = args.scene_graph.stem().string() + "_reconciled.json";
    output = args.scene_graph.parent_path() / filename;
  }

  const auto config = config::fromContext<hydra::Reconciler::Config>(args.config_ns);
  const auto reconciler = hydra::Reconciler(config);

  std::cout << "Loading graph from " << args.scene_graph << "..." << std::endl;
  auto graph = spark_dsg::SceneGraph::load(args.scene_graph);
  hydra::resetActive(*graph);
  std::cout << "Before:\n=======" << std::endl;
  hydra::printCounts(*graph);
  std::cout << "Reconciling graph..." << std::endl;
  auto new_graph = reconciler.reconcile(*graph);
  std::cout << "After:\n======" << std::endl;
  hydra::printCounts(*new_graph);
  std::cout << "Saving graph to " << output << "..." << std::endl;
  new_graph->save(output, true);
  return 0;
}
