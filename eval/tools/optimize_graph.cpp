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
#include <config_utilities/parsing/yaml.h>
#include <gflags/gflags.h>
#include <hydra/backend/backend_module.h>
#include <hydra/common/global_info.h>
#include <kimera_pgmo/deformation_graph.h>

#include <filesystem>

DEFINE_string(config_path, "", "path to backend config to use");
DEFINE_string(graph_path, "", "scene graph file to read");
DEFINE_string(deformation_graph_path, "", "deformation graph file");
DEFINE_string(output_path, "", "dsg file to read");

namespace hydra {

struct OptimizationConfig {
  std::filesystem::path config_path;
  std::filesystem::path graph;
  std::filesystem::path deformation_graph;
  std::filesystem::path output;

  bool valid() const {
    if (config_path.empty() || graph.empty() || deformation_graph.empty()) {
      return false;
    }

    return std::filesystem::exists(config_path) && std::filesystem::exists(graph) &&
           std::filesystem::exists(deformation_graph);
  }

  operator bool() const { return valid(); }
};

std::ostream& operator<<(std::ostream& out, const OptimizationConfig& info) {
  out << "Paths:\n"
      << "  - config_path: " << info.config_path << " (exists: " << std::boolalpha
      << std::filesystem::exists(info.config_path) << ")\n"
      << "  - graph: " << info.graph << " (exists: " << std::boolalpha
      << std::filesystem::exists(info.graph) << ")\n"
      << "  - deformation_graph: " << info.deformation_graph
      << " (exists: " << std::boolalpha
      << std::filesystem::exists(info.deformation_graph) << ")\n"
      << "  - output: " << info.output;
  return out;
}

void optimize_graph(const OptimizationConfig& info) {
  if (!info) {
    LOG(ERROR) << "Invalid input settings! " << info;
    return;
  }

  // TODO(nathan) maybe pull robot id from somewhere
  const PipelineConfig global_config;
  GlobalInfo::init(global_config, 0);
  SharedModuleState::Ptr state(new SharedModuleState());
  state->backend_graph = std::make_shared<SharedDsgInfo>(global_config.layer_id_map);
  state->backend_graph->graph = spark_dsg::DynamicSceneGraph::load(info.graph);

  auto dsg = state->backend_graph->clone();
  const auto config = config::fromYamlFile<BackendModule::Config>(info.config_path);
  BackendModule backend(config, dsg, state);
  LOG(INFO) << "Loading backend state!";
  backend.loadState(info.graph, info.deformation_graph);
  LOG(INFO) << "Loaded backend state!";

  backend.step(true);  // forces optimization even if no loop closures set
  if (!info.output.empty()) {
    LOG(INFO) << "Saving optimized graph to " << info.output;
    dsg->graph->save(info.output);
  }
}

}  // namespace hydra

int main(int argc, char* argv[]) {
  FLAGS_minloglevel = 0;
  FLAGS_logtostderr = 1;
  FLAGS_colorlogtostderr = 1;

  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);

  const hydra::OptimizationConfig info{FLAGS_config_path,
                                       FLAGS_graph_path,
                                       FLAGS_deformation_graph_path,
                                       FLAGS_output_path};
  hydra::optimize_graph(info);
  return 0;
}
