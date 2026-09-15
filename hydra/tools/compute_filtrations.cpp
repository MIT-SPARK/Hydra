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
#include <glog/logging.h>
#include <hydra/rooms/graph_filtration.h>
#include <hydra/utils/timing_utilities.h>
#include <spark_dsg/scene_graph.h>
#include <yaml-cpp/yaml.h>

#include <CLI/CLI.hpp>
#include <filesystem>
#include <fstream>

using hydra::DisjointSet;
using hydra::getGraphFiltration;
using spark_dsg::DsgLayers;
using spark_dsg::NodeId;
using spark_dsg::SceneGraph;

int main(int argc, char* argv[]) {
  FLAGS_minloglevel = 0;
  FLAGS_logtostderr = 1;
  FLAGS_colorlogtostderr = 1;

  CLI::App app("Compute scene graph filtrations");
  double dilation_threshold = 1.0e-4;
  size_t min_component_size = 1;
  bool include_nodes = false;
  std::string output_path = "filtrations.yaml";
  std::vector<std::string> files;
  app.add_option("graphs", files)->required()->check(CLI::ExistingFile);
  app.add_option("--dilation-threshold", dilation_threshold)
      ->check(CLI::NonNegativeNumber);
  app.add_option("--min-component-size", min_component_size)
      ->check(CLI::PositiveNumber);
  app.add_flag("--include-nodes", include_nodes);
  app.add_option("-o,--output-path", output_path);
  CLI11_PARSE(app, argc, argv);
  google::InitGoogleLogging(argv[0]);

  hydra::timing::ElapsedTimeRecorder::instance().disable_output = false;

  std::map<size_t, std::string> filepaths;
  std::map<size_t, std::list<std::pair<double, size_t>>> results;
  std::map<size_t, std::map<NodeId, std::pair<double, double>>> barcodes;

  size_t index = 0;
  for (const auto& filename : files) {
    LOG(INFO) << "Parsing: " << filename << "...";
    CHECK(!filename.empty());

    const std::filesystem::path filepath(filename);
    filepaths[index] = std::filesystem::canonical(filepath).string();
    results[index] = std::list<std::pair<double, size_t>>();
    const auto graph = SceneGraph::load(filename);
    const auto& places = graph->getLayer(DsgLayers::PLACES);
    LOG(INFO) << "Loaded " << filename << " with " << places.numNodes() << " nodes and "
              << places.numEdges() << " edges";

    hydra::BarcodeTracker tracker(min_component_size);
    hydra::Filtration filtration;
    {  // start timing scope
      hydra::timing::ScopedTimer timer("filtration", index, true, 0);
      filtration = getGraphFiltration(
          places,
          tracker,
          dilation_threshold,
          [&](const DisjointSet& components) {
            size_t valid = 0;
            for (const auto& size_tuple : components.sizes) {
              const auto size = size_tuple.second;
              const auto min_size = static_cast<size_t>(min_component_size);
              valid += size >= min_size ? 1 : 0;
            }
            return valid;
          },
          include_nodes);
    }  // stop timing scope

    for (const auto& info : filtration) {
      results[index].push_back({info.distance, info.num_components});
    }

    barcodes[index] = std::map<NodeId, std::pair<double, double>>();
    for (auto&& [id, lifetime] : tracker.barcodes) {
      barcodes[index][id] = {lifetime.start, lifetime.end};
    }

    ++index;
  }

  YAML::Node yaml_results;
  yaml_results["results"] = results;
  yaml_results["barcodes"] = barcodes;
  yaml_results["filepaths"] = filepaths;

  std::ofstream outfile(output_path);
  outfile << yaml_results;
  if (!outfile) {
    std::cerr << "Failed to write " << output_path << std::endl;
    return 1;
  }

  return 0;
}
