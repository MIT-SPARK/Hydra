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
#include <yaml-cpp/yaml.h>

#include <filesystem>
#include <fstream>

DEFINE_double(dilation_threshold, 1.0e-4, "dilation threshold");
DEFINE_int32(min_component_size, 1, "minimum component size");
DEFINE_bool(include_nodes, false, "include nodes in filtration");
DEFINE_string(output_path, "filtrations.json", "output path for json file");

using hydra::DisjointSet;
using hydra::getGraphFiltration;
using spark_dsg::DsgLayers;
using spark_dsg::DynamicSceneGraph;
using spark_dsg::NodeId;

int main(int argc, char* argv[]) {
  FLAGS_minloglevel = 0;
  FLAGS_logtostderr = 1;
  FLAGS_colorlogtostderr = 1;

  google::SetUsageMessage("utility to compute dsg filtrations");
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);

  hydra::timing::ElapsedTimeRecorder::instance().disable_output = false;

  if (argc <= 1) {
    LOG(FATAL) << "DSG files required! Usage: compute_filtrations DSG_FILE1 ...";
    return 1;
  }

  std::vector<std::string> files;
  for (int i = 1; i < argc; ++i) {
    files.push_back(std::string(argv[i]));
  }

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
    const auto graph = DynamicSceneGraph::load(filename);
    const auto& places = graph->getLayer(DsgLayers::PLACES);
    LOG(INFO) << "Loaded " << filename << " with " << places.numNodes() << " nodes and "
              << places.numEdges() << " edges";
    LOG(INFO) << "Original rooms: " << graph->getLayer(DsgLayers::ROOMS).numNodes();

    hydra::BarcodeTracker tracker(FLAGS_min_component_size);
    hydra::Filtration filtration;
    {  // start timing scope
      hydra::timing::ScopedTimer timer("filtration", index, true, 0);
      filtration = getGraphFiltration(
          places,
          tracker,
          FLAGS_dilation_threshold,
          [&](const DisjointSet& components) {
            size_t valid = 0;
            for (const auto& size_tuple : components.sizes) {
              const auto size = size_tuple.second;
              const auto min_size = static_cast<size_t>(FLAGS_min_component_size);
              valid += size >= min_size ? 1 : 0;
            }
            return valid;
          },
          FLAGS_include_nodes);
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

  std::ofstream outfile(FLAGS_output_path);
  outfile << yaml_results;

  return 0;
}
