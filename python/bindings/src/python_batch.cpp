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
#include "hydra/bindings/python_batch.h"

#include <config_utilities/config.h>
#include <config_utilities/formatting/asl.h>
#include <config_utilities/logging/log_to_glog.h>
#include <config_utilities/parsing/yaml.h>
#include <config_utilities/validation.h>
#include <hydra/common/batch_pipeline.h>
#include <hydra/common/global_info.h>
#include <pybind11/stl.h>
#include <pybind11/stl/filesystem.h>

#include "hydra/bindings/glog_utilities.h"

namespace hydra::python {

class PythonBatchPipeline : public BatchPipeline {
 public:
  struct Config : PipelineConfig {
    config::VirtualConfig<GraphBuilder> frontend;
    RoomFinderConfig room_finder;
  } const config;

  PythonBatchPipeline(const Config& config, int robot_id = 0);
  virtual ~PythonBatchPipeline() = default;
  DynamicSceneGraph::Ptr construct(const VolumetricMap& map) const;
};

void declare_config(PythonBatchPipeline::Config& config) {
  using namespace config;
  name("PythonBatchPipeline::Config");
  base<PipelineConfig>(config);
  field(config.frontend, "frontend");
  field(config.room_finder, "backend/room_finder");
}

PythonBatchPipeline::PythonBatchPipeline(const Config& config, int robot_id)
    : BatchPipeline(config, robot_id) {
  GlobalInfo::init(config, robot_id, true);
  GlogSingleton::instance().setLogLevel(0, 0, false);
  config::Settings().print_width = 100;
  config::Settings().print_indent = 45;
}

DynamicSceneGraph::Ptr PythonBatchPipeline::construct(const VolumetricMap& map) const {
  const auto new_map = map.clone();
  return BatchPipeline::construct(config.frontend, *new_map, &config.room_finder);
}

namespace python_batch {

using namespace pybind11::literals;
namespace py = pybind11;

void addBindings(pybind11::module_& m) {
  py::class_<VolumetricMap>(m, "VolumetricMap")
      .def_static(
          "load",
          [](const std::string& filepath) { return VolumetricMap::load(filepath); })
      .def_static("load", [](const std::filesystem::path& filepath) {
        return VolumetricMap::load(filepath.string());
      });

  py::class_<PythonBatchPipeline>(m, "BatchPipeline")
      .def_static(
          "from_config",
          [](const std::string& config, int robot_id) {
            const auto node = YAML::Load(config);
            return std::make_unique<PythonBatchPipeline>(
                config::fromYaml<PythonBatchPipeline::Config>(node), robot_id);
          },
          "config"_a,
          "robot_id"_a = 0)
      .def_static(
          "from_file",
          [](const std::filesystem::path& config, int robot_id) {
            const auto node = YAML::LoadFile(config);
            return std::make_unique<PythonBatchPipeline>(
                config::fromYaml<PythonBatchPipeline::Config>(node), robot_id);
          },
          "config"_a,
          "robot_id"_a = 0)
      .def("construct", &PythonBatchPipeline::construct);
}

}  // namespace python_batch

};  // namespace hydra::python
