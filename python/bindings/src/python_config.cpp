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
#include "hydra/bindings/python_config.h"

#include <config_utilities/internal/yaml_utils.h>
#include <pybind11/stl.h>
#include <pybind11/stl/filesystem.h>

namespace hydra::python {

ConfigOverlay::ConfigOverlay(const std::string& _config_ns,
                             const std::string& _filepath,
                             const std::string& _contents) {
  config_ns = _config_ns;
  if (!_filepath.empty()) {
    filepath = std::make_shared<std::string>(_filepath);
  }
  if (!_contents.empty()) {
    contents = std::make_shared<std::string>(_contents);
  }
}

PythonConfig::PythonConfig(const std::vector<ConfigOverlay>& configs)
    : configs_(configs) {}

void PythonConfig::add(const ConfigOverlay& config) { configs_.push_back(config); }

void PythonConfig::addFile(const std::string& filepath, const std::string& config_ns) {
  ConfigOverlay overlay{config_ns, filepath, ""};
  configs_.push_back(overlay);
}

void PythonConfig::addYaml(const std::string& contents, const std::string& config_ns) {
  ConfigOverlay overlay{config_ns, "", contents};
  configs_.push_back(overlay);
}

YAML::Node PythonConfig::toYaml() const {
  YAML::Node to_return;
  for (const auto& conf : configs_) {
    if (conf.filepath) {
      auto new_node = YAML::LoadFile(*conf.filepath);
      config::internal::moveDownNamespace(new_node, conf.config_ns);
      config::internal::mergeYamlNodes(to_return, new_node);
    }

    if (conf.contents) {
      auto new_node = YAML::Load(*conf.contents);
      config::internal::moveDownNamespace(new_node, conf.config_ns);
      config::internal::mergeYamlNodes(to_return, new_node);
    }
  }

  return to_return;
}

namespace python_config {

using namespace pybind11::literals;
namespace py = pybind11;

void addBindings(pybind11::module_& m) {
  // GCOVR_EXCL_START
  py::class_<ConfigOverlay>(m, "ConfigOverlay")
      .def(py::init([](const std::string& config_ns,
                       const std::string& filepath,
                       const std::string& contents) {
             return ConfigOverlay(config_ns, filepath, contents);
           }),
           "config_ns"_a = "",
           "filepath"_a = "",
           "contents"_a = "")
      .def(py::init([](const std::string& config_ns,
                       const std::filesystem::path& filepath,
                       const std::string& contents) {
             return ConfigOverlay(config_ns, filepath.string(), contents);
           }),
           "config_ns"_a = "",
           "filepath"_a = "",
           "contents"_a = "");
  // GCOVR_EXCL_STOP

  py::class_<PythonConfig>(m, "PythonConfig")
      .def(py::init([]() { return PythonConfig({}); }))
      .def(py::init<const std::vector<ConfigOverlay>&>())
      .def("add", &PythonConfig::add)
      .def("add_file", &PythonConfig::addFile, "filepath"_a, "config_ns"_a = "")
      .def(
          "add_file",
          [](PythonConfig& config,
             const std::filesystem::path& conf_file,
             const std::string config_ns) {
            config.addFile(conf_file.string(), config_ns);
          },
          "filepath"_a,
          "config_ns"_a = "")
      .def("add_yaml", &PythonConfig::addYaml, "contents"_a, "config_ns"_a = "")
      .def("__repr__",
           [](const PythonConfig& conf) { return YAML::Dump(conf.toYaml()); });
}

}  // namespace python_config

}  // namespace hydra::python
