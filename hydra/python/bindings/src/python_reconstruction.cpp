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
#include "hydra/bindings/python_reconstruction.h"

#include <config_utilities/config.h>
#include <config_utilities/parsing/context.h>
#include <config_utilities/validation.h>
#include <glog/logging.h>
#include <hydra/active_window/reconstruction_module.h>
#include <hydra/common/global_info.h>
#include <kimera_pgmo/compression/delta_compression.h>
#include <kimera_pgmo/mesh_offset_info.h>
#include <kimera_pgmo/utils/mesh_io.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pybind11/stl/filesystem.h>
#include <pybind11/stl_bind.h>
#include <spark_dsg/scene_graph.h>
#include <spark_dsg/zmq_interface.h>

#include "hydra/bindings/glog_utilities.h"
#include "hydra/bindings/python_sensor_input.h"
#include "hydra/utils/data_directory.h"
#include "hydra/utils/logging.h"

namespace hydra::python {
namespace {

ReconstructionModule::Config default_config() {
  ReconstructionModule::Config config;
  config.volumetric_map = VolumetricMap::Config{0.1, 16, 0.3, false, false};
  return config;
}

}  // namespace

using namespace spark_dsg;

class PythonReconstruction {
 public:
  struct Config : VerbosityConfig {
    Config() : VerbosityConfig("[python_reconstruction] ") {}
    ReconstructionModule::Config reconstruction = default_config();
  } const config;

  PythonReconstruction(const Config& config, const Sensor::Ptr& sensor);

  virtual ~PythonReconstruction();

  bool step(const std::shared_ptr<InputPacket>& input);

  void save(const std::filesystem::path& output);

  void stop();

  const std::string sensor_name;

 protected:
  std::shared_ptr<ReconstructionModule> module_;
};

void declare_config(PythonReconstruction::Config& config) {
  using namespace config;
  name("PythonReconstructionConfig");
  base<VerbosityConfig>(config);
  field(config.reconstruction, "reconstruction");
}

PythonReconstruction::PythonReconstruction(const Config& config,
                                           const Sensor::Ptr& sensor)
    : config(config::checkValid(config)), sensor_name(sensor ? sensor->name : "") {
  if (!sensor) {
    throw std::runtime_error("invalid sensor!");
  }

  GlobalInfo::instance().setSensor(sensor);

  module_ = std::make_shared<ReconstructionModule>(config.reconstruction, nullptr);
  if (!module_) {
    throw std::runtime_error("could not create reconstruction module");
  }

  MLOG(1) << GlobalInfo::instance();
  MLOG(1) << module_->printInfo();
}

void PythonReconstruction::stop() {}

PythonReconstruction::~PythonReconstruction() { stop(); }

bool PythonReconstruction::step(const std::shared_ptr<InputPacket>& input) {
  return module_->step(input);
}

void PythonReconstruction::save(const std::filesystem::path& output) {
  if (output.empty()) {
    return;
  }

  DataDirectory logs(output);
  if (logs.valid()) {
    module_->map().save(logs.path("map"));
  }

  stop();
}

namespace python_reconstruction {

using namespace pybind11::literals;
namespace py = pybind11;

void addBindings(pybind11::module_& m) {
  py::class_<PythonReconstruction>(m, "ReconstructionPipeline")
      .def(py::init([](const Sensor::Ptr& sensor, const std::string& ns) {
             const auto config = config::fromContext<PythonReconstruction::Config>(ns);
             return std::make_unique<PythonReconstruction>(config, sensor);
           }),
           "sensor"_a,
           "ns"_a = "")
      .def("save", &PythonReconstruction::save)
      .def("stop", &PythonReconstruction::stop)
      .def(
          "step",
          [](PythonReconstruction& pipeline,
             size_t timestamp_ns,
             const Eigen::Vector4d& world_R_body,
             const Eigen::Vector3d& world_t_body,
             const py::buffer& rgb,
             const py::buffer& depth) {
            auto sensor_input = std::make_unique<PythonSensorInput>(
                timestamp_ns, depth, PythonImage(), rgb, pipeline.sensor_name);

            auto input = std::make_shared<InputPacket>();
            input->timestamp_ns = timestamp_ns;
            input->world_t_body = world_t_body;
            input->world_R_body = Eigen::Quaterniond(
                world_R_body[0], world_R_body[1], world_R_body[2], world_R_body[3]);
            input->sensor_input = std::move(sensor_input);
            return pipeline.step(input);
          },
          "timestamp_ns"_a,
          "world_R_body"_a,
          "world_t_body"_a,
          "rgb"_a,
          "depth"_a);
}

}  // namespace python_reconstruction
}  // namespace hydra::python
