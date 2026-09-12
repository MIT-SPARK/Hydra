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
#include <config_utilities/printing.h>
#include <config_utilities/types/path.h>
#include <config_utilities/validation.h>
#include <glog/logging.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pybind11/stl/filesystem.h>

#include "hydra/active_window/reconstruction_module.h"
#include "hydra/bindings/glog_utilities.h"
#include "hydra/bindings/python_sensor_input.h"
#include "hydra/common/global_info.h"
#include "hydra/frontend/mesh_compression.h"
#include "hydra/input/input_filter.h"
#include "hydra/utils/data_directory.h"
#include "hydra/utils/logging.h"
#include "hydra/utils/pgmo_mesh_traits.h"  // IWYU pragma: keep

using namespace spark_dsg;
namespace py = pybind11;

namespace hydra::python {
namespace {

ReconstructionModule::Config default_config() {
  ReconstructionModule::Config config;
  config.volumetric_map = VolumetricMap::Config{0.1, 16, 0.3, false, false};
  return config;
}

}  // namespace

class PythonReconstruction {
 public:
  struct Config : VerbosityConfig {
    Config() : VerbosityConfig("[python_reconstruction] ") {}

    ReconstructionModule::Config reconstruction = default_config();
    std::vector<config::VirtualConfig<InputFilter, true>> filters;
    config::VirtualConfig<MeshCompressor> compression{MeshCompression::Config{}};
  } const config;

  PythonReconstruction(const Config& config, const Sensor::Ptr& sensor);

  bool step(size_t timestamp_ns,
            const Eigen::Vector4d& odom_R_body,
            const Eigen::Vector3d& odom_t_body,
            const py::buffer& rgb,
            const py::buffer& depth);

  bool stepImpl(const std::shared_ptr<SensorInputPacket>& packet,
                const Eigen::Isometry3d& world_T_body);

  void save(const std::filesystem::path& output);

  Mesh::Ptr mesh() const;

  const Sensor::ConstPtr sensor;

 protected:
  SensorInputPacket::Ptr last_input_;
  ActiveWindowModule::OutputQueue::Ptr queue_;
  std::vector<std::unique_ptr<InputFilter>> filters_;
  std::shared_ptr<ReconstructionModule> module_;

  Mesh::Ptr mesh_;
  kimera_pgmo::MeshOffsetInfo offsets_;
  std::unique_ptr<MeshCompressor> compressor_;
};

void declare_config(PythonReconstruction::Config& config) {
  using namespace config;
  name("PythonReconstructionConfig");
  base<VerbosityConfig>(config);
  field(config.reconstruction, "reconstruction");
  field(config.filters, "filters");
  field(config.compression, "compression");
}

PythonReconstruction::PythonReconstruction(const Config& config,
                                           const Sensor::Ptr& sensor)
    : config(config::checkValid(config)),
      sensor(sensor),
      compressor_(config.compression.create()) {
  if (!sensor) {
    throw std::runtime_error("invalid sensor!");
  }

  for (const auto& filter : config.filters) {
    filters_.push_back(filter.create());
  }

  queue_ = std::make_shared<ActiveWindowModule::OutputQueue>();
  module_ = std::make_shared<ReconstructionModule>(config.reconstruction, queue_);
  if (compressor_) {
    mesh_ = std::make_shared<Mesh>();
  }

  MLOG(2) << "\n" << config::toString(GlobalInfo::instance().getConfig());
  MLOG(1) << "\n" << module_->printInfo();
}

bool PythonReconstruction::step(size_t timestamp_ns,
                                const Eigen::Vector4d& odom_R_body,
                                const Eigen::Vector3d& odom_t_body,
                                const py::buffer& rgb,
                                const py::buffer& depth) {
  auto packet = std::make_shared<PythonImageInput>(timestamp_ns, rgb, depth);
  const auto rotation = Eigen::Quaterniond(
      odom_R_body[0], odom_R_body[1], odom_R_body[2], odom_R_body[3]);
  const auto translation = Eigen::Translation3d(odom_t_body);
  const auto odom_T_body = Eigen::Isometry3d(translation * rotation);
  return stepImpl(packet, odom_T_body);
}

bool PythonReconstruction::stepImpl(const std::shared_ptr<SensorInputPacket>& packet,
                                    const Eigen::Isometry3d& odom_T_body) {
  for (const auto& filter : filters_) {
    if (!filter) {
      continue;
    }

    if (!filter->valid(*packet, last_input_.get())) {
      LOG(ERROR) << "Skipping input!";
      return false;
    }
  }

  last_input_ = packet;

  auto data = std::make_shared<InputData>(sensor);
  data->timestamp_ns = packet->timestamp_ns;
  data->world_T_body = odom_T_body;
  packet->fillInputData(*data);

  if (!module_->step(data)) {
    return false;
  }

  const auto output = queue_->pop();
  if (!output) {
    return false;
  }

  const auto delta = compressor_->update(*output, module_->window());
  delta->updateMesh(*mesh_, offsets_);
  return true;
}

void PythonReconstruction::save(const std::filesystem::path& output) {
  if (output.empty()) {
    return;
  }

  DataDirectory logs(output);
  if (!logs) {
    return;
  }

  module_->map().save(logs.path() / "map");
  if (mesh_) {
    mesh_->save(logs.path() / "mesh.sparkdsg");
  }
}

Mesh::Ptr PythonReconstruction::mesh() const { return mesh_; }

namespace python_reconstruction {

using namespace pybind11::literals;

void addBindings(pybind11::module_& m) {
  py::class_<PythonReconstruction>(m, "ReconstructionPipeline")
      .def(py::init([](const Sensor::Ptr& sensor, const std::string& ns) {
             const auto config = config::fromContext<PythonReconstruction::Config>(ns);
             return std::make_unique<PythonReconstruction>(config, sensor);
           }),
           "sensor"_a,
           "ns"_a = "")
      .def("save", &PythonReconstruction::save)
      .def_property_readonly("mesh", &PythonReconstruction::mesh)
      .def("step",
           &PythonReconstruction::step,
           "timestamp_ns"_a,
           "odom_R_body"_a,
           "odom_t_body"_a,
           "rgb"_a,
           "depth"_a);
}

}  // namespace python_reconstruction
}  // namespace hydra::python
