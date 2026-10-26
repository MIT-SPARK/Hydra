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
#include "hydra/bindings/python_pipeline.h"

#include <config_utilities/config.h>
#include <config_utilities/parsing/context.h>
#include <config_utilities/printing.h>
#include <config_utilities/validation.h>
#include <glog/logging.h>
#include <hydra/active_window/reconstruction_module.h>
#include <hydra/backend/backend_module.h>
#include <hydra/backend/zmq_interfaces.h>
#include <hydra/common/global_info.h>
#include <hydra/common/hydra_pipeline.h>
#include <hydra/common/pipeline_queues.h>
#include <hydra/frontend/graph_builder.h>
#include <hydra/input/camera.h>
#include <hydra/loop_closure/loop_closure_module.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pybind11/stl/filesystem.h>
#include <pybind11/stl_bind.h>

#include <filesystem>

#include "hydra/bindings/glog_utilities.h"
#include "hydra/bindings/python_sensor_input.h"
#include "hydra/bindings/python_sensors.h"
#include "hydra/input/input_filter.h"

using namespace spark_dsg;

namespace hydra::python {

class PythonPipeline : public HydraPipeline {
 public:
  struct Config : HydraPipeline::Config {
    template <typename T>
    using VirtualConfig = config::VirtualConfig<T>;
    std::vector<config::VirtualConfig<InputFilter, true>> filters;
  } const config;

  PythonPipeline(const Config& config,
                 const Sensor::Ptr& sensor,
                 std::unique_ptr<Labelspace>&& labelspace);

  virtual ~PythonPipeline() = default;

  void start() override;

  void reset();

  bool step(const std::shared_ptr<SensorInputPacket>& packet,
            const Eigen::Isometry3d& world_T_body);

  const Sensor::ConstPtr sensor;

 protected:
  bool step_mode_only_;
  SensorInputPacket::Ptr last_input_;
  std::vector<std::unique_ptr<InputFilter>> filters_;

 private:
  void initModules();
};

void declare_config(PythonPipeline::Config& config) {
  using namespace config;
  name("PythonPipeline::Config");
  base<HydraPipeline::Config>(config);
}

PythonPipeline::PythonPipeline(const Config& _config,
                               const Sensor::Ptr& sensor,
                               std::unique_ptr<Labelspace>&& labelspace)
    : HydraPipeline(_config, std::move(labelspace)), config(_config), sensor(sensor) {
  if (!sensor) {
    throw std::runtime_error("Invalid sensor!");
  }

  for (const auto& filter : config.filters) {
    filters_.push_back(filter.create());
  }

  MLOG(0) << "Using sensor '" << sensor->name << "':\n" << sensor->dump();
  initModules();
}

void PythonPipeline::start() {
  if (step_mode_only_) {
    LOG(INFO) << "Running in step mode!";
  } else {
    LOG(INFO) << "Running in parallel";
    HydraPipeline::start();
  }
}

void PythonPipeline::initModules() {
  init();
  showModules();
}

void PythonPipeline::reset() {
  stop();

  initState();  // resets scene graphs
  PipelineQueues::instance().clear();

  initModules();
  if (!step_mode_only_) {
    start();
  }
}

bool PythonPipeline::step(const std::shared_ptr<SensorInputPacket>& packet,
                          const Eigen::Isometry3d& odom_T_body) {
  auto input = std::make_shared<InputData>(sensor);
  input->timestamp_ns = packet->timestamp_ns;
  input->world_T_body = odom_T_body;
  packet->fillInputData(*input);

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

  if (!active_window_->step(input)) {
    return false;
  }

  if (!frontend_->spinOnce()) {
    LOG(ERROR) << "[Hydra] Frontend failed to return output";
    return false;
  }

  backend_->step(false);
  return true;
}

namespace python_pipeline {

using namespace pybind11::literals;
namespace py = pybind11;

void addBindings(pybind11::module_& m) {
  namespace fs = std::filesystem;
  py::class_<PythonPipeline>(m, "HydraPipeline")
      .def(
          py::init([](const Sensor::Ptr& sensor, const Labelspace& labelspace, int id) {
            const auto config = config::fromContext<PythonPipeline::Config>();
            const auto global_config = config::fromContext<PipelineConfig>();
            GlobalInfo::init(global_config, id);
            return std::make_unique<PythonPipeline>(
                config, sensor, std::make_unique<Labelspace>(labelspace));
          }),
          "sensor"_a,
          "labelspace"_a = Labelspace{},
          "robot_id"_a = 0)
      .def("start", &PythonPipeline::start)
      .def("stop", &PythonPipeline::stop)
      .def(
          "save",
          [](const PythonPipeline& pipeline, const fs::path& output) {
            pipeline.save(DataDirectory(output));
          },
          "output"_a)
      .def("reset", &PythonPipeline::reset)
      .def(
          "step",
          [](PythonPipeline& pipeline,
             size_t stamp,
             const Eigen::Vector4d& rot,
             const Eigen::Vector3d& pos,
             const py::buffer& rgb,
             const py::buffer& depth,
             const FeatureVector& feature) {
            const Eigen::Quaterniond q(rot[0], rot[1], rot[2], rot[3]);
            const Eigen::Isometry3d pose = Eigen::Translation<double, 3>(pos) * q;
            auto packet = std::make_shared<PythonImageInput>(stamp, rgb, depth);
            packet->input_feature = feature;

            return pipeline.step(packet, pose);
          },
          "timestamp_ns"_a,
          "odom_R_body"_a,
          "odom_t_body"_a,
          "rgb"_a,
          "depth"_a,
          "feature"_a = FeatureVector())
      .def(
          "step",
          [](PythonPipeline& pipeline,
             size_t stamp,
             const Eigen::Vector4d& rot,
             const Eigen::Vector3d& pos,
             const py::buffer& rgb,
             const py::buffer& depth,
             const py::buffer& labels,
             const FeatureVector& feature) {
            const Eigen::Quaterniond q(rot[0], rot[1], rot[2], rot[3]);
            const Eigen::Isometry3d pose = Eigen::Translation<double, 3>(pos) * q;
            auto packet = std::make_shared<PythonImageInput>(stamp, rgb, depth, labels);
            packet->input_feature = feature;

            return pipeline.step(packet, pose);
          },
          "timestamp_ns"_a,
          "odom_R_body"_a,
          "odom_t_body"_a,
          "rgb"_a,
          "depth"_a,
          "labels"_a,
          "feature"_a = FeatureVector());
}

}  // namespace python_pipeline
}  // namespace hydra::python
