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
  struct Config : PipelineConfig {
    template <typename T>
    using VirtualConfig = config::VirtualConfig<T>;
    std::vector<config::VirtualConfig<InputFilter, true>> filters;
    VirtualConfig<ActiveWindowModule> active_window{ReconstructionModule::Config()};
    VirtualConfig<GraphBuilder> frontend{GraphBuilder::Config()};
    VirtualConfig<BackendModule> backend{BackendModule::Config()};
    VirtualConfig<LoopClosureModule> lcd;
  } const config;

  PythonPipeline(const Config& config,
                 const Sensor::Ptr& sensor,
                 int robot_id = 0,
                 int config_verbosity = 0,
                 bool step_mode_only = true);

  virtual ~PythonPipeline();

  void start() override;

  void stop() override;

  void reset();

  bool step(const std::shared_ptr<SensorInputPacket>& packet,
            const Eigen::Isometry3d& world_T_body);

  SceneGraph::Ptr getSceneGraph() const;

  const Sensor::ConstPtr sensor;

 protected:
  bool step_mode_only_;
  SensorInputPacket::Ptr last_input_;
  std::vector<std::unique_ptr<InputFilter>> filters_;
  std::shared_ptr<ActiveWindowModule> active_window_;
  std::shared_ptr<GraphBuilder> frontend_;
  std::shared_ptr<BackendModule> backend_;
  std::shared_ptr<LoopClosureModule> loop_closure_;

 private:
  void initModules();
};

void declare_config(PythonPipeline::Config& config) {
  using namespace config;
  name("PythonPipeline::Config");
  base<PipelineConfig>(config);
  field(config.active_window, "active_window");
  config.frontend.setOptional();
  field(config.frontend, "frontend");
  config.backend.setOptional();
  field(config.backend, "backend");
  config.lcd.setOptional();
  field(config.lcd, "lcd");
}

PythonPipeline::PythonPipeline(const Config& _config,
                               const Sensor::Ptr& sensor,
                               int robot_id,
                               int config_verbosity,
                               bool step_mode_only)
    : HydraPipeline(_config, robot_id, config_verbosity),
      config(_config),
      sensor(sensor),
      step_mode_only_(step_mode_only) {
  if (!sensor) {
    throw std::runtime_error("Invalid sensor!");
  }

  for (const auto& filter : config.filters) {
    filters_.push_back(filter.create());
  }

  VLOG(config_verbosity) << "Using sensor '" << sensor->name << "':\n"
                         << sensor->dump();
  initModules();
}

PythonPipeline::~PythonPipeline() { stop(); }

void PythonPipeline::start() {
  if (step_mode_only_) {
    LOG(INFO) << "Running in step mode!";
  } else {
    LOG(INFO) << "Running in parallel";
    HydraPipeline::start();
  }
}

void PythonPipeline::initModules() {
  frontend_ = config.frontend.create(frontend_dsg_, shared_state_);
  backend_ = config.backend.create(backend_dsg_, shared_state_);
  loop_closure_ = config.lcd.create(shared_state_);

  active_window_ =
      config.active_window.create(frontend_ ? frontend_->queue() : nullptr);
  modules_["reconstruction"] = active_window_;
  if (frontend_) {
    modules_["frontend"] = frontend_;
  }

  if (frontend_ && backend_) {
    modules_["backend"] = backend_;
  }

  if (frontend_ && loop_closure_) {
    frontend_->setLcdQueue(loop_closure_->queue());
    modules_["lcd"] = loop_closure_;
  }
}

void PythonPipeline::stop() {
  if (step_mode_only_) {
    return;
  }

  HydraPipeline::stop();
}

void PythonPipeline::reset() {
  stop();

  active_window_.reset();
  frontend_.reset();
  backend_.reset();
  loop_closure_.reset();
  modules_.clear();

  // reset state
  const auto& config = GlobalInfo::instance();
  frontend_dsg_ = config.createSharedDsg();
  backend_dsg_ = config.createSharedDsg();

  // setup dependent graphs
  shared_state_.reset(new SharedModuleState());
  shared_state_->lcd_graph = config.createSharedDsg();
  shared_state_->backend_graph = config.createSharedDsg();

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

SceneGraph::Ptr PythonPipeline::getSceneGraph() const {
  return backend_dsg_->graph->clone();
}

namespace python_pipeline {

using namespace pybind11::literals;
namespace py = pybind11;

void addBindings(pybind11::module_& m) {
  namespace fs = std::filesystem;
  py::class_<PythonPipeline>(m, "HydraPipeline")
      .def(py::init([](const Sensor::Ptr& sensor, int id, bool step_mode) {
             const auto config = config::fromContext<PythonPipeline::Config>();
             return std::make_unique<PythonPipeline>(config, sensor, id, step_mode);
           }),
           "sensor"_a,
           "robot_id"_a = 0,
           "use_step_mode"_a = true)
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
             size_t timestamp_ns,
             const Eigen::Vector4d& odom_R_body,
             const Eigen::Vector3d& odom_t_body,
             const py::buffer& rgb,
             const py::buffer& depth,
             const py::buffer& labels,
             const FeatureVector& feature) {
            auto packet =
                std::make_shared<PythonImageInput>(timestamp_ns, rgb, depth, labels);
            packet->input_feature = feature;
            const Eigen::Quaterniond q(
                odom_R_body[0], odom_R_body[1], odom_R_body[2], odom_R_body[3]);
            const Eigen::Isometry3d odom_T_body =
                Eigen::Translation<double, 3>(odom_t_body) * q;
            return pipeline.step(packet, odom_T_body);
          },
          "timestamp_ns"_a,
          "odom_R_body"_a,
          "odom_t_body"_a,
          "rgb"_a,
          "depth"_a,
          "labels"_a = py::buffer(),
          "feature"_a = FeatureVector());
}

}  // namespace python_pipeline

};  // namespace hydra::python
