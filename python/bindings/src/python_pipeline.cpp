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
#include <config_utilities/parsing/yaml.h>
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

#include "hydra/bindings/glog_utilities.h"
#include "hydra/bindings/python_image.h"
#include "hydra/bindings/python_sensor_input.h"
#include "hydra/bindings/python_sensors.h"

namespace hydra::python {

class PythonPipeline : public HydraPipeline {
 public:
  struct Config : PipelineConfig {
    config::VirtualConfig<ActiveWindowModule> active_window{
        ReconstructionModule::Config()};
    config::VirtualConfig<GraphBuilder> frontend{GraphBuilder::Config()};
    config::VirtualConfig<BackendModule> backend{BackendModule::Config()};
  } const config;

  PythonPipeline(const Config& config,
                 const Sensor::Ptr& sensor,
                 int robot_id = 0,
                 int config_verbosity = 0,
                 bool freeze_global_info = true,
                 bool step_mode_only = true,
                 std::string zmq_url = "");

  virtual ~PythonPipeline();

  void start() override;

  void stop() override;

  void save() override;

  void reset();

  bool step(const InputPacket::Ptr& input);

  DynamicSceneGraph::Ptr getSceneGraph() const;

  const std::string sensor_name;
  const std::string zmq_url;

 protected:
  bool step_mode_only_;
  std::shared_ptr<ActiveWindowModule> active_window_;
  std::shared_ptr<GraphBuilder> frontend_;
  std::shared_ptr<BackendModule> backend_;
  std::shared_ptr<LoopClosureModule> loop_closure_;

  DynamicSceneGraph::Ptr graph_;

 private:
  void initModules();
};

void declare_config(PythonPipeline::Config& config) {
  using namespace config;
  name("PythonPipeline::Config");
  base<PipelineConfig>(config);
  field(config.active_window, "active_window");
  field(config.frontend, "frontend");
  field(config.backend, "backend");
}

PythonPipeline::PythonPipeline(const Config& _config,
                               const Sensor::Ptr& sensor,
                               int robot_id,
                               int config_verbosity,
                               bool freeze_global_info,
                               bool step_mode_only,
                               std::string zmq_url)
    : HydraPipeline(_config, robot_id, config_verbosity, freeze_global_info),
      config(_config),
      sensor_name(sensor ? sensor->name : ""),
      zmq_url(zmq_url),
      step_mode_only_(step_mode_only) {
  if (!sensor) {
    throw std::runtime_error("Invalid sensor!");
  }

  GlogSingleton::instance().setLogLevel(0, 0, false);
  config::Settings().print_width = 100;
  config::Settings().print_indent = 45;

  GlobalInfo::instance().setSensor(sensor);
  VLOG(config_verbosity) << "\n"
                         << config::toString(GlobalInfo::instance().getConfig());

  initModules();
}

PythonPipeline::~PythonPipeline() { stop(); }

void PythonPipeline::start() {
  if (step_mode_only_) {
    LOG(INFO) << "[Hydra Python] Running in step mode!";
  } else {
    LOG(INFO) << "[Hydra Python] Running in parallel";
    HydraPipeline::start();
  }
}

void PythonPipeline::initModules() {
  const auto& logs = GlobalInfo::instance().getLogs();
  frontend_ = config.frontend.create(frontend_dsg_, shared_state_, logs);
  if (!frontend_) {
    throw std::runtime_error("Invalid frontend config!");
  }

  modules_["frontend"] = frontend_;
  backend_ = config.backend.create(backend_dsg_, shared_state_, logs);
  modules_["backend"] = backend_;
  active_window_ = config.active_window.create(frontend_->queue());
  modules_["reconstruction"] = active_window_;

  if (!zmq_url.empty()) {
    ZmqSink::Config zmq_config{zmq_url, true};
    backend_->addSink(std::make_shared<ZmqSink>(zmq_config));
  }

  // TODO(nathan) LCD

  showModules();
  VLOG(config_verbosity_) << GlobalInfo::instance();
}

void PythonPipeline::stop() {
  if (step_mode_only_) {
    return;
  }

  HydraPipeline::stop();
}

void PythonPipeline::reset() {
  stop();

  // reset specific module instances
  active_window_.reset();
  frontend_.reset();
  backend_.reset();
  loop_closure_.reset();

  // reset any other modules (will actually deconstruct specific module instances given
  // shared_ptr usage)
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
    // avoid spamming config
    start();
  }
}

void PythonPipeline::save() {
  HydraPipeline::save();
  GlobalInfo::exit();
}

bool PythonPipeline::step(const InputPacket::Ptr& input) {
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

DynamicSceneGraph::Ptr PythonPipeline::getSceneGraph() const {
  return backend_dsg_->graph->clone();
}

namespace python_pipeline {

using namespace pybind11::literals;
namespace py = pybind11;

void addBindings(pybind11::module_& m) {
  py::class_<PythonPipeline>(m, "HydraPipeline")
      .def_static(
          "from_config",
          [](const std::string& contents,
             const Sensor::Ptr& sensor,
             int robot_id,
             int config_verbosity,
             bool freeze_global_info,
             bool step_mode_only,
             const std::string zmq_url) {
            const auto node = YAML::Load(contents);
            return std::make_unique<PythonPipeline>(
                config::fromYaml<PythonPipeline::Config>(node),
                sensor,
                robot_id,
                config_verbosity,
                freeze_global_info,
                step_mode_only,
                zmq_url);
          },
          "config"_a,
          "camera"_a,
          "robot_id"_a = 0,
          "config_verbosity"_a = 0,
          "freeze_global_info"_a = true,
          "use_step_mode"_a = true,
          "zmq_url"_a = "")
      .def_static(
          "from_file",
          [](const std::filesystem::path& filepath,
             const Sensor::Ptr& sensor,
             int robot_id,
             int config_verbosity,
             bool freeze_global_info,
             bool step_mode_only,
             const std::string& zmq_url) {
            const auto node = YAML::LoadFile(filepath);
            return std::make_unique<PythonPipeline>(
                config::fromYaml<PythonPipeline::Config>(node),
                sensor,
                robot_id,
                config_verbosity,
                freeze_global_info,
                step_mode_only,
                zmq_url);
          },
          "config"_a,
          "camera"_a,
          "robot_id"_a = 0,
          "config_verbosity"_a = 0,
          "freeze_global_info"_a = true,
          "use_step_mode"_a = true,
          "zmq_url"_a = "")
      .def_static("default_config",
                  []() {
                    std::stringstream ss;
                    ss << config::toYaml(PythonPipeline::Config());
                    return ss.str();
                  })
      .def("save", &PythonPipeline::save)
      .def("reset", &PythonPipeline::reset)
      .def(
          "step",
          [](PythonPipeline& pipeline,
             size_t timestamp_ns,
             const Eigen::Vector3d& world_t_body,
             const Eigen::Vector4d& world_R_body,
             const py::buffer& depth,
             const py::buffer& labels,
             const py::buffer& rgb,
             const FeatureVector& feature) {
            auto input = std::make_shared<InputPacket>();
            input->timestamp_ns = timestamp_ns;
            input->world_t_body = world_t_body;
            input->world_R_body = Eigen::Quaterniond(
                world_R_body[0], world_R_body[1], world_R_body[2], world_R_body[3]);
            input->sensor_input = std::make_unique<PythonSensorInput>(
                timestamp_ns, depth, labels, rgb, pipeline.sensor_name);
            input->sensor_input->input_feature = feature;
            return pipeline.step(input);
          },
          "timestamp_ns"_a,
          "world_t_body"_a,
          "world_R_body"_a,
          "depth"_a,
          "labels"_a,
          "rgb"_a,
          "feature"_a = FeatureVector())
      .def(
          "step",
          [](PythonPipeline& pipeline,
             size_t timestamp_ns,
             const Eigen::Vector3d& world_t_body,
             const Eigen::Vector4d& world_R_body,
             const PythonSensorInput::PointVec& points,
             const PythonSensorInput::LabelVec& labels,
             const PythonSensorInput::ColorVec& colors,
             const FeatureVector& feature) {
            auto input = std::make_shared<InputPacket>();
            input->timestamp_ns = timestamp_ns;
            input->world_t_body = world_t_body;
            input->world_R_body = Eigen::Quaterniond(
                world_R_body[0], world_R_body[1], world_R_body[2], world_R_body[3]);
            input->sensor_input = std::make_unique<PythonSensorInput>(
                timestamp_ns, points, labels, colors, pipeline.sensor_name);
            input->sensor_input->input_feature = feature;
            return pipeline.step(input);
          },
          "timestamp_ns"_a,
          "world_t_body"_a,
          "world_R_body"_a,
          "points"_a,
          "labels"_a,
          "colors"_a,
          "feature"_a = FeatureVector())
      .def_property_readonly("graph", &PythonPipeline::getSceneGraph);
}

}  // namespace python_pipeline

};  // namespace hydra::python
