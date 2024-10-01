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
#include "hydra/bindings/hydra_python_pipeline.h"

#include <config_utilities/formatting/asl.h>
#include <config_utilities/logging/log_to_glog.h>
#include <config_utilities/parsing/yaml.h>
#include <config_utilities/validation.h>
#include <hydra/active_window/reconstruction_module.h>
#include <hydra/backend/backend_module.h>
#include <hydra/common/global_info.h>
#include <hydra/common/pipeline_queues.h>
#include <hydra/frontend/gvd_place_extractor.h>
#include <hydra/loop_closure/loop_closure_module.h>
#include <hydra/places/compression_graph_extractor.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pybind11/stl/filesystem.h>
#include <pybind11/stl_bind.h>

#include "hydra/bindings/glog_utilities.h"
#include "hydra/bindings/python_config.h"
#include "hydra/bindings/python_image.h"
#include "hydra/bindings/python_sensor_input.h"

namespace hydra::python {

PythonCamera::PythonCamera()
    : body_R_sensor(Eigen::Matrix3d::Identity()),
      body_p_sensor(Eigen::Vector3d::Zero()) {}

config::VirtualConfig<Sensor> PythonCamera::sensor() const {
  ParamSensorExtrinsics::Config extrinsics_config;
  extrinsics_config.body_R_sensor = body_R_sensor;
  extrinsics_config.body_p_sensor = body_p_sensor;
  Camera::Config camera_config = camera;
  camera_config.extrinsics = extrinsics_config;
  config::VirtualConfig<Sensor> to_return(camera_config);
  return to_return;
}

HydraPythonPipeline::HydraPythonPipeline(const PipelineConfig& config,
                                         int robot_id,
                                         int config_verbosity,
                                         bool freeze_global_info,
                                         bool step_mode_only)
    : HydraPipeline(config, robot_id, config_verbosity, freeze_global_info),
      step_mode_only_(step_mode_only) {
  GlogSingleton::instance().setLogLevel(0, 0, false);
  config::Settings().print_width = 100;
  config::Settings().print_indent = 45;
}

HydraPythonPipeline::~HydraPythonPipeline() {}

void HydraPythonPipeline::initPython(const PythonConfig& config,
                                     const PythonCamera& camera) {
  GlobalInfo::instance().setSensor("python", camera.sensor());
  pipeline_config_ = config.toYaml();

  initModules();
  showModules();
  VLOG(config_verbosity_) << GlobalInfo::instance();
}

void HydraPythonPipeline::start() {
  if (step_mode_only_) {
    showModules();
  } else {
    HydraPipeline::start();
  }
}

void HydraPythonPipeline::initModules() {
  const auto& logs = GlobalInfo::instance().getLogs();
  frontend_ = config::createFromYamlWithNamespace<GraphBuilder>(
      pipeline_config_, "frontend", frontend_dsg_, shared_state_, logs);
  backend_ = config::createFromYamlWithNamespace<BackendModule>(
      pipeline_config_, "backend", backend_dsg_, shared_state_, logs);
  reconstruction_ = config::createFromYamlWithNamespace<ReconstructionModule>(
      pipeline_config_, "reconstruction", frontend_->queue());

  // we want to make use of the module map in the base pipeline, so we add all the
  // explicit modules here
  modules_["reconstruction"] = reconstruction_;
  modules_["frontend"] = frontend_;
  modules_["backend"] = backend_;

  if (GlobalInfo::instance().getConfig().enable_lcd) {
    auto lcd_config = config::fromYaml<LoopClosureConfig>(pipeline_config_);
    lcd_config.detector.num_semantic_classes = GlobalInfo::instance().getTotalLabels();
    config::checkValid(lcd_config);

    loop_closure_ = std::make_shared<LoopClosureModule>(lcd_config, shared_state_);
    modules_["lcd"] = loop_closure_;
  }
}

void HydraPythonPipeline::stop() {
  if (step_mode_only_) {
    return;
  }

  HydraPipeline::stop();
}

void HydraPythonPipeline::reset() {
  stop();

  // reset specific module instances
  reconstruction_.reset();
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
  shared_state_.reset(new SharedModuleState());
  shared_state_->lcd_graph = config.createSharedDsg();
  shared_state_->backend_graph = config.createSharedDsg();

  PipelineQueues::instance().clear();

  // TODO(nathan) fix local graph member

  initModules();

  if (!step_mode_only_) {
    // avoid spamming config
    start();
  }
}

void HydraPythonPipeline::save() {
  HydraPipeline::save();
  GlobalInfo::exit();
}

bool HydraPythonPipeline::step(const InputPacket::Ptr& input) {
  if (!reconstruction_->step(input)) {
    return false;
  }

  if (!frontend_->spinOnce()) {
    LOG(ERROR) << "[Hydra] Frontend failed to return output";
    return false;
  }

  backend_->step(false);
  return true;
}

DynamicSceneGraph::Ptr HydraPythonPipeline::getSceneGraph() const {
  if (step_mode_only_) {
    // no need to worry about the graph being modified when modules aren't running in
    // parallel
    return backend_dsg_->graph;
  }

  return graph_;
}

namespace hydra_python_pipeline {

using namespace pybind11::literals;
namespace py = pybind11;

void addBindings(pybind11::module_& m) {
  py::class_<Camera::Config>(m, "CameraConfig")
      .def_readwrite("min_range", &Camera::Config::min_range)
      .def_readwrite("max_range", &Camera::Config::max_range)
      .def_readwrite("width", &Camera::Config::width)
      .def_readwrite("height", &Camera::Config::height)
      .def_readwrite("cx", &Camera::Config::cx)
      .def_readwrite("cy", &Camera::Config::cy)
      .def_readwrite("fx", &Camera::Config::fx)
      .def_readwrite("fy", &Camera::Config::fy);

  py::class_<PythonCamera>(m, "PythonCamera")
      .def(py::init<>())
      .def_readwrite("intrinsics", &PythonCamera::camera)
      .def_readwrite("body_R_sensor", &PythonCamera::body_R_sensor)
      .def_readwrite("body_p_sensor", &PythonCamera::body_p_sensor);

  py::class_<HydraPythonPipeline>(m, "HydraPipeline")
      .def(py::init<const PipelineConfig&, int, int, bool, bool>(),
           "config"_a,
           "robot_id"_a = 0,
           "config_verbosity"_a = 0,
           "freeze_global_info"_a = true,
           "use_step_mode"_a = true)
      .def("init", &HydraPythonPipeline::initPython, "config"_a, "camera"_a)
      .def("save", &HydraPythonPipeline::save)
      .def("reset", &HydraPythonPipeline::reset)
      .def(
          "step",
          [](HydraPythonPipeline& pipeline,
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
                timestamp_ns, depth, labels, rgb, "python");
            input->sensor_input->input_feature = feature;
            return pipeline.step(input);
          },
          "timestamp_ns"_a,
          "world_t_body"_a,
          "world_R_body"_a,
          "depth"_a,
          "labels"_a,
          "rgb"_a,
          "features"_a = FeatureVector())
      .def(
          "step",
          [](HydraPythonPipeline& pipeline,
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
                timestamp_ns, points, labels, colors, "python");
            input->sensor_input->input_feature = feature;
            return pipeline.step(input);
          },
          "timestamp_ns"_a,
          "world_t_body"_a,
          "world_R_body"_a,
          "points"_a,
          "labels"_a,
          "colors"_a,
          "features"_a = FeatureVector())
      .def_property_readonly("graph", &HydraPythonPipeline::getSceneGraph);
}

}  // namespace hydra_python_pipeline

};  // namespace hydra::python
