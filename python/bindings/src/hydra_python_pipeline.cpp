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

#include <config_utilities/config.h>
#include <config_utilities/formatting/asl.h>
#include <config_utilities/logging/log_to_glog.h>
#include <config_utilities/parsing/yaml.h>
#include <config_utilities/validation.h>
#include <hydra/backend/backend_module.h>
#include <hydra/common/hydra_config.h>
#include <hydra/loop_closure/loop_closure_module.h>
#include <hydra/reconstruction/reconstruction_module.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pybind11/stl/filesystem.h>
#include <pybind11/stl_bind.h>

#include "hydra/bindings/glog_utilities.h"
#include "hydra/bindings/python_config.h"
#include "hydra/bindings/python_image.h"
#include "hydra/bindings/python_sensor_input.h"

namespace hydra::python {

HydraPythonPipeline::HydraPythonPipeline(const PipelineConfig& config,
                                         int robot_id,
                                         int config_verbosity,
                                         bool step_mode_only)
    : HydraPipeline(config, robot_id, config_verbosity),
      step_mode_only_(step_mode_only) {
  GlogSingleton::instance().setLogLevel(0, 0, false);
  config::Settings().setLogger("glog");
  config::Settings().print_width = 100;
  config::Settings().print_indent = 45;
}

HydraPythonPipeline::~HydraPythonPipeline() {}

void HydraPythonPipeline::initPython(const PythonConfig& config) {
  const auto& logs = HydraConfig::instance().getLogs();
  const auto node = config.toYaml();
  frontend_ = config::createFromYamlWithNamespace<FrontendModule>(
      node, "frontend", frontend_dsg_, shared_state_, logs);
  backend_ = config::createFromYamlWithNamespace<BackendModule>(
      node, "backend", frontend_dsg_, backend_dsg_, shared_state_, logs);
  reconstruction_ = config::createFromYamlWithNamespace<ReconstructionModule>(
      node, "reconstruction", frontend_->getQueue());

  // we want to make use of the module map in the base pipeline, so we add all the
  // explicit modules here
  modules_["reconstruction"] = reconstruction_;
  modules_["frontend"] = frontend_;
  modules_["backend"] = backend_;

  if (HydraConfig::instance().getConfig().enable_lcd) {
    auto lcd_config = config::fromYaml<LoopClosureConfig>(node);
    lcd_config.detector.num_semantic_classes = HydraConfig::instance().getTotalLabels();
    config::checkValid(lcd_config);

    shared_state_->lcd_queue.reset(new InputQueue<LcdInput::Ptr>());
    loop_closure_ =
        std::make_shared<LoopClosureModule>(lcd_config, frontend_dsg_, shared_state_);
    modules_["lcd"] = loop_closure_;
  }

  showModules();
  VLOG(config_verbosity_) << HydraConfig::instance();
}

void HydraPythonPipeline::start() {
  if (step_mode_only_) {
    showModules();
  } else {
    HydraPipeline::start();
  }
}

void HydraPythonPipeline::stop() {
  if (step_mode_only_) {
    return;
  }

  HydraPipeline::stop();
}

void HydraPythonPipeline::save() {
  HydraPipeline::save();
  HydraConfig::exit();
}

bool HydraPythonPipeline::spinOnce(const ReconstructionInput& input) {
  if (!reconstruction_->spinOnce(input)) {
    return false;
  }

  if (!frontend_->spinOnce()) {
    LOG(ERROR) << "[Hydra] Frontend failed to return output";
    return false;
  }

  backend_->spinOnce(true);
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
  py::class_<HydraPythonPipeline>(m, "HydraPipeline")
      .def(py::init<const PipelineConfig&, int, int, bool>(),
           "config"_a,
           "robot_id"_a = 0,
           "config_verbosity"_a = 0,
           "use_step_mode"_a = true)
      .def("init", &HydraPythonPipeline::initPython, "config"_a)
      .def("save", &HydraPythonPipeline::save)
      .def("step",
           [](HydraPythonPipeline& pipeline,
              size_t timestamp_ns,
              const Eigen::Vector3d& world_t_body,
              const Eigen::Vector4d& world_R_body,
              const py::buffer& depth,
              const py::buffer& labels,
              const py::buffer& rgb) {
             auto input = std::make_shared<ReconstructionInput>();
             input->timestamp_ns = timestamp_ns;
             input->world_t_body = world_t_body;
             input->world_R_body = Eigen::Quaterniond(
                 world_R_body[0], world_R_body[1], world_R_body[2], world_R_body[3]);
             input->sensor_input =
                 std::make_unique<PythonSensorInput>(timestamp_ns, depth, labels, rgb);
             return pipeline.spinOnce(*input);
           })
      .def("step",
           [](HydraPythonPipeline& pipeline,
              size_t timestamp_ns,
              const Eigen::Vector3d& world_t_body,
              const Eigen::Vector4d& world_R_body,
              const PythonSensorInput::PointVec& points,
              const PythonSensorInput::LabelVec& labels,
              const PythonSensorInput::ColorVec& colors) {
             auto input = std::make_shared<ReconstructionInput>();
             input->timestamp_ns = timestamp_ns;
             input->world_t_body = world_t_body;
             input->world_R_body = Eigen::Quaterniond(
                 world_R_body[0], world_R_body[1], world_R_body[2], world_R_body[3]);
             input->sensor_input = std::make_unique<PythonSensorInput>(
                 timestamp_ns, points, labels, colors);
             return pipeline.spinOnce(*input);
           })
      .def_property_readonly("graph", &HydraPythonPipeline::getSceneGraph);
}

}  // namespace hydra_python_pipeline

};  // namespace hydra::python
