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
#pragma once
#include <hydra/active_window/active_window_module.h>
#include <hydra/active_window/reconstruction_module.h>
#include <hydra/backend/backend_module.h>
#include <hydra/common/hydra_pipeline.h>
#include <hydra/frontend/graph_builder.h>

#include <memory>

#include "hydra_ros/input/feature_receiver.h"
#include "hydra_ros/input/ros_input_module.h"
#include "hydra_ros/utils/status_monitor.h"

namespace hydra {

class BowSubscriber;
class ExternalLoopClosureSubscriber;

class HydraRosPipeline : public HydraPipeline {
 public:
  struct Config {
    //! @brief Configuration for active window / metric-semantic reconstruction
    config::VirtualConfig<ActiveWindowModule> active_window{
        ReconstructionModule::Config()};
    //! @brief Configuration for frontend module
    config::VirtualConfig<GraphBuilder> frontend{GraphBuilder::Config()};
    //! @brief Configuration for backend module
    config::VirtualConfig<BackendModule> backend{BackendModule::Config()};
    //! @brief Publish frontend scene graph in addition to backend
    bool enable_frontend_output = true;
    //! @brief Turn on zmq-based publishing
    bool enable_zmq_interface = false;
    //! @brief Configuration for sensor inputs
    RosInputModule::Config input;
    //! @brief Receiver for language features
    config::VirtualConfig<FeatureReceiver> features;
    //! @brief Verbosity setting for main pipeline class
    int verbosity = 1;
    //! @brief Show the config passed to Hydra (before resolving sensor configurations)
    bool preprint_config = false;
    //! @brief Monitor to report whether or Hydra is running normally
    StatusMonitor::Config status_monitor;
  } const config;

  explicit HydraRosPipeline(int robot_id, int config_verbosity = 1);

  virtual ~HydraRosPipeline();

  void init() override;

  void start() override;

  void stop() override;

 protected:
  virtual void initLCD();

 protected:
  std::unique_ptr<StatusMonitor> status_monitor_;
  std::shared_ptr<ActiveWindowModule> active_window_;
  std::shared_ptr<GraphBuilder> frontend_;
  std::shared_ptr<BackendModule> backend_;

  std::unique_ptr<BowSubscriber> bow_sub_;
  std::unique_ptr<ExternalLoopClosureSubscriber> external_loop_closure_sub_;
};

void declare_config(HydraRosPipeline::Config& config);

}  // namespace hydra
