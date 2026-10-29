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
#include <hydra/common/hydra_pipeline.h>

#include <memory>

#include "hydra_ros/frontend/sub_keyframe_module.h"
#include "hydra_ros/input/ros_input_module.h"
#include "hydra_ros/utils/status_monitor.h"

namespace hydra {

class ExternalLoopClosureSubscriber;

class HydraRosPipeline : public HydraPipeline {
 public:
  struct Config : HydraPipeline::Config {
    //! Show the config passed to Hydra (before resolving sensor configurations)
    bool preprint_config = false;
    //! Configuration for sensor inputs
    RosInputModule::Config input;
    //! Monitor to report whether or Hydra is running normally
    StatusMonitor::Config status_monitor;
    //! @brief Optional config for full-rate sub-keyframe capture
    config::VirtualConfig<SubKeyframeModule> sub_keyframe;
  } const config;

  explicit HydraRosPipeline(const Config& config);

  virtual ~HydraRosPipeline();

  void init() override;

  void start() override;

  void stop() override;

 protected:
  std::unique_ptr<RosInputModule> input_;
  std::unique_ptr<StatusMonitor> monitor_;
  std::unique_ptr<ExternalLoopClosureSubscriber> external_loop_closure_sub_;
  std::unique_ptr<SubKeyframeModule> sub_keyframe_;
};

void declare_config(HydraRosPipeline::Config& config);

}  // namespace hydra
