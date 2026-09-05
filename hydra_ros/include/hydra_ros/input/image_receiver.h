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
#include "hydra_ros/input/ros_data_receiver.h"
#include "hydra_ros/utils/qos_config.h"

namespace hydra {

class ImageReceiver : public RosDataReceiver {
 public:
  struct Config : RosDataReceiver::Config {
    //! Type of semantic message to subscribe to
    enum class SemanticsType {
      NONE,        // RGBD only
      CLOSED_SET,  // 2D closed-set semantic segmentation
      INSTANCE,    // 2D closed-set instance segmentation
      OPEN_SET,    // 2D open-set (and open-vocab) segmentation
    } semantics_type = SemanticsType::NONE;
    //! Whether or not to enable image-level features
    bool with_feature = false;
    //! Whether or not to use exact synchronization (versus approximate)
    bool use_exact = false;
    //! Queue depth for synchronization
    size_t queue_size = 10;
    //! QoS settings for subscribers
    QoSConfig qos = rclcpp::SensorDataQoS();
  } const config;

  ImageReceiver(const Config& config, const std::string& sensor_name);

  ~ImageReceiver();

 protected:
  bool initImpl() override;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

void declare_config(ImageReceiver::Config& config);

}  // namespace hydra
