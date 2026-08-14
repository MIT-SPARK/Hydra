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
#include <hydra/input/input_module.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <optional>
#include <string>

#include <rclcpp/time.hpp>

#include <Eigen/Geometry>

namespace hydra {

PoseStatus lookupTransform(const tf2_ros::Buffer& buffer,
                           const std::optional<rclcpp::Time>& stamp,
                           const std::string& target,
                           const std::string& source,
                           size_t max_tries = 0,
                           double wait_duration_s = 0.1,
                           int verbosity = 10,
                           std::string* message = nullptr);

struct TFLookup {
  struct Config {
    //! Amount of time to wait between tf lookup attempts
    double wait_duration_s = 0.1;
    //! Buffer size in second for tf
    double buffer_size_s = 30.0;
    //! Number of lookup attempts before giving up (0 will try forever)
    size_t max_tries = 5;
    //! Logging verbosity of tf lookup process
    int verbosity = 3;
    //! Get the buffer size in nanoseconds
    std::chrono::nanoseconds buffer_size_ns() const;
  } const config;

  explicit TFLookup(const Config& config);
  PoseStatus getBodyPose(uint64_t timestamp_ns) const;

  tf2_ros::Buffer buffer;
  tf2_ros::TransformListener listener;
};

void declare_config(TFLookup::Config& config);

}  // namespace hydra
