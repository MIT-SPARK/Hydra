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
#include "hydra_ros/utils/tf_lookup.h"

#include <config_utilities/config.h>
#include <config_utilities/printing.h>
#include <config_utilities/validation.h>
#include <glog/logging.h>
#include <hydra/common/global_info.h>
#include <ianvs/node_handle.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

namespace hydra {

std::chrono::nanoseconds TFLookup::Config::buffer_size_ns() const {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(buffer_size_s));
}

PoseStatus lookupTransform(const tf2_ros::Buffer& buffer,
                           const std::optional<rclcpp::Time>& stamp,
                           const std::string& target,
                           const std::string& source,
                           size_t max_tries,
                           double wait_duration_s,
                           int verbosity,
                           std::string* message) {
  using namespace std::chrono_literals;
  rclcpp::WallRate tf_wait_rate(1.0 / wait_duration_s);
  std::string stamp_suffix;
  if (stamp) {
    std::stringstream ss;
    ss << " @ " << stamp.value().nanoseconds() << " [ns]";
    stamp_suffix = ss.str();
  }

  bool have_transform = false;
  std::string err_str;
  VLOG(verbosity) << "Looking up transform " << target << "_T_" << source
                  << stamp_suffix;

  const auto lookup_time = stamp.value_or(rclcpp::Time());
  size_t attempt_number = 0;
  while (rclcpp::ok()) {
    VLOG(verbosity) << "Attempting to lookup tf @ " << lookup_time.nanoseconds()
                    << " [ns]: " << attempt_number << " / "
                    << (max_tries ? std::to_string(max_tries) : "n/a");
    if (max_tries && attempt_number >= max_tries) {
      break;
    }

    if (buffer.canTransform(
            target, source, lookup_time, rclcpp::Duration(0ns), &err_str)) {
      have_transform = true;
      break;
    }

    ++attempt_number;
    tf_wait_rate.sleep();
    // TODO(nathan) spin if needed
  }

  if (!have_transform) {
    std::stringstream ss;
    ss << "'" << target << "_T_" << source << stamp_suffix;
    if (!err_str.empty()) {
      ss << "' (last error: '" << err_str << "')";
    }

    if (message) {
      *message = ss.str();
    } else {
      LOG(ERROR) << "Failed to find " << ss.str();
    }

    return {false, {}, {}};
  }

  geometry_msgs::msg::TransformStamped transform;
  try {
    transform = buffer.lookupTransform(target, source, lookup_time);
  } catch (const tf2::TransformException& ex) {
    LOG(ERROR) << "Failed to look up available transform " << target << "_T_" << source
               << stamp_suffix;
    return {false, {}, {}};
  }

  PoseStatus to_return;
  to_return.is_valid = true;
  tf2::fromMsg(transform.transform.translation, to_return.target_p_source);
  tf2::fromMsg(transform.transform.rotation, to_return.target_R_source);
  to_return.target_R_source.normalize();
  return to_return;
}

void declare_config(TFLookup::Config& config) {
  using namespace config;
  name("TFLookup::Config");
  field(config.wait_duration_s, "wait_duration_s");
  field(config.buffer_size_s, "buffer_size_s");
  field(config.max_tries, "max_tries");
  field(config.verbosity, "verbosity");
}

TFLookup::TFLookup(const Config& config)
    : config(config),
      buffer(ianvs::NodeHandle::this_node().clock(), config.buffer_size_ns()),
      listener(buffer) {}

PoseStatus TFLookup::getBodyPose(uint64_t timestamp_ns) const {
  rclcpp::Time curr_ros_time(timestamp_ns);
  return lookupTransform(buffer,
                         curr_ros_time,
                         GlobalInfo::instance().getFrames().odom,
                         GlobalInfo::instance().getFrames().robot,
                         config.max_tries,
                         config.wait_duration_s,
                         config.verbosity);
}

}  // namespace hydra
