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
#include "hydra_multi_ros/status_monitor.h"

#include <config_utilities/config.h>
#include <config_utilities/validation.h>

#include <chrono>
#include <nlohmann/json.hpp>

namespace hydra_multi {

StatusMonitor::StatusMonitor(const Config& config, ianvs::NodeHandle nh)
    : config(config::checkValid(config)),
      node_name(nh.node_name()),
      nh_(nh),
      pub_(nh.create_publisher<std_msgs::msg::String>("status", 1)) {}

void StatusMonitor::recordSpin(const MultiBackendModuleStatus& status) {
  // start critical section to update observations
  std::lock_guard<std::mutex> lock(mutex_);
  last_status_ = status;
  last_stamp_ = std::chrono::nanoseconds(nh_.now().nanoseconds());
  // end critical section to update observations
}

void StatusMonitor::start() {
  const auto period_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::duration<double>(config.report_period_s));
  timer_ = nh_.create_timer(period_ms, true, std::bind(&StatusMonitor::publish, this));
}

void StatusMonitor::publish() {
  bool have_spin = true;
  std::chrono::nanoseconds last_stamp;
  MultiBackendModuleStatus last_status;
  {  // get observations
    std::lock_guard<std::mutex> lock(mutex_);
    if (!last_status_ || !last_stamp_) {
      have_spin = false;
    }

    last_stamp = *last_stamp_;
    last_status = *last_status_;
  }  // end critical section

  std::chrono::nanoseconds curr_time_ns(nh_.now().nanoseconds());

  nlohmann::json record;
  record["nickname"] = config.nickname;
  record["node_name"] = node_name;

  if (have_spin) {
    const auto diff_s = std::chrono::duration_cast<std::chrono::duration<double>>(
        curr_time_ns - last_stamp);
    const auto valid = diff_s.count() < config.max_time_between_spins_s;
    record["status"] = valid ? "NOMINAL" : "ERROR";

    std::stringstream note;

    if (valid) {
      note << last_status.inlier_interrobot_loop_closures << " / "
           << last_status.total_interrobot_loop_closures
           << " valid interrobot loop closures";
    } else {
      note << "time since last spin: " << std::setprecision(3) << diff_s.count()
           << " [s]";
    }

    record["note"] = note.str();
  } else {
    record["status"] = "WARNING";
    record["note"] = "waiting for scene graphs";
  }

  std::stringstream ss;
  ss << record;

  auto msg = std::make_unique<std_msgs::msg::String>();
  msg->data = ss.str();
  pub_->publish(std::move(msg));
}

void declare_config(StatusMonitor::Config& config) {
  using namespace config;
  name("StatusMonitor::Config");
  field(config.nickname, "nickname");
  field(config.max_time_between_spins_s, "max_time_between_spins_s", "s");
  checkCondition(!config.nickname.empty(), "nickname");
  check(config.max_time_between_spins_s, GT, 0.0, "max_time_between_spins_s");
}

}  // namespace hydra_multi
