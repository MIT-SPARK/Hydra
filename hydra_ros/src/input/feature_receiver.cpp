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
#include "hydra_ros/input/feature_receiver.h"

#include <config_utilities/config.h>
#include <config_utilities/printing.h>
#include <config_utilities/validation.h>
#include <glog/logging.h>
#include <hydra/common/global_info.h>
#include <hydra/common/pipeline_queues.h>
#include <hydra/frontend/view_selector.h>
#include <ianvs/node_handle.h>

#include <semantic_inference_msgs/msg/feature_vector_stamped.hpp>

namespace hydra {

void declare_config(FeatureReceiver::Config& config) {
  using namespace config;
  name("FeatureReceiver::Config");
  field(config.ns, "ns");
  field(config.queue_size, "queue_size");
  field(config.tf_lookup, "tf_lookup");
  field(config.sensors_to_exclude, "sensors_to_exclude");
  field(config.verbosity, "verbosity");
}

using semantic_inference_msgs::msg::FeatureVectorStamped;

struct FeatureSubscriber {
  using Ptr = std::unique_ptr<FeatureSubscriber>;
  using Callback = std::function<PoseStatus(uint64_t)>;

  FeatureSubscriber(ianvs::NodeHandle nh,
                    const std::string& sensor_name,
                    const Callback& pose_callback,
                    size_t queue_size = 10,
                    size_t verbosity = 0);

  void callback(const FeatureVectorStamped& msg);

  rclcpp::Subscription<FeatureVectorStamped>::SharedPtr sub;
  const std::string sensor_name;
  const Callback pose_callback;
  const size_t verbosity;
};

FeatureSubscriber::FeatureSubscriber(ianvs::NodeHandle nh,
                                     const std::string& sensor_name,
                                     const Callback& pose_callback,
                                     size_t queue_size,
                                     size_t verbosity)
    : sensor_name(sensor_name), pose_callback(pose_callback), verbosity(verbosity) {
  const std::string topic = sensor_name + "/feature";
  sub = nh.create_subscription<FeatureVectorStamped>(
      topic, queue_size, &FeatureSubscriber::callback, this);
}

void FeatureSubscriber::callback(const FeatureVectorStamped& msg) {
  const auto timestamp_ns = rclcpp::Time(msg.header.stamp).nanoseconds();
  const auto& vec = msg.feature.data;

  const auto sensor = GlobalInfo::instance().getSensor(sensor_name);
  const auto pose_status = pose_callback(timestamp_ns);
  if (!pose_status) {
    LOG(WARNING) << "Dropping feature @ " << timestamp_ns << "[ns] for sensor "
                 << sensor_name;
    return;
  }

  const Eigen::Isometry3d world_T_sensor =
      pose_status.target_T_source() * sensor->body_T_sensor();
  auto packet = std::make_unique<FeatureView>(
      timestamp_ns,
      world_T_sensor.inverse(),
      Eigen::Map<const Eigen::VectorXf>(vec.data(), vec.size()),
      GlobalInfo::instance().getSensor(sensor_name).get());

  LOG_IF(INFO, verbosity >= 2) << "Pushing new feature to input queue @ "
                               << timestamp_ns << "[ns] for '" << sensor_name << "'";
  PipelineQueues::instance().input_features_queue.push(std::move(packet));
}

void FeatureReceiver::start() {
  std::set<std::string> to_exclude(config.sensors_to_exclude.begin(),
                                   config.sensors_to_exclude.end());
  const auto sensor_names = GlobalInfo::instance().getAvailableSensors();
  for (const auto& name : sensor_names) {
    if (to_exclude.count(name)) {
      LOG_IF(INFO, config.verbosity >= 1)
          << "Excluding '" << name << "' from feature receivers!";
      continue;
    }

    auto nh = ianvs::NodeHandle::this_node(config.ns);
    LOG_IF(INFO, config.verbosity >= 1)
        << "Making feature receiver for '" << name << "'";
    subs_.push_back(std::make_unique<FeatureSubscriber>(
        nh,
        name,
        [this](uint64_t timestamp_ns) { return lookup_.getBodyPose(timestamp_ns); },
        config.queue_size,
        config.verbosity));
  }
}

FeatureReceiver::FeatureReceiver(const Config& config)
    : config(config::checkValid(config)), lookup_(config.tf_lookup) {}

FeatureReceiver::~FeatureReceiver() {}

void FeatureReceiver::stop() {}

std::string FeatureReceiver::printInfo() const { return config::toString(config); }

}  // namespace hydra
