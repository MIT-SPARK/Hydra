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
#include "hydra_ros/frontend/sub_keyframe_module.h"

#include <config_utilities/config.h>
#include <config_utilities/factory.h>
#include <config_utilities/printing.h>
#include <config_utilities/types/path.h>
#include <config_utilities/validation.h>
#include <glog/logging.h>
#include <hydra/common/pipeline_queues.h>
#include <hydra/input/camera.h>
#include <hydra/input/sensor_input_packet.h>
#include <hydra/utils/image_folder.h>

namespace hydra {
namespace {

static const auto registration =
    config::RegistrationWithConfig<SubKeyframeModule,
                                   SubKeyframeModule,
                                   SubKeyframeModule::Config>("SubKeyframeModule");

}  // namespace

void declare_config(SubKeyframeModule::Config& config) {
  using namespace config;
  name("SubKeyframeModule::Config");
  base<VerbosityConfig>(config);
  field<Path>(config.image_output_path, "image_output_path");
  field(config.sensor_name, "sensor_name");
  field(config.gate, "gate");
  field(config.tf_lookup, "tf_lookup");
  field(config.queue_max_size, "queue_max_size");
  check<Path::IsSet>(config.image_output_path, "image_output_path");
}

SubKeyframeModule::Config::Config()
    : VerbosityConfig(VerbosityConfig::default_verbosity("sub_keyframes")) {}

SubKeyframeModule::SubKeyframeModule(const Config& config)
    : config(config::checkValid(config)),
      gate_(config.gate),
      writer_(config.image_output_path, utils::kSubKeyframePrefix) {
  // images are accepted as soon as the module exists, i.e., before any input
  PipelineQueues::instance().enableSubKeyframes(config.queue_max_size,
                                                config.sensor_name);
}

SubKeyframeModule::~SubKeyframeModule() {
  stopImpl();
  PipelineQueues::instance().disableSubKeyframes();
}

void SubKeyframeModule::start() {
  lookup_ = std::make_unique<TFLookup>(config.tf_lookup);
  should_shutdown_ = false;
  spin_thread_ = std::make_unique<std::thread>(&SubKeyframeModule::spin, this);
}

void SubKeyframeModule::stop() { stopImpl(); }

void SubKeyframeModule::stopImpl() {
  should_shutdown_ = true;
  if (spin_thread_) {
    spin_thread_->join();
    spin_thread_.reset();
  }
}

std::string SubKeyframeModule::printInfo() const { return config::toString(config); }

bool SubKeyframeModule::processInput(const SubKeyframeInput& input,
                                     const Eigen::Isometry3d& world_T_body) {
  if (!input.parse ||
      !gate_.shouldTrigger(world_T_body.translation(),
                           Eigen::Quaterniond(world_T_body.rotation()))) {
    return false;
  }

  const auto packet = input.parse();
  if (!packet) {
    return false;
  }

  const auto camera = dynamic_cast<const Camera*>(input.sensor.get());
  if (camera) {
    writer_.writeCalib(CameraCalib::fromCamera(*camera));
  }

  const auto timestamp_ns = packet->timestamp_ns;
  const nlohmann::json meta{{"world_T_body", isometryToJson(world_T_body)}};
  if (!writer_.write(timestamp_ns,
                     colorToKeyframe(packet->color),
                     depthToKeyframe(packet->depth),
                     meta)) {
    return false;
  }

  // the frontend owns all modifications of the frontend graph
  PipelineQueues::instance().subkeyframe_node_queue.push(
      {timestamp_ns, world_T_body, writer_.imageFolder(timestamp_ns)}, false);
  return true;
}

void SubKeyframeModule::spin() {
  auto& queue = PipelineQueues::instance().subkeyframe_queue;
  while (!should_shutdown_) {
    if (!queue.poll()) {
      continue;
    }

    const auto input = queue.pop();
    // drop the images on failures instead of stopping capture
    try {
      const auto pose = lookup_->getBodyPose(input.timestamp_ns);
      if (!pose) {
        MLOG(2) << "No pose for images @ " << input.timestamp_ns << " [ns]";
        continue;
      }

      processInput(input, pose.target_T_source());
    } catch (const std::exception& e) {
      LOG_EVERY_N(WARNING, 100) << config.prefix << "Dropped images: " << e.what();
    }
  }
}

}  // namespace hydra
