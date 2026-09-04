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
#include "hydra_ros/input/image_receiver.h"

#include <config_utilities/config.h>
#include <config_utilities/types/path.h>
#include <config_utilities/validation.h>
#include <glog/logging.h>

namespace hydra {
namespace {

static const auto closed_registration =
    config::RegistrationWithConfig<DataReceiver,
                                   ClosedSetImageReceiver,
                                   ClosedSetImageReceiver::Config,
                                   std::string>("ClosedSetImageReceiver");

static const auto instance_registration =
    config::RegistrationWithConfig<DataReceiver,
                                   InstanceImageReceiver,
                                   InstanceImageReceiver::Config,
                                   std::string>("InstanceImageReceiver");

static const auto open_registration =
    config::RegistrationWithConfig<hydra::DataReceiver,
                                   OpenSetImageReceiver,
                                   OpenSetImageReceiver::Config,
                                   std::string>("OpenSetImageReceiver");

static const auto color_registration =
    config::RegistrationWithConfig<hydra::DataReceiver,
                                   ColormappedLabelImageReceiver,
                                   ColormappedLabelImageReceiver::Config,
                                   std::string>("ColormappedLabelImageReceiver");

}  // namespace

using semantic_inference_msgs::msg::FeatureImage;
using sensor_msgs::msg::Image;

ClosedSetImageReceiver::ClosedSetImageReceiver(const Config& config,
                                               const std::string& sensor_name)
    : ImageReceiverImpl<LabelSubscriber>(config, sensor_name) {}

void declare_config(ClosedSetImageReceiver::Config& config) {
  using namespace config;
  name("ClosedSetImageReceiver::Config");
  base<RosDataReceiver::Config>(config);
}

InstanceImageReceiver::InstanceImageReceiver(const Config& config,
                                             const std::string& sensor_name)
    : ImageReceiverImpl<InstanceSubscriber>(config, sensor_name) {}

void declare_config(InstanceImageReceiver::Config& config) {
  using namespace config;
  name("InstanceImageReceiver::Config");
  base<RosDataReceiver::Config>(config);
}

OpenSetImageReceiver::OpenSetImageReceiver(const Config& config,
                                           const std::string& sensor_name)
    : ImageReceiverImpl<FeatureSubscriber>(config, sensor_name) {}

void declare_config(OpenSetImageReceiver::Config& config) {
  using namespace config;
  name("OpenSetImageReceiver::Config");
  base<hydra::RosDataReceiver::Config>(config);
}

ColormappedLabelImageReceiver::ColormappedLabelImageReceiver(const Config& config,
                                                             const std::string& name)
    : ImageReceiverImpl<ColormappedLabelSubscriber>(config, name),
      config(config::checkValid(config)),
      colormap_(SemanticColorMap::fromCsv(config.colormap_path)) {
  CHECK(colormap_) << "Colormap required!";
}

bool ColormappedLabelImageReceiver::initImpl() {
  using Base = ImageReceiverImpl<ColormappedLabelSubscriber>;
  const auto ret = Base::initImpl();
  semantic_sub_.setColormap(colormap_.get(), config.default_label);
  return ret;
}

void declare_config(ColormappedLabelImageReceiver::Config& config) {
  using namespace config;
  name("ColormappedLabelImageReceiver::Config");
  base<hydra::RosDataReceiver::Config>(config);
  field<Path::Absolute>(config.colormap_path, "colormap_path");
  field(config.default_label, "default_label");
  check<Path::Exists>(config.colormap_path, "colormap_path");
}

}  // namespace hydra
