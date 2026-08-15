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

#include <cv_bridge/cv_bridge.hpp>

namespace hydra {

using semantic_inference_msgs::msg::FeatureImage;
using sensor_msgs::msg::Image;

ColorSubscriber::ColorSubscriber() = default;

ColorSubscriber::ColorSubscriber(ianvs::NodeHandle nh, uint32_t queue_size)
    : impl_(std::make_shared<FilterSub<Image>>(nh, "rgb/image_raw", queue_size)) {}

ColorSubscriber::~ColorSubscriber() = default;

ColorSubscriber::Filter& ColorSubscriber::getFilter() const {
  return *CHECK_NOTNULL(impl_);
}

void ColorSubscriber::fillInput(const Image& img, ImageInputPacket& packet) const {
  // Allow also mono images to be converted to grayscale.
  if (sensor_msgs::image_encodings::isColor(img.encoding) ||
      sensor_msgs::image_encodings::isBayer(img.encoding)) {
    try {
      packet.color =
          cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::RGB8)->image;
      return;
    } catch (const cv_bridge::Exception& e) {
      LOG(ERROR) << "Failed to convert color image: " << e.what();
      return;
    }
  } else if (sensor_msgs::image_encodings::isMono(img.encoding)) {
    try {
      cv::Mat mono =
          cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8)->image;
      cv::cvtColor(mono, packet.color, cv::COLOR_GRAY2RGB);
      return;
    } catch (const cv_bridge::Exception& e) {
      LOG(ERROR) << "Failed to convert mono image as color input: " << e.what();
      return;
    }
  }
  LOG(ERROR) << "Failed to convert color image: unsupported encoding: " << img.encoding;
}

DepthSubscriber::DepthSubscriber() = default;

DepthSubscriber::DepthSubscriber(ianvs::NodeHandle nh, uint32_t queue_size)
    : impl_(std::make_shared<FilterSub<Image>>(
          nh, "depth_registered/image_rect", queue_size)) {}

DepthSubscriber::~DepthSubscriber() = default;

DepthSubscriber::Filter& DepthSubscriber::getFilter() const {
  return *CHECK_NOTNULL(impl_);
}

void DepthSubscriber::fillInput(const Image& img, ImageInputPacket& packet) const {
  try {
    packet.depth = cv_bridge::toCvCopy(img)->image;
  } catch (const cv_bridge::Exception& e) {
    LOG(ERROR) << "Failed to convert depth image: " << e.what();
  }
}

LabelSubscriber::LabelSubscriber() = default;

LabelSubscriber::LabelSubscriber(ianvs::NodeHandle nh, uint32_t queue_size)
    : impl_(std::make_shared<FilterSub<Image>>(nh, "semantic/image_raw", queue_size)) {}

LabelSubscriber::~LabelSubscriber() = default;

LabelSubscriber::Filter& LabelSubscriber::getFilter() const {
  return *CHECK_NOTNULL(impl_);
}

void LabelSubscriber::fillInput(const Image& img, ImageInputPacket& packet) const {
  try {
    packet.labels = cv_bridge::toCvCopy(img)->image;
  } catch (const cv_bridge::Exception& e) {
    LOG(ERROR) << "Failed to convert label image: " << e.what();
  }
}

InstanceSubscriber::InstanceSubscriber() = default;

InstanceSubscriber::InstanceSubscriber(ianvs::NodeHandle nh, uint32_t queue_size)
    : impl_(std::make_shared<FilterSub<Image>>(nh, "semantic/image_raw", queue_size)) {}

InstanceSubscriber::~InstanceSubscriber() = default;

InstanceSubscriber::Filter& InstanceSubscriber::getFilter() const {
  return *CHECK_NOTNULL(impl_);
}

void InstanceSubscriber::fillInput(const Image& img, ImageInputPacket& packet) const {
  cv::Mat mat;
  try {
    mat = cv_bridge::toCvCopy(img)->image;
  } catch (const cv_bridge::Exception& e) {
    LOG(ERROR) << "Failed to convert label image: " << e.what();
  }

  if (mat.type() != CV_32SC1) {
    LOG(ERROR) << "Invalid encoding for instance+label image";
    return;
  }

  packet.labels = cv::Mat(mat.size(), CV_32SC1);
  packet.instances = cv::Mat(mat.size(), CV_16SC1);
  for (int r = 0; r < mat.rows; ++r) {
    for (int c = 0; c < mat.cols; ++c) {
      const auto original = mat.at<int32_t>(r, c);
      packet.labels.at<int32_t>(r, c) = original & 0xFFFF;
      packet.instances.at<int16_t>(r, c) = original >> 16;
    }
  }
}

ColormappedLabelSubscriber::ColormappedLabelSubscriber()
    : default_label_(-1), colormap_(nullptr) {}

ColormappedLabelSubscriber::ColormappedLabelSubscriber(ianvs::NodeHandle nh,
                                                       uint32_t queue_size)
    : default_label_(-1),
      colormap_(nullptr),
      impl_(std::make_shared<FilterSub<Image>>(nh, "semantic/image_raw", queue_size)) {}

ColormappedLabelSubscriber::~ColormappedLabelSubscriber() = default;

ColormappedLabelSubscriber::Filter& ColormappedLabelSubscriber::getFilter() const {
  return *CHECK_NOTNULL(impl_);
}

void ColormappedLabelSubscriber::setColormap(const SemanticColorMap* colormap,
                                             int32_t default_label) {
  colormap_ = colormap;
  default_label_ = default_label;
}

void ColormappedLabelSubscriber::fillInput(const Image& img,
                                           ImageInputPacket& packet) const {
  if (!colormap_) {
    LOG(ERROR) << "Colormap not set for subscriber!";
    return;
  }

  cv::Mat colors;
  try {
    colors = cv_bridge::toCvCopy(img)->image;
  } catch (const cv_bridge::Exception& e) {
    LOG(ERROR) << "Failed to convert label image: " << e.what();
    return;
  }

  if (colors.empty() || colors.channels() != 3) {
    LOG(ERROR) << "Failed to decode color image to semantics!";
    return;
  }

  packet.labels = cv::Mat(colors.size(), CV_32SC1);
  for (int r = 0; r < colors.rows; ++r) {
    for (int c = 0; c < colors.cols; ++c) {
      const auto& pixel = colors.at<cv::Vec3b>(r, c);
      const spark_dsg::Color color(pixel[0], pixel[1], pixel[2]);
      packet.labels.at<int32_t>(r, c) =
          colormap_->getLabelFromColor(color).value_or(default_label_);
    }
  }
}

FeatureSubscriber::FeatureSubscriber() = default;

FeatureSubscriber::FeatureSubscriber(ianvs::NodeHandle nh, uint32_t queue_size)
    : impl_(std::make_shared<FilterSub<FeatureImage>>(
          nh, "semantic/image_raw", queue_size)) {}

FeatureSubscriber::~FeatureSubscriber() = default;

FeatureSubscriber::Filter& FeatureSubscriber::getFilter() const {
  return *CHECK_NOTNULL(impl_);
}

void FeatureSubscriber::fillInput(const MsgType& msg, ImageInputPacket& packet) const {
  try {
    packet.instances = cv_bridge::toCvCopy(msg.image)->image;
  } catch (const cv_bridge::Exception& e) {
    LOG(ERROR) << "Failed to convert depth image: " << e.what();
  }

  CHECK_EQ(msg.mask_ids.size(), msg.features.size());
  for (size_t i = 0; i < msg.mask_ids.size(); ++i) {
    const auto& vec = msg.features[i].data;
    packet.label_features.emplace(
        msg.mask_ids[i],
        Eigen::Map<const hydra::FeatureVector>(vec.data(), vec.size()));
  }
}

RGBDImageReceiver::RGBDImageReceiver(const Config& config,
                                     const std::string& sensor_name)
    : RosDataReceiver(config, sensor_name) {}

bool RGBDImageReceiver::initImpl() {
  color_sub_ = ColorSubscriber(ianvs::NodeHandle::this_node(ns_));
  depth_sub_ = DepthSubscriber(ianvs::NodeHandle::this_node(ns_));
  sync_.reset(new Synchronizer(
      Policy(config.queue_size), color_sub_.getFilter(), depth_sub_.getFilter()));
  sync_->registerCallback(&RGBDImageReceiver::callback, this);
  return true;
}

void RGBDImageReceiver::callback(const sensor_msgs::msg::Image::ConstSharedPtr& color,
                                 const sensor_msgs::msg::Image::ConstSharedPtr& depth) {
  const auto timestamp_ns = rclcpp::Time(color->header.stamp).nanoseconds();
  if (!checkInputTimestamp(timestamp_ns)) {
    return;
  }

  auto packet = std::make_shared<ImageInputPacket>(timestamp_ns, sensor_name_);
  color_sub_.fillInput(*color, *packet);
  depth_sub_.fillInput(*depth, *packet);
  queue.push(packet);
}

void declare_config(RGBDImageReceiver::Config& config) {
  using namespace config;
  name("RGBDImageReceiver::Config");
  base<RosDataReceiver::Config>(config);
}

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

namespace {

static const auto no_semantic_registration =
    config::RegistrationWithConfig<DataReceiver,
                                   RGBDImageReceiver,
                                   RGBDImageReceiver::Config,
                                   std::string>("RGBDImageReceiver");

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
}  // namespace hydra
