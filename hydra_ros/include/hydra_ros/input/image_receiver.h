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
#include <hydra/common/semantic_color_map.h>
#include <ianvs/node_handle.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <rclcpp/time.hpp>
#include <semantic_inference_msgs/msg/feature_image.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "hydra_ros/input/ros_data_receiver.h"

namespace hydra {

template <typename MsgT>
struct FilterSub : public message_filters::SimpleFilter<MsgT> {
  FilterSub(ianvs::NodeHandle nh, const std::string& topic, uint32_t queue_size)
      : subscriber(nh.create_subscription<MsgT>(
            topic, queue_size, [this](const typename MsgT::ConstSharedPtr& msg) {
              this->signalMessage(msg);
            })) {}

  typename rclcpp::Subscription<MsgT>::SharedPtr subscriber;
};

struct ColorSubscriber {
 public:
  using MsgType = sensor_msgs::msg::Image;
  using Filter = message_filters::SimpleFilter<MsgType>;

  ColorSubscriber();
  explicit ColorSubscriber(ianvs::NodeHandle nh, uint32_t queue_size = 1);
  virtual ~ColorSubscriber();

  Filter& getFilter() const;
  void fillInput(const sensor_msgs::msg::Image& img, ImageInputPacket& packet) const;

 private:
  std::shared_ptr<FilterSub<sensor_msgs::msg::Image>> impl_;
};

struct DepthSubscriber {
 public:
  using MsgType = sensor_msgs::msg::Image;
  using Filter = message_filters::SimpleFilter<MsgType>;

  DepthSubscriber();
  explicit DepthSubscriber(ianvs::NodeHandle nh, uint32_t queue_size = 1);
  virtual ~DepthSubscriber();

  Filter& getFilter() const;
  void fillInput(const sensor_msgs::msg::Image& img, ImageInputPacket& packet) const;

 private:
  std::shared_ptr<FilterSub<sensor_msgs::msg::Image>> impl_;
};

struct LabelSubscriber {
 public:
  using MsgType = sensor_msgs::msg::Image;
  using Filter = message_filters::SimpleFilter<MsgType>;

  LabelSubscriber();
  explicit LabelSubscriber(ianvs::NodeHandle nh, uint32_t queue_size = 1);
  virtual ~LabelSubscriber();

  Filter& getFilter() const;
  void fillInput(const sensor_msgs::msg::Image& img, ImageInputPacket& packet) const;

 private:
  std::shared_ptr<FilterSub<sensor_msgs::msg::Image>> impl_;
};

struct InstanceSubscriber {
 public:
  using MsgType = sensor_msgs::msg::Image;
  using Filter = message_filters::SimpleFilter<MsgType>;

  InstanceSubscriber();
  explicit InstanceSubscriber(ianvs::NodeHandle nh, uint32_t queue_size = 1);
  virtual ~InstanceSubscriber();

  Filter& getFilter() const;
  void fillInput(const sensor_msgs::msg::Image& img, ImageInputPacket& packet) const;

 private:
  std::shared_ptr<FilterSub<sensor_msgs::msg::Image>> impl_;
};

struct ColormappedLabelSubscriber {
 public:
  using MsgType = sensor_msgs::msg::Image;
  using Filter = message_filters::SimpleFilter<MsgType>;

  ColormappedLabelSubscriber();
  explicit ColormappedLabelSubscriber(ianvs::NodeHandle nh, uint32_t queue_size = 1);
  virtual ~ColormappedLabelSubscriber();

  Filter& getFilter() const;
  void setColormap(const SemanticColorMap* colormap, int32_t default_label);
  void fillInput(const sensor_msgs::msg::Image& img, ImageInputPacket& packet) const;

 private:
  int32_t default_label_;
  const SemanticColorMap* colormap_;
  std::shared_ptr<FilterSub<sensor_msgs::msg::Image>> impl_;
};

struct FeatureSubscriber {
 public:
  using MsgType = semantic_inference_msgs::msg::FeatureImage;
  using Filter = message_filters::SimpleFilter<MsgType>;

  FeatureSubscriber();
  explicit FeatureSubscriber(ianvs::NodeHandle nh, uint32_t queue_size = 1);
  virtual ~FeatureSubscriber();

  Filter& getFilter() const;
  void fillInput(const MsgType& img, ImageInputPacket& packet) const;

 private:
  std::shared_ptr<FilterSub<semantic_inference_msgs::msg::FeatureImage>> impl_;
};

template <typename SemanticT>
class ImageReceiverImpl : public RosDataReceiver {
 public:
  using SemanticMsgPtr = typename SemanticT::MsgType::ConstPtr;
  using Policy =
      message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image,
                                                      sensor_msgs::msg::Image,
                                                      typename SemanticT::MsgType>;
  using Synchronizer = message_filters::Synchronizer<Policy>;

  ImageReceiverImpl(const RosDataReceiver::Config& config,
                    const std::string& sensor_name);
  virtual ~ImageReceiverImpl() = default;

 protected:
  bool initImpl() override;

  void callback(const sensor_msgs::msg::Image::ConstSharedPtr& color,
                const sensor_msgs::msg::Image::ConstSharedPtr& depth,
                const SemanticMsgPtr& labels);

  ColorSubscriber color_sub_;
  DepthSubscriber depth_sub_;
  SemanticT semantic_sub_;
  std::unique_ptr<Synchronizer> sync_;
};

template <typename SemanticT>
ImageReceiverImpl<SemanticT>::ImageReceiverImpl(const Config& config,
                                                const std::string& sensor_name)
    : RosDataReceiver(config, sensor_name) {}

template <typename SemanticT>
bool ImageReceiverImpl<SemanticT>::initImpl() {
  color_sub_ = ColorSubscriber(ianvs::NodeHandle::this_node(ns_));
  depth_sub_ = DepthSubscriber(ianvs::NodeHandle::this_node(ns_));
  semantic_sub_ = SemanticT(ianvs::NodeHandle::this_node(ns_));
  sync_.reset(new Synchronizer(Policy(config.queue_size),
                               color_sub_.getFilter(),
                               depth_sub_.getFilter(),
                               semantic_sub_.getFilter()));
  sync_->registerCallback(&ImageReceiverImpl<SemanticT>::callback, this);
  return true;
}

template <typename SemanticT>
void ImageReceiverImpl<SemanticT>::callback(
    const sensor_msgs::msg::Image::ConstSharedPtr& color,
    const sensor_msgs::msg::Image::ConstSharedPtr& depth,
    const SemanticMsgPtr& labels) {
  const auto timestamp_ns = rclcpp::Time(color->header.stamp).nanoseconds();
  if (!checkInputTimestamp(timestamp_ns)) {
    return;
  }

  auto packet = std::make_shared<ImageInputPacket>(timestamp_ns, sensor_name_);
  color_sub_.fillInput(*color, *packet);
  depth_sub_.fillInput(*depth, *packet);
  semantic_sub_.fillInput(*labels, *packet);
  queue.push(packet);
}

class RGBDImageReceiver : public RosDataReceiver {
 public:
  struct Config : RosDataReceiver::Config {};
  using Policy =
      message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image,
                                                      sensor_msgs::msg::Image>;
  using Synchronizer = message_filters::Synchronizer<Policy>;

  RGBDImageReceiver(const Config& config, const std::string& sensor_name);
  virtual ~RGBDImageReceiver() = default;

 protected:
  bool initImpl() override;

  void callback(const sensor_msgs::msg::Image::ConstSharedPtr& color,
                const sensor_msgs::msg::Image::ConstSharedPtr& depth);

  ColorSubscriber color_sub_;
  DepthSubscriber depth_sub_;
  std::unique_ptr<Synchronizer> sync_;
};

void declare_config(RGBDImageReceiver::Config& config);

class ClosedSetImageReceiver : public ImageReceiverImpl<LabelSubscriber> {
 public:
  struct Config : RosDataReceiver::Config {};
  ClosedSetImageReceiver(const Config& config, const std::string& sensor_name);
  virtual ~ClosedSetImageReceiver() = default;
};

void declare_config(ClosedSetImageReceiver::Config& config);

class InstanceImageReceiver : public ImageReceiverImpl<InstanceSubscriber> {
 public:
  struct Config : RosDataReceiver::Config {};
  InstanceImageReceiver(const Config& config, const std::string& sensor_name);
  virtual ~InstanceImageReceiver() = default;
};

void declare_config(InstanceImageReceiver::Config& config);

class OpenSetImageReceiver : public ImageReceiverImpl<FeatureSubscriber> {
 public:
  struct Config : RosDataReceiver::Config {};
  OpenSetImageReceiver(const Config& config, const std::string& sensor_name);
  virtual ~OpenSetImageReceiver() = default;
};

void declare_config(OpenSetImageReceiver::Config& config);

class ColormappedLabelImageReceiver
    : public ImageReceiverImpl<ColormappedLabelSubscriber> {
 public:
  struct Config : RosDataReceiver::Config {
    //! Path to colormap CSV to use to remap colors to labels
    std::filesystem::path colormap_path;
    //! Label value to use when value is unknown
    int32_t default_label = -1;
  } const config;

  ColormappedLabelImageReceiver(const Config& config, const std::string& sensor_name);
  virtual ~ColormappedLabelImageReceiver() = default;
  bool initImpl() override;

 private:
  std::unique_ptr<SemanticColorMap> colormap_;
};

void declare_config(ColormappedLabelImageReceiver::Config& config);

}  // namespace hydra
