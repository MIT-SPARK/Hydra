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
#include <config_utilities/types/enum.h>
#include <config_utilities/validation.h>
#include <glog/logging.h>
#include <ianvs/node_handle.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>

#include <cv_bridge/cv_bridge.hpp>
#include <rclcpp/time.hpp>
#include <semantic_inference_msgs/msg/feature_image.hpp>
#include <semantic_inference_msgs/msg/feature_vector_stamped.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>

using sensor_msgs::msg::Image;
using FeatureImage = semantic_inference_msgs::msg::FeatureImage;

namespace hydra {
namespace {

static const auto registration =
    config::RegistrationWithConfig<DataReceiver,
                                   ImageReceiver,
                                   ImageReceiver::Config,
                                   std::string>("ImageReceiver");

cv::Mat parseColor(const Image& msg) {
  using namespace sensor_msgs::image_encodings;
  if (isColor(msg.encoding) || isBayer(msg.encoding)) {
    try {
      return cv_bridge::toCvCopy(msg, RGB8)->image;
    } catch (const cv_bridge::Exception& e) {
      LOG(ERROR) << "Failed to convert color image: " << e.what();
      return cv::Mat();
    }
  }

  if (isMono(msg.encoding)) {
    try {
      cv::Mat color;
      auto mono = cv_bridge::toCvCopy(msg, MONO8);
      cv::cvtColor(mono->image, color, cv::COLOR_GRAY2RGB);
      return color;
    } catch (const cv_bridge::Exception& e) {
      LOG(ERROR) << "Failed to convert mono image as color input: " << e.what();
      return cv::Mat();
    }
  }

  LOG(ERROR) << "Failed to convert color image: unsupported encoding: " << msg.encoding;
  return cv::Mat();
}

cv::Mat parseDepth(const Image& img) {
  try {
    return cv_bridge::toCvCopy(img)->image;
  } catch (const cv_bridge::Exception& e) {
    LOG(ERROR) << "Failed to convert depth image: " << e.what();
    return cv::Mat();
  }
}

}  // namespace

template <typename MsgT, typename Derived>
struct SemanticAdapter {
  using MsgType = MsgT;
  using Callback = const std::function<void(const typename MsgType::ConstPtr&)>;

  void fill(const MsgT& msg, ImageInputPacket& packet) {
    static_cast<Derived>(this)->fill(msg, packet);
  }

  rclcpp::Subscription<MsgType>::SharedPtr sub;

 protected:
  SemanticAdapter(ianvs::NodeHandle nh,
                  const std::string& topic,
                  const rclcpp::QoS& qos,
                  const Callback& callback)
      : sub(nh.create_subscription<MsgType>(topic, qos, callback)) {}
};

struct ClosedSetAdapter : public SemanticAdapter<Image, ClosedSetAdapter> {
  using Base = SemanticAdapter<Image, ClosedSetAdapter>;
  using MsgType = Base::MsgType;

  ClosedSetAdapter(ianvs::NodeHandle nh,
                   const std::string& topic,
                   const rclcpp::QoS& qos,
                   const Base::Callback& callback)
      : Base(nh, topic, qos, callback) {}

  void fill(const Image& msg, ImageInputPacket& packet) const;
};

struct InstanceAdapter : public SemanticAdapter<Image, InstanceAdapter> {
  using Base = SemanticAdapter<Image, InstanceAdapter>;
  using MsgType = Base::MsgType;

  InstanceAdapter(ianvs::NodeHandle nh,
                  const std::string& topic,
                  const rclcpp::QoS& qos,
                  const Base::Callback& callback)
      : Base(nh, topic, qos, callback) {}

  void fill(const Image& img, ImageInputPacket& packet) const;
};

struct OpenSetAdapter : public SemanticAdapter<FeatureImage, OpenSetAdapter> {
  using Base = SemanticAdapter<FeatureImage, OpenSetAdapter>;
  using MsgType = Base::MsgType;

  OpenSetAdapter(ianvs::NodeHandle nh,
                 const std::string& topic,
                 const rclcpp::QoS& qos,
                 const Base::Callback& callback)
      : Base(nh, topic, qos, callback) {}

  void fill(const FeatureImage& img, ImageInputPacket& packet) const;
};

void ClosedSetAdapter::fill(const Image& msg, ImageInputPacket& packet) const {
  try {
    packet.labels = cv_bridge::toCvCopy(msg)->image;
  } catch (const cv_bridge::Exception& e) {
    LOG(ERROR) << "Failed to convert label image: " << e.what();
  }
}

void InstanceAdapter::fill(const Image& img, ImageInputPacket& packet) const {
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

void OpenSetAdapter::fill(const FeatureImage& msg, ImageInputPacket& packet) const {
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

using message_filters::Synchronizer;
using message_filters::sync_policies::ApproximateTime;
using message_filters::sync_policies::ExactTime;

template <typename MsgT, bool exact>
struct policy_type;

template <typename MsgT>
struct policy_type<MsgT, true> {
  using value = ExactTime<Image, Image, MsgT>;
};

template <typename MsgT>
struct policy_type<MsgT, false> {
  using value = ApproximateTime<Image, Image, MsgT>;
};

struct ImageReceiverBase {
  virtual ~ImageReceiverBase() = default;
};

template <typename AdapterT, bool exact>
struct ImageReceiverImpl : public ImageReceiverBase {
  using ImgPtr = Image::ConstSharedPtr;
  using MsgT = typename AdapterT::MsgType;
  using MsgPtr = typename AdapterT::MsgType::ConstSharedPtr;

  ImageReceiverImpl(ianvs::NodeHandle nh,
                    const std::string& name,
                    const ImageReceiver::Config& config);

  void callback(const Image::ConstSharedPtr& color,
                const Image::ConstSharedPtr& depth,
                const MsgPtr& labels);

  const std::string name;
  const bool use_exact;
  MessageQueue<SensorInputPacket::Ptr> queue;
  Synchronizer<typename policy_type<MsgT, exact>::value> sync;

  rclcpp::Subscription<Image>::SharedPtr color;
  rclcpp::Subscription<Image>::SharedPtr depth;
  AdapterT semantics;
  rclcpp::Subscription<Image>::SharedPtr feature;
};

template <typename AdapterT, bool exact>
ImageReceiverImpl<AdapterT, exact>::ImageReceiverImpl(
    ianvs::NodeHandle nh, const std::string& name, const ImageReceiver::Config& config)
    : name(name),
      use_exact(config.use_exact),
      sync(config.queue_size),
      color(nh.create_subscription<Image>(
          "rgb/image_raw",
          config.qos,
          [this](const ImgPtr& msg) { sync.template add<0>(msg); })),
      depth(nh.create_subscription<Image>(
          "depth_registered/image_rect",
          config.qos,
          [this](const ImgPtr& msg) { sync.template add<1>(msg); })),
      semantics(nh, "semantic/image_raw", config.qos, [this](const MsgPtr& msg) {
        sync.template add<2>(msg);
      }) {
  sync.registerCallback(&ImageReceiverImpl<AdapterT, exact>::callback, this);
}

template <typename AdapterT, bool exact>
void ImageReceiverImpl<AdapterT, exact>::callback(
    const Image::ConstSharedPtr& _color,
    const Image::ConstSharedPtr& _depth,
    const typename MsgT::ConstSharedPtr& _semantics) {
  const auto timestamp_ns = rclcpp::Time(_color->header.stamp).nanoseconds();

  auto packet = std::make_shared<ImageInputPacket>(timestamp_ns, name);
  packet->color = parseColor(*_color);
  packet->depth = parseDepth(*_depth);
  semantics.fill(*_semantics, *packet);
  queue.push(packet);
}

template <typename AdapterT>
std::unique_ptr<ImageReceiverBase> makeReceiver(const ImageReceiver::Config& config,
                                                ianvs::NodeHandle nh,
                                                const std::string& name) {
  if (config.use_exact) {
    return std::make_unique<ImageReceiverImpl<AdapterT, true>>(nh, name, config);
  } else {
    return std::make_unique<ImageReceiverImpl<AdapterT, false>>(nh, name, config);
  }
}

struct ImageReceiver::Impl {
  explicit Impl(const ImageReceiver::Config& config,
                ianvs::NodeHandle nh,
                const std::string& name) {
    switch (config.semantics_type) {
      case ImageReceiver::Config::SemanticsType::NONE:
        recv = makeReceiver<ClosedSetAdapter>(config, nh, name);
        break;
      case ImageReceiver::Config::SemanticsType::CLOSED_SET:
        recv = makeReceiver<ClosedSetAdapter>(config, nh, name);
        break;
      case ImageReceiver::Config::SemanticsType::INSTANCE:
        recv = makeReceiver<InstanceAdapter>(config, nh, name);
        break;
      case ImageReceiver::Config::SemanticsType::OPEN_SET:
        recv = makeReceiver<OpenSetAdapter>(config, nh, name);
        break;
    }
  }

  std::unique_ptr<ImageReceiverBase> recv;
};

void declare_config(ImageReceiver::Config& config) {
  using namespace config;
  name("ImageReceiver::Config");
  enum_field(config.semantics_type,
             "semantics_type",
             {{ImageReceiver::Config::SemanticsType::NONE, "none"},
              {ImageReceiver::Config::SemanticsType::CLOSED_SET, "closed_set"},
              {ImageReceiver::Config::SemanticsType::INSTANCE, "instance"},
              {ImageReceiver::Config::SemanticsType::OPEN_SET, "open_set"}});
  field(config.use_exact, "use_exact");
  field(config.queue_size, "queue_size");
  field(config.qos, "qos");
}

ImageReceiver::ImageReceiver(const Config& config, const std::string& sensor_name)
    : RosDataReceiver(config, sensor_name), config(config) {
  if (config.queue_size <= 2 && !config.use_exact) {
    LOG(WARNING) << "ApproximateTime policy requires queue sizes larger than 2";
  }
}

ImageReceiver::~ImageReceiver() = default;

bool ImageReceiver::initImpl() {
  auto nh = ianvs::NodeHandle::this_node(ns_);
  impl_.reset(new Impl(config, nh, sensor_name));
  return true;
}

// class ColormappedLabelImageReceiver
//     : public ImageReceiverImpl<ColormappedLabelSubscriber> {
//  public:
//   struct Config : RosDataReceiver::Config {
//     //! Path to colormap CSV to use to remap colors to labels
//     std::filesystem::path colormap_path;
//     //! Label value to use when value is unknown
//     int32_t default_label = -1;
//   } const config;
//
//   ColormappedLabelImageReceiver(const Config& config, const std::string&
//   sensor_name); virtual ~ColormappedLabelImageReceiver() = default; bool
//   initImpl() override;
//
//  private:
//   std::unique_ptr<SemanticColorMap> colormap_;
// };
// ColormappedLabelImageReceiver::ColormappedLabelImageReceiver(const Config&
// config,
//                                                              const std::string&
//                                                              name)
//     : ImageReceiverImpl<ColormappedLabelSubscriber>(config, name),
//       config(config::checkValid(config)),
//       colormap_(SemanticColorMap::fromCsv(config.colormap_path)) {
//   CHECK(colormap_) << "Colormap required!";
// }
//
// bool ColormappedLabelImageReceiver::initImpl() {
//   using Base = ImageReceiverImpl<ColormappedLabelSubscriber>;
//   const auto ret = Base::initImpl();
//   semantic_sub_.setColormap(colormap_.get(), config.default_label);
//   return ret;
// }
//
// void declare_config(ColormappedLabelImageReceiver::Config& config) {
//   using namespace config;
//   name("ColormappedLabelImageReceiver::Config");
//   base<hydra::RosDataReceiver::Config>(config);
//   field<Path::Absolute>(config.colormap_path, "colormap_path");
//   field(config.default_label, "default_label");
//   check<Path::Exists>(config.colormap_path, "colormap_path");
// }
//
// struct ColormappedLabelSubscriber {
//  public:
//   using MsgType = sensor_msgs::msg::Image;
//
//   explicit ColormappedLabelSubscriber(ianvs::NodeHandle nh, uint32_t queue_size =
//   1);
//
//   void setColormap(const SemanticColorMap* colormap, int32_t default_label);
//   void fillInput(const sensor_msgs::msg::Image& img, ImageInputPacket& packet)
//   const;
//
//  private:
//   int32_t default_label_;
//   const SemanticColorMap* colormap_;
//   std::shared_ptr<FilterSub<sensor_msgs::msg::Image>> impl_;
// };
//
// ColormappedLabelSubscriber::ColormappedLabelSubscriber()
//     : default_label_(-1), colormap_(nullptr) {}
//
// ColormappedLabelSubscriber::ColormappedLabelSubscriber(ianvs::NodeHandle nh,
//                                                        uint32_t queue_size)
//     : default_label_(-1), colormap_(nullptr) {}
//
// ColormappedLabelSubscriber::~ColormappedLabelSubscriber() = default;
//
// void ColormappedLabelSubscriber::setColormap(const SemanticColorMap* colormap,
//                                              int32_t default_label) {
//   colormap_ = colormap;
//   default_label_ = default_label;
// }
//
// void ColormappedLabelSubscriber::fillInput(const Image& img,
//                                            ImageInputPacket& packet) const {
//   if (!colormap_) {
//     LOG(ERROR) << "Colormap not set for subscriber!";
//     return;
//   }
//
//   cv::Mat colors;
//   try {
//     colors = cv_bridge::toCvCopy(img)->image;
//   } catch (const cv_bridge::Exception& e) {
//     LOG(ERROR) << "Failed to convert label image: " << e.what();
//     return;
//   }
//
//   if (colors.empty() || colors.channels() != 3) {
//     LOG(ERROR) << "Failed to decode color image to semantics!";
//     return;
//   }
//
//   packet.labels = cv::Mat(colors.size(), CV_32SC1);
//   for (int r = 0; r < colors.rows; ++r) {
//     for (int c = 0; c < colors.cols; ++c) {
//       const auto& pixel = colors.at<cv::Vec3b>(r, c);
//       const spark_dsg::Color color(pixel[0], pixel[1], pixel[2]);
//       packet.labels.at<int32_t>(r, c) =
//           colormap_->getLabelFromColor(color).value_or(default_label_);
//     }
//   }
// }
//

}  // namespace hydra
