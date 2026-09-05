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

using semantic_inference_msgs::msg::FeatureImage;
using semantic_inference_msgs::msg::FeatureVectorStamped;
using sensor_msgs::msg::Image;

using message_filters::Synchronizer;
using message_filters::sync_policies::ApproximateTime;
using message_filters::sync_policies::ExactTime;

namespace hydra {
namespace {

#define MAKE_VARIADIC(Policy, Underlying)                                           \
  template <typename... MsgT>                                                       \
  struct Policy;                                                                    \
                                                                                    \
  template <typename A, typename B>                                                 \
  struct Policy<A, B> {                                                             \
    using value = Underlying<A, B>;                                                 \
  };                                                                                \
                                                                                    \
  template <typename A, typename B, typename C>                                     \
  struct Policy<A, B, C> {                                                          \
    using value = Underlying<A, B, C>;                                              \
  };                                                                                \
                                                                                    \
  template <typename A, typename B, typename C, typename D>                         \
  struct Policy<A, B, C, D> {                                                       \
    using value = Underlying<A, B, C, D>;                                           \
  };                                                                                \
                                                                                    \
  template <typename A, typename B, typename C, typename D, typename E>             \
  struct Policy<A, B, C, D, E> {                                                    \
    using value = Underlying<A, B, C, D, E>;                                        \
  };                                                                                \
                                                                                    \
  template <typename A, typename B, typename C, typename D, typename E, typename F> \
  struct Policy<A, B, C, D, E, F> {                                                 \
    using value = Underlying<A, B, C, D, E, F>;                                     \
  };                                                                                \
                                                                                    \
  template <typename A,                                                             \
            typename B,                                                             \
            typename C,                                                             \
            typename D,                                                             \
            typename E,                                                             \
            typename F,                                                             \
            typename G>                                                             \
  struct Policy<A, B, C, D, E, F, G> {                                              \
    using value = Underlying<A, B, C, D, E, F, G>;                                  \
  };                                                                                \
                                                                                    \
  template <typename A,                                                             \
            typename B,                                                             \
            typename C,                                                             \
            typename D,                                                             \
            typename E,                                                             \
            typename F,                                                             \
            typename G,                                                             \
            typename H>                                                             \
  struct Policy<A, B, C, D, E, F, G, H> {                                           \
    using value = Underlying<A, B, C, D, E, F, G, H>;                               \
  };                                                                                \
                                                                                    \
  template <typename A,                                                             \
            typename B,                                                             \
            typename C,                                                             \
            typename D,                                                             \
            typename E,                                                             \
            typename F,                                                             \
            typename G,                                                             \
            typename H,                                                             \
            typename I>                                                             \
  struct Policy<A, B, C, D, E, F, G, H, I> {                                        \
    using value = Underlying<A, B, C, D, E, F, G, H, I>;                            \
  };                                                                                \
                                                                                    \
  template <typename... T>                                                          \
  using Policy##_v = Policy<T...>::value;                                           \
                                                                                    \
  template <typename Tuple>                                                         \
  struct Policy##_from_tuple;                                                       \
                                                                                    \
  template <template <typename...> typename List, typename... OtherT>               \
  struct Policy##_from_tuple<List<OtherT...>> {                                     \
    using value = Policy##_v<OtherT...>;                                            \
  };

MAKE_VARIADIC(approx_policy, ApproximateTime)
MAKE_VARIADIC(exact_policy, ExactTime)

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

cv::Mat parseImage(const Image& msg) {
  try {
    return cv_bridge::toCvCopy(msg)->image;
  } catch (const cv_bridge::Exception& e) {
    LOG(ERROR) << "Failed to convert image: " << e.what();
    return cv::Mat();
  }
}

struct NullAdapter {
  using MsgType = message_filters::NullType;

  template <typename RecvT>
  NullAdapter(ianvs::NodeHandle, const std::string&, const rclcpp::QoS&, RecvT&) {}
};

struct ClosedSetAdapter {
  using MsgType = Image;

  template <typename RecvT>
  ClosedSetAdapter(ianvs::NodeHandle nh,
                   const std::string& topic,
                   const rclcpp::QoS& qos,
                   RecvT& receiver)
      : sub(nh.create_subscription<Image>(
            topic, qos, [&receiver](const Image::ConstSharedPtr& msg) {
              receiver.sync.template add<2>(msg);
            })) {}

  static void fill(const Image& msg, ImageInputPacket& packet) {
    packet.labels = parseImage(msg);
  }

  rclcpp::Subscription<Image>::SharedPtr sub;
};

struct InstanceAdapter {
  using MsgType = Image;

  template <typename RecvT>
  InstanceAdapter(ianvs::NodeHandle nh,
                  const std::string& topic,
                  const rclcpp::QoS& qos,
                  RecvT& receiver)
      : sub(nh.create_subscription<Image>(
            topic, qos, [&receiver](const Image::ConstSharedPtr& msg) {
              receiver.sync.template add<2>(msg);
            })) {}

  static void fill(const Image& msg, ImageInputPacket& packet) {
    const auto mat = parseImage(msg);
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

  rclcpp::Subscription<Image>::SharedPtr sub;
};

struct OpenSetAdapter {
  using MsgType = FeatureImage;

  template <typename RecvT>
  OpenSetAdapter(ianvs::NodeHandle nh,
                 const std::string& topic,
                 const rclcpp::QoS& qos,
                 RecvT& receiver)
      : sub(nh.create_subscription<FeatureImage>(
            topic, qos, [&receiver](const FeatureImage::ConstSharedPtr& msg) {
              receiver.sync.template add<2>(msg);
            })) {}

  static void fill(const FeatureImage& msg, ImageInputPacket& packet) {
    packet.instances = parseImage(msg.image);

    CHECK_EQ(msg.mask_ids.size(), msg.features.size());
    for (size_t i = 0; i < msg.mask_ids.size(); ++i) {
      const auto& vec = msg.features[i].data;
      packet.label_features.emplace(
          msg.mask_ids[i], Eigen::Map<const FeatureVector>(vec.data(), vec.size()));
    }
  }

  rclcpp::Subscription<FeatureImage>::SharedPtr sub;
};

template <bool enabled>
struct FeatureAdapter;

template <>
struct FeatureAdapter<false> {
  template <typename RecvT>
  FeatureAdapter(ianvs::NodeHandle, const std::string&, const rclcpp::QoS&, RecvT&) {}
};

template <>
struct FeatureAdapter<true> {
  using MsgType = FeatureVectorStamped;

  template <typename RecvT>
  FeatureAdapter(ianvs::NodeHandle nh,
                 const std::string& topic,
                 const rclcpp::QoS& qos,
                 RecvT& receiver)
      : sub(nh.create_subscription<MsgType>(
            topic, qos, [&receiver](const MsgType::ConstSharedPtr& msg) {
              receiver.sync.template add<RecvT::Info::feature_offset>(msg);
            })) {}

  static void fill(const MsgType& msg, ImageInputPacket& packet) {
    const auto& vec = msg.feature.data;
    packet.input_feature = Eigen::Map<const FeatureVector>(vec.data(), vec.size());
  }

  rclcpp::Subscription<MsgType>::SharedPtr sub;
};

}  // namespace

template <typename T, typename MsgT, bool should_add>
struct add_type;

template <template <typename...> typename List, typename MsgT, typename... OtherT>
struct add_type<List<OtherT...>, MsgT, true> {
  using value = List<OtherT..., MsgT>;
};

template <template <typename...> typename List, typename MsgT, typename... OtherT>
struct add_type<List<OtherT...>, MsgT, false> {
  using value = List<OtherT...>;
};

template <typename T, typename MsgT, bool should_add>
using add_type_v = add_type<T, MsgT, should_add>::value;

template <bool exact>
struct policy_type;

template <>
struct policy_type<true> {
  template <typename... Args>
  using policy_from_tuple = exact_policy_from_tuple<Args...>::value;
};

template <>
struct policy_type<false> {
  template <typename... Args>
  using policy_from_tuple = approx_policy_from_tuple<Args...>::value;
};

template <bool _with_feature, bool exact>
struct ReceiverType : policy_type<exact> {
  static constexpr bool with_feature = _with_feature;
};

struct PacketBuilderBase {
  using ImagePacketPtr = std::shared_ptr<ImageInputPacket>;
  using Queue = MessageQueue<SensorInputPacket::Ptr>;

  PacketBuilderBase(const std::string& sensor_name, Queue& queue)
      : sensor_name(sensor_name), queue(queue) {}

  ImagePacketPtr make_packet(const Image::ConstSharedPtr& color,
                             const Image::ConstSharedPtr& depth) const {
    const auto timestamp_ns = rclcpp::Time(color->header.stamp).nanoseconds();
    auto packet = std::make_shared<ImageInputPacket>(timestamp_ns, sensor_name);
    packet->color = parseColor(*color);
    packet->depth = parseDepth(*depth);
    return packet;
  }

  const std::string sensor_name;
  Queue& queue;
};

template <typename... AdapterT>
void fillPacket(ImageInputPacket& packet,
                const typename AdapterT::MsgType::ConstSharedPtr&... msg);

template <>
void fillPacket(ImageInputPacket&) {}

template <typename AdapterT, typename... OtherT>
void fillPacket(ImageInputPacket& packet,
                const typename AdapterT::MsgType::ConstSharedPtr& msg,
                const typename OtherT::MsgType::ConstSharedPtr&... others) {
  AdapterT::fill(*msg, packet);
  fillPacket<OtherT...>(packet, others...);
}

template <typename T>
struct PacketBuilder;

template <template <typename...> typename List, typename... AdapterT>
struct PacketBuilder<List<AdapterT...>> : PacketBuilderBase {
  PacketBuilder(const std::string& sensor_name, Queue& queue)
      : PacketBuilderBase(sensor_name, queue) {}

  void callback(const Image::ConstSharedPtr& color,
                const Image::ConstSharedPtr& depth,
                const typename AdapterT::MsgType::ConstSharedPtr&... others) {
    auto packet = make_packet(color, depth);
    fillPacket<AdapterT...>(*packet, others...);
    queue.push(packet);
  }
};

template <typename... Args>
struct type_list {};

template <typename T, typename R>
struct ReceiverInfo {
  static constexpr bool is_null = std::is_same_v<T, NullAdapter>;
  static constexpr size_t feature_offset = is_null ? 2 : 3;

  using vec = FeatureVectorStamped;
  using msg = typename T::MsgType;

  using adapters = add_type_v<add_type_v<type_list<>, T, !is_null>,
                              FeatureAdapter<true>,
                              R::with_feature>;

  using types = add_type_v<add_type_v<type_list<Image, Image>, msg, !is_null>,
                           FeatureVectorStamped,
                           R::with_feature>;

  using policy = R::template policy_from_tuple<types>;
  using builder = PacketBuilder<adapters>;
};

struct ImageReceiverBase {
  virtual ~ImageReceiverBase() = default;
};

template <typename AdapterT, typename TypeT>
struct ImageReceiverImpl : public ImageReceiverBase {
  using Queue = PacketBuilderBase::Queue;
  using ImgPtr = Image::ConstSharedPtr;

  using Info = ReceiverInfo<AdapterT, TypeT>;
  using Sync = Synchronizer<typename Info::policy>;

  ImageReceiverImpl(ianvs::NodeHandle nh,
                    const std::string& name,
                    const rclcpp::QoS& qos,
                    size_t queue_size,
                    Queue& queue);

  Sync sync;
  Info::builder builder;

  rclcpp::Subscription<Image>::SharedPtr color;
  rclcpp::Subscription<Image>::SharedPtr depth;
  AdapterT semantics;
  FeatureAdapter<TypeT::with_feature> feature;
  rclcpp::Subscription<Image>::SharedPtr traversability;
};

template <typename AdapterT, typename TypeT>
ImageReceiverImpl<AdapterT, TypeT>::ImageReceiverImpl(ianvs::NodeHandle nh,
                                                      const std::string& name,
                                                      const rclcpp::QoS& qos,
                                                      size_t queue_size,
                                                      Queue& queue)
    : sync(queue_size),
      builder(name, queue),
      color(nh.create_subscription<Image>(
          "rgb/image_raw",
          qos,
          [this](const ImgPtr& msg) { sync.template add<0>(msg); })),
      depth(nh.create_subscription<Image>(
          "depth_registered/image_rect",
          qos,
          [this](const ImgPtr& msg) { sync.template add<1>(msg); })),
      semantics(nh, "semantic/image_raw", qos, *this),
      feature(nh, "semantic/feature", qos, *this) {
  sync.registerCallback(&Info::builder::callback, &builder);
}

template <typename AdapterT, bool with_feature>
using ExactRecv = ImageReceiverImpl<AdapterT, ReceiverType<with_feature, true>>;

template <typename AdapterT, bool with_feature>
using ApproxRecv = ImageReceiverImpl<AdapterT, ReceiverType<with_feature, false>>;

template <typename T>
std::unique_ptr<ImageReceiverBase> makeReceiver(const ImageReceiver::Config& config,
                                                ianvs::NodeHandle nh,
                                                const std::string& name,
                                                PacketBuilderBase::Queue& queue) {
  const auto qos = config.qos;
  const auto queue_size = config.queue_size;
  if (config.use_exact) {
    if (config.with_feature) {
      return std::make_unique<ExactRecv<T, true>>(nh, name, qos, queue_size, queue);
    } else {
      return std::make_unique<ExactRecv<T, false>>(nh, name, qos, queue_size, queue);
    }
  } else {
    if (config.with_feature) {
      return std::make_unique<ApproxRecv<T, true>>(nh, name, qos, queue_size, queue);
    } else {
      return std::make_unique<ApproxRecv<T, false>>(nh, name, qos, queue_size, queue);
    }
  }
}

struct ImageReceiver::Impl {
  explicit Impl(const ImageReceiver::Config& config,
                ianvs::NodeHandle nh,
                const std::string& name,
                PacketBuilderBase::Queue& queue) {
    switch (config.semantics_type) {
      case ImageReceiver::Config::SemanticsType::NONE:
        recv = makeReceiver<NullAdapter>(config, nh, name, queue);
        break;
      case ImageReceiver::Config::SemanticsType::CLOSED_SET:
        recv = makeReceiver<ClosedSetAdapter>(config, nh, name, queue);
        break;
      case ImageReceiver::Config::SemanticsType::INSTANCE:
        recv = makeReceiver<InstanceAdapter>(config, nh, name, queue);
        break;
      case ImageReceiver::Config::SemanticsType::OPEN_SET:
        recv = makeReceiver<OpenSetAdapter>(config, nh, name, queue);
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
  impl_.reset(new Impl(config, nh, sensor_name, queue_));
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
