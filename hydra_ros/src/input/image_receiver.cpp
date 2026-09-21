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
#include <hydra/utils/timing_utilities.h>
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

#include "hydra_ros/input/message_sync_queue.h"

using semantic_inference_msgs::msg::FeatureImage;
using semantic_inference_msgs::msg::FeatureVectorStamped;
using sensor_msgs::msg::Image;

using message_filters::Synchronizer;
using message_filters::sync_policies::ApproximateTime;
using message_filters::sync_policies::ExactTime;
using rclcpp::node_interfaces::NodeBaseInterface;

namespace hydra {
namespace {

static constexpr auto MutexGroup = rclcpp::CallbackGroupType::MutuallyExclusive;

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
                                   Sensor::ConstPtr,
                                   DataReceiver::OutputQueue::Ptr>("ImageReceiver");

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
      : group(nh.as<NodeBaseInterface>()->create_callback_group(MutexGroup)),
        sub(nh.create_subscription<Image>(
            topic,
            qos,
            [&receiver](const Image::ConstSharedPtr& msg) {
              receiver.sync.template add<2>(msg);
            },
            group)) {}

  static void fill(const Image& msg, ImageInputPacket& packet) {
    packet.labels = parseImage(msg);
  }

  rclcpp::CallbackGroup::SharedPtr group;
  rclcpp::Subscription<Image>::SharedPtr sub;
};

struct InstanceAdapter {
  using MsgType = Image;

  template <typename RecvT>
  InstanceAdapter(ianvs::NodeHandle nh,
                  const std::string& topic,
                  const rclcpp::QoS& qos,
                  RecvT& receiver)
      : group(nh.as<NodeBaseInterface>()->create_callback_group(MutexGroup)),
        sub(nh.create_subscription<Image>(
            topic,
            qos,
            [&receiver](const Image::ConstSharedPtr& msg) {
              receiver.sync.template add<2>(msg);
            },
            group)) {}

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

  rclcpp::CallbackGroup::SharedPtr group;
  rclcpp::Subscription<Image>::SharedPtr sub;
};

struct OpenSetAdapter {
  using MsgType = FeatureImage;

  template <typename RecvT>
  OpenSetAdapter(ianvs::NodeHandle nh,
                 const std::string& topic,
                 const rclcpp::QoS& qos,
                 RecvT& receiver)
      : group(nh.as<NodeBaseInterface>()->create_callback_group(MutexGroup)),
        sub(nh.create_subscription<FeatureImage>(
            topic,
            qos,
            [&receiver](const FeatureImage::ConstSharedPtr& msg) {
              receiver.sync.template add<2>(msg);
            },
            group)) {}

  static void fill(const FeatureImage& msg, ImageInputPacket& packet) {
    packet.instances = parseImage(msg.image);

    CHECK_EQ(msg.mask_ids.size(), msg.features.size());
    for (size_t i = 0; i < msg.mask_ids.size(); ++i) {
      const auto& vec = msg.features[i].data;
      packet.label_features.emplace(
          msg.mask_ids[i], Eigen::Map<const FeatureVector>(vec.data(), vec.size()));
    }
  }

  rclcpp::CallbackGroup::SharedPtr group;
  rclcpp::Subscription<FeatureImage>::SharedPtr sub;
};

template <bool enabled>
struct TraversabilityAdapter;

template <>
struct TraversabilityAdapter<false> {
  template <typename RecvT>
  TraversabilityAdapter(ianvs::NodeHandle,
                        const std::string&,
                        const rclcpp::QoS&,
                        RecvT&) {}
};

template <>
struct TraversabilityAdapter<true> {
  using MsgType = Image;

  template <typename RecvT>
  TraversabilityAdapter(ianvs::NodeHandle nh,
                        const std::string& topic,
                        const rclcpp::QoS& qos,
                        RecvT& receiver)
      : group(nh.as<NodeBaseInterface>()->create_callback_group(MutexGroup)),
        sub(nh.create_subscription<Image>(
            topic,
            qos,
            [&receiver](const Image::ConstSharedPtr& msg) {
              receiver.sync.template add<RecvT::Info::traversability_offset>(msg);
            },
            group)) {}

  static void fill(const Image& msg, ImageInputPacket& packet) {
    packet.traversability = parseImage(msg);
  }

  rclcpp::CallbackGroup::SharedPtr group;
  rclcpp::Subscription<Image>::SharedPtr sub;
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

template <bool _with_traversability, bool exact>
struct ReceiverType : policy_type<exact> {
  static constexpr bool with_traversability = _with_traversability;
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

struct PacketBuilderBase {
  using ImagePacketPtr = std::shared_ptr<ImageInputPacket>;
  using Callback = std::function<void(SensorInputPacket::Ptr)>;

  explicit PacketBuilderBase(const Callback& push) : push(push) {}

  ImagePacketPtr make_packet(const Image::ConstSharedPtr& color,
                             const Image::ConstSharedPtr& depth) const {
    const auto timestamp_ns = rclcpp::Time(color->header.stamp).nanoseconds();
    auto packet = std::make_shared<ImageInputPacket>(timestamp_ns);
    packet->color = parseColor(*color);
    packet->depth = parseDepth(*depth);
    return packet;
  }

  const Callback push;
};

template <typename T>
struct PacketBuilder;

template <template <typename...> typename List, typename... AdapterT>
struct PacketBuilder<List<AdapterT...>> : PacketBuilderBase {
  PacketBuilder(const Callback& callback) : PacketBuilderBase(callback) {}

  void callback(const Image::ConstSharedPtr& color,
                const Image::ConstSharedPtr& depth,
                const typename AdapterT::MsgType::ConstSharedPtr&... others) {
    const auto timestamp_ns = rclcpp::Time(color->header.stamp).nanoseconds();
    timing::ScopedTimer timer("input/packet_creation", timestamp_ns);

    auto packet = make_packet(color, depth);
    fillPacket<AdapterT...>(*packet, others...);
    push(packet);
  }
};

template <typename... Args>
struct type_list {};

template <typename T, typename R>
struct ReceiverInfo {
  static constexpr bool is_null = std::is_same_v<T, NullAdapter>;
  static constexpr size_t traversability_offset = is_null ? 2 : 3;

  using msg = typename T::MsgType;

  using adapters = add_type_v<add_type_v<type_list<>, T, !is_null>,
                              TraversabilityAdapter<true>,
                              R::with_traversability>;

  using types = add_type_v<add_type_v<type_list<Image, Image>, msg, !is_null>,
                           Image,
                           R::with_traversability>;

  using policy = R::template policy_from_tuple<types>;
  using builder = PacketBuilder<adapters>;
};

struct ImageReceiverBase {
  virtual ~ImageReceiverBase() = default;
};

using PacketQueue = MessageQueue<ImageInputPacket::Ptr>;

template <typename AdapterT, typename TypeT>
struct ImageReceiverImpl : public ImageReceiverBase {
  using ImgPtr = Image::ConstSharedPtr;

  using Info = ReceiverInfo<AdapterT, TypeT>;
  using Sync = Synchronizer<typename Info::policy>;
  using FeatureQueue = MessageSyncQueue<FeatureVectorStamped>;

  ImageReceiverImpl(ianvs::NodeHandle nh,
                    const rclcpp::QoS& qos,
                    size_t queue_size,
                    bool with_feature,
                    PacketQueue& queue);

  void push(ImageInputPacket::Ptr packet);

  Sync sync;
  PacketQueue& queue;
  Info::builder builder;

  rclcpp::CallbackGroup::SharedPtr color_group;
  rclcpp::Subscription<Image>::SharedPtr color;

  rclcpp::CallbackGroup::SharedPtr depth_group;
  rclcpp::Subscription<Image>::SharedPtr depth;

  AdapterT semantics;
  std::unique_ptr<FeatureQueue> features;
  TraversabilityAdapter<TypeT::with_traversability> traversability;
};

template <typename AdapterT, typename TypeT>
ImageReceiverImpl<AdapterT, TypeT>::ImageReceiverImpl(ianvs::NodeHandle nh,
                                                      const rclcpp::QoS& qos,
                                                      size_t queue_size,
                                                      bool with_feature,
                                                      PacketQueue& queue)
    : sync(queue_size),
      queue(queue),
      builder([this](auto packet) { push(packet); }),
      color_group(nh.as<NodeBaseInterface>()->create_callback_group(MutexGroup)),
      color(nh.create_subscription<Image>(
          "rgb/image_raw",
          qos,
          [this](const ImgPtr& msg) { sync.template add<0>(msg); },
          color_group)),
      depth_group(nh.as<NodeBaseInterface>()->create_callback_group(MutexGroup)),
      depth(nh.create_subscription<Image>(
          "depth_registered/image_rect",
          qos,
          [this](const ImgPtr& msg) { sync.template add<1>(msg); },
          depth_group)),
      semantics(nh, "semantic/image_raw", qos, *this),
      traversability(nh, "traversability/image_raw", qos, *this) {
  sync.registerCallback(&Info::builder::callback, &builder);
  if (with_feature) {
    MessageSyncQueueConfig feature_config{queue_size, qos};
    features = std::make_unique<FeatureQueue>(feature_config, nh, "semantic/feature");
  }
}

template <typename AdapterT, typename TypeT>
void ImageReceiverImpl<AdapterT, TypeT>::push(ImageInputPacket::Ptr packet) {
  if (!packet) {
    return;
  }

  if (features) {
    auto msg = features->sync(packet->timestamp_ns);
    if (msg) {
      const auto& vec = msg->feature.data;
      packet->input_feature = Eigen::Map<const FeatureVector>(vec.data(), vec.size());
    }
  }

  queue.push(packet);
}

template <typename T, bool traversability>
using ExactRecv = ImageReceiverImpl<T, ReceiverType<traversability, true>>;

template <typename T, bool traversability>
using ApproxRecv = ImageReceiverImpl<T, ReceiverType<traversability, false>>;

template <typename T, template <typename, bool> typename RecvT>
std::unique_ptr<ImageReceiverBase> makeReceiver(const ImageReceiver::Config& config,
                                                ianvs::NodeHandle nh,
                                                PacketQueue& queue) {
  const auto qos = config.qos;
  const auto size = config.queue_size;
  if (config.with_traversability) {
    return std::make_unique<RecvT<T, true>>(nh, qos, size, config.with_feature, queue);
  } else {
    return std::make_unique<RecvT<T, false>>(nh, qos, size, config.with_feature, queue);
  }
}

template <typename T>
std::unique_ptr<ImageReceiverBase> makeReceiver(const ImageReceiver::Config& config,
                                                ianvs::NodeHandle nh,
                                                PacketQueue& queue) {
  if (config.use_exact) {
    return makeReceiver<T, ExactRecv>(config, nh, queue);
  } else {
    return makeReceiver<T, ApproxRecv>(config, nh, queue);
  }
}

struct ImageReceiver::Impl {
  explicit Impl(const ImageReceiver::Config& config,
                ianvs::NodeHandle nh,
                PacketQueue& queue) {
    switch (config.semantics_type) {
      case ImageReceiver::Config::SemanticsType::NONE:
        recv = makeReceiver<NullAdapter>(config, nh, queue);
        break;
      case ImageReceiver::Config::SemanticsType::CLOSED_SET:
        recv = makeReceiver<ClosedSetAdapter>(config, nh, queue);
        break;
      case ImageReceiver::Config::SemanticsType::INSTANCE:
        recv = makeReceiver<InstanceAdapter>(config, nh, queue);
        break;
      case ImageReceiver::Config::SemanticsType::OPEN_SET:
        recv = makeReceiver<OpenSetAdapter>(config, nh, queue);
        break;
    }
  }

  std::unique_ptr<ImageReceiverBase> recv;
};

void declare_config(ImageReceiver::Config& config) {
  using namespace config;
  name("ImageReceiver::Config");
  base<RosDataReceiver::Config>(config);
  enum_field(config.semantics_type,
             "semantics_type",
             {{ImageReceiver::Config::SemanticsType::NONE, "none"},
              {ImageReceiver::Config::SemanticsType::CLOSED_SET, "closed_set"},
              {ImageReceiver::Config::SemanticsType::INSTANCE, "instance"},
              {ImageReceiver::Config::SemanticsType::OPEN_SET, "open_set"}});
  field(config.with_feature, "with_feature");
  field(config.with_traversability, "with_traversability");
  field(config.use_exact, "use_exact");
  field(config.queue_size, "queue_size");
  field(config.qos, "qos");
}

ImageReceiver::ImageReceiver(const Config& config,
                             const Sensor::ConstPtr& sensor,
                             const OutputQueue::Ptr& output)
    : RosDataReceiver(config, sensor, output), config(config) {
  if (config.queue_size <= 2 && !config.use_exact) {
    LOG(WARNING) << "ApproximateTime policy requires queue sizes larger than 2";
  }
}

ImageReceiver::~ImageReceiver() = default;

void ImageReceiver::stop() {
  impl_.reset();  // we want cancel subscriptions before stopping the receiver thread
  DataReceiver::stop();
}

bool ImageReceiver::initImpl() {
  auto nh = ianvs::NodeHandle::this_node(ns_);
  impl_.reset(new Impl(config, nh, queue_));
  return true;
}

}  // namespace hydra
