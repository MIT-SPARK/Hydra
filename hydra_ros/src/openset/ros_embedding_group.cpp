#include "hydra_ros/openset/ros_embedding_group.h"

#include <config_utilities/config.h>
#include <glog/logging.h>
#include <hydra/openset/openset_types.h>
#include <ianvs/message_wait_functor.h>

#include <semantic_inference_msgs/msg/feature_vectors.hpp>
#include <semantic_inference_msgs/srv/encode_feature.hpp>

using semantic_inference_msgs::msg::FeatureVectors;
using semantic_inference_msgs::srv::EncodeFeature;

namespace hydra {

using namespace std::chrono_literals;

template <typename T>
std::future_status wait_for_result(T& future, std::chrono::seconds timeout) {
  auto status = std::future_status::timeout;
  const auto start = std::chrono::steady_clock::now();
  while (status != std::future_status::ready && rclcpp::ok()) {
    if (std::chrono::steady_clock::now() - start > timeout) {
      break;
    }

    status = future.wait_for(100ms);
  }

  return status;
}

void declare_config(RosEmbeddingGroup::Config& config) {
  using namespace config;
  name("RosEmbeddingGroup::Config");
  field(config.ns, "ns");
  field(config.silent_wait, "silent_wait");
  field(config.prompts, "prompts");
}

RosEmbeddingGroup::RosEmbeddingGroup(const Config& config) {
  if (config.prompts.empty()) {
    auto nh = ianvs::NodeHandle::this_node(config.ns);
    const auto topic_name = nh.resolve_name("features", false);
    LOG_IF(INFO, !config.silent_wait)
        << "Waiting for embeddings on '" << topic_name << "'";

    const auto msg = ianvs::getSingleMessage<FeatureVectors>(nh, "features", true);
    if (!msg) {
      LOG(ERROR) << "Failed to get embeddings from '" << topic_name << "'";
      return;
    }

    for (size_t i = 0; i < msg->features.size(); ++i) {
      const auto& vec = msg->features[i].data;
      embeddings.emplace_back(Eigen::Map<const FeatureVector>(vec.data(), vec.size()));
      if (i < msg->names.size()) {
        names.push_back(msg->names[i]);
      }
    }

    LOG_IF(INFO, !config.silent_wait) << "Got embeddings from '" << topic_name << "'!";
    return;
  }

  auto nh = ianvs::NodeHandle::this_node(config.ns);
  const auto service_name = nh.resolve_name("embed", true);
  LOG_IF(INFO, !config.silent_wait)
      << "Waiting for embedding encoder on '" << service_name << "'...";

  auto client = nh.create_client<EncodeFeature>("embed");
  while (!client->wait_for_service(10ms) && rclcpp::ok()) {
  }

  for (const auto& prompt : config.prompts) {
    auto msg = std::make_unique<EncodeFeature::Request>();
    msg->prompt = prompt;
    auto rep = client->async_send_request(std::move(msg)).future.share();
    if (wait_for_result(rep, 1s) != std::future_status::ready) {
      LOG(ERROR) << "Failed to get result for '" << prompt << "'";
      continue;
    }

    names.push_back(prompt);
    const auto& vec = rep.get()->feature.feature.data;
    embeddings.emplace_back(Eigen::Map<const FeatureVector>(vec.data(), vec.size()));
  }

  LOG_IF(INFO, !config.silent_wait)
      << "Finished embedding prompts on using '" << service_name << "'!";
}

}  // namespace hydra
