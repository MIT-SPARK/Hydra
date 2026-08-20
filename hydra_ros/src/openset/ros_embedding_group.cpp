#include "hydra_ros/openset/ros_embedding_group.h"

#include <config_utilities/config.h>
#include <config_utilities/factory.h>
#include <glog/logging.h>
#include <hydra/openset/openset_types.h>
#include <ianvs/message_wait_functor.h>

#include <semantic_inference_msgs/msg/feature_vectors.hpp>
#include <semantic_inference_msgs/srv/encode_feature.hpp>

using semantic_inference_msgs::msg::FeatureVectors;
using semantic_inference_msgs::srv::EncodeFeature;

namespace hydra {
namespace {

static const auto registration =
    config::RegistrationWithConfig<EmbeddingGroup,
                                   RosEmbeddingGroup,
                                   RosEmbeddingGroup::Config>("RosEmbeddingGroup");

}

using namespace std::chrono_literals;

void declare_config(RosEmbeddingGroup::Config& config) {
  using namespace config;
  name("RosEmbeddingGroup::Config");
  base<VerbosityConfig>(config);
  field(config.ns, "ns");
  field(config.prompt_timeout_ms, "propmt_timeout_ms");
  field(config.prompts, "prompts");
}

RosEmbeddingGroup::Config::Config() : VerbosityConfig("[RosEmbeddingGroup] ") {}

RosEmbeddingGroup::RosEmbeddingGroup(const Config& config) {
  if (config.prompts.empty()) {
    auto nh = ianvs::NodeHandle::this_node(config.ns);
    const auto topic_name = nh.resolve_name("features", false);
    MLOG(1) << "Waiting for embeddings on '" << topic_name << "'";

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

    MLOG(1) << "Got embeddings from '" << topic_name << "'!";
    return;
  }

  auto nh = ianvs::NodeHandle::this_node(config.ns);
  const auto service_name = nh.resolve_name("embed", true);
  MLOG(1) << "Waiting for embedding encoder on '" << service_name << "'...";

  auto client = nh.create_client<EncodeFeature>("embed");
  while (!client->wait_for_service(10ms) && rclcpp::ok()) {
    MLOG(3) << "Waiting for embedding encoder on '" << service_name << "'...";
  }

  if (!rclcpp::ok()) {
    return;
  }

  MLOG(1) << "Embedding prompts on '" << service_name << "'";

  for (const auto& prompt : config.prompts) {
    auto req = std::make_shared<EncodeFeature::Request>();
    req->prompt = prompt;

    MLOG(2) << "Requesting embedding for '" << prompt << "'";
    auto rep = ianvs::call_service(*client, req, config.prompt_timeout_ms, &nh);
    if (!rep) {
      LOG(ERROR) << "Failed to get result for '" << prompt << "'";
      continue;
    }

    MLOG(2) << "Got embedding for '" << prompt << "'";

    names.push_back(prompt);
    const auto& vec = rep.get()->feature.feature.data;
    embeddings.emplace_back(Eigen::Map<const FeatureVector>(vec.data(), vec.size()));
  }

  MLOG(1) << "Finished embedding prompts using '" << service_name << "'";
}

}  // namespace hydra
