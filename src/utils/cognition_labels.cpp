#include "hydra/utils/cognition_labels.h"

#include <config_utilities/config.h>
#include <config_utilities/dynamic_config.h>
#include <config_utilities/printing.h>
#include <config_utilities/types/enum.h>
#include <config_utilities/validation.h>
#include <glog/logging.h>

namespace hydra {

using FeatureType = CognitionLabels::Config::FeatureType;

void declare_config(CognitionLabels::Config& config) {
  using namespace config;
  name("CognitionLabels::Config");
  field(config.ns, "ns");
  enum_field(config.feature_type,
             "feature_type",
             {{FeatureType::CLIP, "CLIP"},
              {FeatureType::SENTENCE, "SENTENCE"},
              {FeatureType::BOTH, "BOTH"}});
  field(config.distance_metric, "distance_metric");
}

CognitionLabels::CognitionLabels(const Config& config)
    : config(config::checkValid(config)),
      distance_metric_(config.distance_metric.create()) {
  LOG(INFO) << "Using CognitionLabels:\n" << config::toString(config);
}

void CognitionLabels::setup(const Config& config) {
  instance_.reset(new CognitionLabels(config));
}

FeatureVector getClipFeature(const nlohmann::json& meta) {
  if (!meta.contains("clip_feature") || meta["clip_feature"].is_null()) {
    return {};
  }
  const auto clip_vec = meta["clip_feature"].get<std::vector<float>>();
  if (clip_vec.empty()) {
    return {};
  }
  return Eigen::Map<const FeatureVector>(clip_vec.data(), clip_vec.size());
}

FeatureVector getSentenceFeature(const nlohmann::json& meta) {
  if (!meta.contains("sentence_embedding_feature") ||
      meta["sentence_embedding_feature"].is_null()) {
    return {};
  }
  const auto sentence_vec =
      meta["sentence_embedding_feature"].get<std::vector<float>>();
  if (sentence_vec.empty()) {
    return {};
  }
  return Eigen::Map<const FeatureVector>(sentence_vec.data(), sentence_vec.size());
}

FeatureVector CognitionLabels::getFeature(const spark_dsg::DynamicSceneGraph& dsg,
                                          int id) {
  // Lookup the ID in the DSG.
  const auto& config = instance().config;
  if (!dsg.metadata.get().contains(config.ns)) {
    // std::stringstream ss;
    // for (auto it = dsg.metadata.get().begin(); it != dsg.metadata.get().end(); ++it)
    // {
    //   ss << " " << it.key();
    // }
    // LOG(ERROR) << "DSG does not contain namespace '" << config.ns
    //            << "', contains:" << ss.str();
    return {};
  }
  const auto string_id = std::to_string(id);
  const auto meta = dsg.metadata.get().at(config.ns);
  if (!meta.contains(string_id)) {
    // std::stringstream ss;
    // for (auto it = meta.begin(); it != meta.end(); ++it) {
    //   ss << " " << it.key();
    // }
    // LOG(ERROR) << "DSG metadata does not contain ID " << string_id << " in namespace
    // "
    //            << config.ns << ", contains:" << ss.str();
    return {};
  }
  const auto& value = meta.at(string_id);

  switch (config.feature_type) {
    case FeatureType::CLIP:
      return getClipFeature(value);

    case FeatureType::SENTENCE:
      return getSentenceFeature(value);

    case FeatureType::BOTH: {
      const auto clip_feature = getClipFeature(value);
      const auto sentence_feature = getSentenceFeature(value);
      if (clip_feature.size() == 0 || sentence_feature.size() == 0) {
        return {};
      }
      FeatureVector feature(clip_feature.size() + sentence_feature.size());
      feature << clip_feature, sentence_feature;
      return feature;
    }
    default:
      LOG(ERROR) << "Unhandled feature type.";
      return {};
  }
}

float CognitionLabels::getScore(const spark_dsg::DynamicSceneGraph& dsg,
                                int id1,
                                int id2) {
  const auto f1 = getFeature(dsg, id1);
  if (f1.size() == 0) {
    return 0.0f;
  }
  return getScore(f1, getFeature(dsg, id2));
}

float CognitionLabels::getScore(const FeatureVector& f1, const FeatureVector& f2) {
  if (f1.size() == 0 || f2.size() == 0) {
    return 0.0f;
  }
  return instance().distance_metric_->score(f1, f2);
}

CognitionLabels& CognitionLabels::instance() {
  if (!instance_) {
    setup();
  }
  return *instance_;
}

std::unique_ptr<CognitionLabels> CognitionLabels::instance_ = nullptr;

const FeatureVector& LazyCognitionLabels::get(int id) const {
  auto it = cache_.find(id);
  if (it != cache_.end()) {
    return it->second;
  }
  it = cache_.emplace(id, CognitionLabels::getFeature(dsg_, id)).first;
  return it->second;
}

std::pair<int, float> getMaxCognitionLabel(const std::map<int, float>& labels) {
  int max_label = -1;
  float max_weight = -1.0f;
  float total_weight = 0.0f;

  for (const auto& [label, weight] : labels) {
    total_weight += weight;
    if (weight > max_weight) {
      max_weight = weight;
      max_label = label;
    }
  }
  if (total_weight > 0.0f) {
    max_weight /= total_weight;
  }
  return {max_label, max_weight};
}

}  // namespace hydra
