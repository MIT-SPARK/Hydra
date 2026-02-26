#include "hydra/utils/daaam_labels.h"

#include <config_utilities/config.h>
#include <config_utilities/dynamic_config.h>
#include <config_utilities/printing.h>
#include <config_utilities/types/enum.h>
#include <config_utilities/validation.h>
#include <glog/logging.h>

namespace hydra {

using FeatureType = DaaamLabels::Config::FeatureType;

void declare_config(DaaamLabels::Config& config) {
  using namespace config;
  name("DaaamLabels::Config");
  field(config.ns, "ns");
  enum_field(config.feature_type,
             "feature_type",
             {{FeatureType::CLIP, "CLIP"},
              {FeatureType::SENTENCE, "SENTENCE"},
              {FeatureType::BOTH, "BOTH"}});
  field(config.distance_metric, "distance_metric");
}

DaaamLabels::DaaamLabels(const Config& config)
    : config(config::checkValid(config)),
      distance_metric_(config.distance_metric.create()) {}

void DaaamLabels::setup(const Config& config) {
  instance_.reset(new DaaamLabels(config));
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

FeatureVector DaaamLabels::getFeature(const spark_dsg::DynamicSceneGraph& dsg,
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

float DaaamLabels::getScore(const spark_dsg::DynamicSceneGraph& dsg,
                                int id1,
                                int id2) {
  const auto f1 = getFeature(dsg, id1);
  if (f1.size() == 0) {
    return 0.0f;
  }
  return getScore(f1, getFeature(dsg, id2));
}

float DaaamLabels::getScore(const FeatureVector& f1, const FeatureVector& f2) {
  if (f1.size() == 0 || f2.size() == 0) {
    return 0.0f;
  }
  return instance().distance_metric_->score(f1, f2);
}

DaaamLabels& DaaamLabels::instance() {
  if (!instance_) {
    setup();
  }
  return *instance_;
}

std::unique_ptr<DaaamLabels> DaaamLabels::instance_ = nullptr;

const FeatureVector& LazyDaaamLabels::get(int id) const {
  auto it = cache_.find(id);
  if (it != cache_.end()) {
    return it->second;
  }
  it = cache_.emplace(id, DaaamLabels::getFeature(dsg_, id)).first;
  return it->second;
}

std::pair<int, float> getMaxDaaamLabel(const std::map<spark_dsg::SemanticLabel, float>& labels) {
  int max_label = -1;
  float max_weight = -1.0f;
  float total_weight = 0.0f;

  for (auto it = labels.begin(); it != labels.end(); ++it) {
    total_weight += it->second;
    if (it->second > max_weight) {
      max_weight = it->second;
      max_label = static_cast<int>(it->first);
    }
  }
  if (total_weight > 0.0f) {
    max_weight /= total_weight;
  }
  return {max_label, max_weight};
}

}  // namespace hydra
