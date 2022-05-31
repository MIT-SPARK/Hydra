#pragma once
#include "hydra_dsg_builder/config_utils.h"
#include "hydra_dsg_builder/dsg_lcd_detector.h"

namespace teaser {

using TeaserInlierSelectionMode = RobustRegistrationSolver::INLIER_SELECTION_MODE;

}  // namespace teaser

DECLARE_CONFIG_ENUM(hydra::lcd,
                    DescriptorScoreType,
                    {DescriptorScoreType::COSINE, "COSINE"},
                    {DescriptorScoreType::L1, "L1"})

DECLARE_CONFIG_ENUM(teaser,
                    TeaserInlierSelectionMode,
                    {TeaserInlierSelectionMode::PMC_EXACT, "PMC_EXACT"},
                    {TeaserInlierSelectionMode::PMC_HEU, "PMC_HEU"},
                    {TeaserInlierSelectionMode::KCORE_HEU, "KCORE_HEU"},
                    {TeaserInlierSelectionMode::NONE, "NONE"})

namespace config_parser {

template <typename T>
struct ConfigVisitor<std::map<hydra::LayerId, T>> {
  using ConfigMap = std::map<hydra::LayerId, T>;
  using MapType = typename ConfigMap::mapped_type;

  template <typename V, typename std::enable_if<is_parser<V>::value, bool>::type = true>
  static auto visit_config(const V& v, ConfigMap& value) {
    for (const auto& child : v.children()) {
      const auto layer = hydra::KimeraDsgLayers::StringToLayerId(child);
      value[layer] = MapType();
      v.visit(child, value[layer]);
    }
  }

  template <typename V,
            typename std::enable_if<!is_parser<V>::value, bool>::type = true>
  static auto visit_config(const V& v, ConfigMap& value) {
    for (auto& kv_pair : value) {
      const auto layer_str = hydra::KimeraDsgLayers::LayerIdToString(kv_pair.first);
      v.visit(layer_str, kv_pair.second);
    }
  }
};

}  // namespace config_parser

namespace hydra {

struct DsgLcdModuleConfig {
  lcd::DsgLcdDetectorConfig detector;
  bool visualize_dsg_lcd = false;
  std::string lcd_visualizer_ns = "/dsg/lcd_visualizer";
  double lcd_agent_horizon_s = 1.5;
  double descriptor_creation_horizon_m = 10.0;
};

namespace lcd {

template <typename Visitor>
void visit_config(const Visitor& v, LayerRegistrationConfig& config) {
  v.visit("min_correspondences", config.min_correspondences);
  v.visit("min_inliers", config.min_inliers);
  v.visit("log_registration_problem", config.log_registration_problem);
  v.visit("registration_output_path", config.registration_output_path);
}

template <typename Visitor>
void visit_config(const Visitor& v, DescriptorMatchConfig& config) {
  v.visit("min_score", config.min_score);
  v.visit("min_registration_score", config.min_registration_score);
  v.visit("min_time_separation_s", config.min_time_separation_s);
  v.visit("max_registration_matches", config.max_registration_matches);
  v.visit("min_score_ratio", config.min_score_ratio);
  v.visit("min_match_separation_m", config.min_match_separation_m);
  v.visit("type", config.type);
}

template <typename Visitor, typename T>
void visit_config(const Visitor& v, HistogramConfig<T>& config) {
  v.visit("min", config.min);
  v.visit("max", config.max);
  v.visit("bins", config.bins);
}

template <typename Visitor>
void visit_config(const Visitor& v, DsgLcdDetectorConfig& config) {
  v.visit("search_configs", config.search_configs);
  auto v_search = v["search_configs"];
  v_search.visit("agent", config.agent_search_config);
  v.visit("registration_configs", config.registration_configs);
  v.visit("teaser", config.teaser_config);
  v.visit("enable_agent_registration", config.enable_agent_registration);
  v.visit("object_radius_m", config.object_radius_m);
  v.visit("num_semantic_classes", config.num_semantic_classes);
  v.visit("place_radius_m", config.place_radius_m);
  v.visit("place_histogram_config", config.place_histogram_config);
  if (config_parser::is_parser<Visitor>()) {
    config.agent_search_config.min_registration_score =
        config.agent_search_config.min_score;
  }
}

}  // namespace lcd

template <typename Visitor>
void visit_config(const Visitor& v, DsgLcdModuleConfig& config) {
  v.visit("lcd", config.detector);
  v.visit("visualize_dsg_lcd", config.visualize_dsg_lcd);
  v.visit("lcd_visualizer_ns", config.lcd_visualizer_ns);
  v.visit("lcd_agent_horizon_s", config.lcd_agent_horizon_s);
  v.visit("descriptor_creation_horizon_m", config.descriptor_creation_horizon_m);
}

}  // namespace hydra

namespace teaser {

template <typename Visitor>
void visit_config(const Visitor& v, teaser::RobustRegistrationSolver::Params& config) {
  v.visit("estimate_scaling", config.estimate_scaling);
  v.visit("cbar2", config.cbar2);
  v.visit("rotation_gnc_factor", config.rotation_gnc_factor);
  v.visit("rotation_max_iterations", config.rotation_max_iterations);
  v.visit("kcore_heuristic_threshold", config.kcore_heuristic_threshold);
  v.visit("inlier_selection_mode", config.inlier_selection_mode);
  v.visit("max_clique_time_limit", config.max_clique_time_limit);
}

}  // namespace teaser

DECLARE_CONFIG_OSTREAM_OPERATOR(teaser, RobustRegistrationSolver::Params)
DECLARE_CONFIG_OSTREAM_OPERATOR(hydra::lcd, HistogramConfig<double>)
DECLARE_CONFIG_OSTREAM_OPERATOR(hydra::lcd, LayerRegistrationConfig)
DECLARE_CONFIG_OSTREAM_OPERATOR(hydra::lcd, DescriptorMatchConfig)
DECLARE_CONFIG_OSTREAM_OPERATOR(hydra::lcd, DsgLcdDetectorConfig)
DECLARE_CONFIG_OSTREAM_OPERATOR(hydra, DsgLcdModuleConfig)
