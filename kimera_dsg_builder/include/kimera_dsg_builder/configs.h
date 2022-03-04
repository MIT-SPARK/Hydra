#pragma once
#include "kimera_dsg_builder/dsg_lcd_module.h"
#include "kimera_dsg_builder/incremental_room_finder.h"

#include <KimeraRPGO/SolverParams.h>
#include <hydra_utils/config.h>
#include <hydra_utils/eigen_config_types.h>
#include <voxblox_ros/mesh_vis.h>

#include <glog/logging.h>

#include <sstream>

namespace kimera {
namespace incremental {

using RoomClusterModeEnum = RoomFinder::Config::ClusterMode;

}  // namespace incremental
}  // namespace kimera

DECLARE_CONFIG_ENUM(kimera::incremental,
                    RoomClusterModeEnum,
                    {RoomClusterModeEnum::SPECTRAL, "SPECTRAL"},
                    {RoomClusterModeEnum::MODULARITY, "MODULARITY"},
                    {RoomClusterModeEnum::NONE, "NONE"})

DECLARE_CONFIG_ENUM(kimera::lcd,
                    DescriptorScoreType,
                    {DescriptorScoreType::COSINE, "COSINE"},
                    {DescriptorScoreType::L1, "L1"})

namespace teaser {

using TeaserInlierSelectionMode = RobustRegistrationSolver::INLIER_SELECTION_MODE;

}  // namespace teaser

DECLARE_CONFIG_ENUM(teaser,
                    TeaserInlierSelectionMode,
                    {TeaserInlierSelectionMode::PMC_EXACT, "PMC_EXACT"},
                    {TeaserInlierSelectionMode::PMC_HEU, "PMC_HEU"},
                    {TeaserInlierSelectionMode::KCORE_HEU, "KCORE_HEU"},
                    {TeaserInlierSelectionMode::NONE, "NONE"})

DECLARE_CONFIG_ENUM(KimeraRPGO,
                    Verbosity,
                    {Verbosity::UPDATE, "UPDATE"},
                    {Verbosity::QUIET, "QUIET"},
                    {Verbosity::VERBOSE, "VERBOSE"})

DECLARE_CONFIG_ENUM(KimeraRPGO, Solver, {Solver::LM, "LM"}, {Solver::GN, "GN"})

namespace config_parser {

template <typename T>
struct ConfigVisitor<std::map<kimera::LayerId, T>> {
  using ConfigMap = std::map<kimera::LayerId, T>;
  using MapType = typename ConfigMap::mapped_type;

  template <typename V, typename std::enable_if<is_parser<V>::value, bool>::type = true>
  static auto visit_config(const V& v, ConfigMap& value) {
    for (const auto& child : v.children()) {
      const auto layer = kimera::KimeraDsgLayers::StringToLayerId(child);
      value[layer] = MapType();
      v.visit(child, value[layer]);
    }
  }

  template <typename V,
            typename std::enable_if<!is_parser<V>::value, bool>::type = true>
  static auto visit_config(const V& v, ConfigMap& value) {
    for (auto& kv_pair : value) {
      const auto layer_str = kimera::KimeraDsgLayers::LayerIdToString(kv_pair.first);
      v.visit(layer_str, kv_pair.second);
    }
  }
};

}  // namespace config_parser

namespace kimera {
namespace incremental {

struct DsgFrontendConfig {
  // TODO(nathan) consider unifying with backend
  bool should_log = true;
  std::string log_path;
  size_t mesh_queue_size = 10;
  size_t min_object_vertices = 20;
  bool prune_mesh_indices = false;
  std::string sensor_frame = "base_link";
  bool enable_lcd = false;
  bool visualize_dsg_lcd = false;
  std::string lcd_visualizer_ns = "/dsg/lcd_visualizer";
  // TODO(nathan) consider moving to lcd config
  double lcd_agent_horizon_s = 1.5;
  double descriptor_creation_horizon_m = 10.0;
  std::string mesh_ns = "";
};

struct DsgBackendConfig {
  bool should_log = true;
  std::string log_path;

  bool visualize_place_factors = true;
  SemanticNodeAttributes::ColorVector building_color{169, 8, 194};  // purple
  SemanticNodeAttributes::Label building_semantic_label = 22u;

  bool enable_rooms = true;
  RoomFinder::Config room_finder;

  struct PgmoConfig {
    bool should_log = true;
    std::string log_path;
    // covariance
    double place_mesh_variance;
    double place_edge_variance;
    // rpgo
    bool gnc_fix_prev_inliers = true;
    KimeraRPGO::Verbosity rpgo_verbosity = KimeraRPGO::Verbosity::UPDATE;
    KimeraRPGO::Solver rpgo_solver = KimeraRPGO::Solver::LM;
  } pgmo;

  // dsg
  bool add_places_to_deformation_graph = true;
  bool optimize_on_lc = true;
  bool enable_node_merging = true;
  bool call_update_periodically = true;
  std::map<LayerId, bool> merge_update_map{{KimeraDsgLayers::OBJECTS, false},
                                           {KimeraDsgLayers::PLACES, true},
                                           {KimeraDsgLayers::ROOMS, false},
                                           {KimeraDsgLayers::BUILDINGS, false}};
  bool merge_update_dynamic = true;
  double places_merge_pos_threshold_m = 0.4;
  double places_merge_distance_tolerance_m = 0.3;
};

template <typename Visitor>
void visit_config(const Visitor& v, DsgFrontendConfig& config) {
  // TODO(nathan) replace with single param (derive should_log from log_path)
  v.visit("should_log", config.should_log);
  v.visit("log_path", config.log_path);
  v.visit("mesh_queue_size", config.mesh_queue_size);
  v.visit("min_object_vertices", config.min_object_vertices);
  v.visit("prune_mesh_indices", config.prune_mesh_indices);
  v.visit("sensor_frame", config.sensor_frame);
  v.visit("enable_lcd", config.enable_lcd);
  v.visit("visualize_dsg_lcd", config.visualize_dsg_lcd);
  v.visit("lcd_visualizer_ns", config.lcd_visualizer_ns);
  v.visit("lcd_agent_horizon_s", config.lcd_agent_horizon_s);
  v.visit("descriptor_creation_horizon_m", config.descriptor_creation_horizon_m);
  v.visit("mesh_ns", config.mesh_ns);
}

struct EnableMapConverter {
  EnableMapConverter() = default;

  std::map<std::string, bool> from(const std::map<LayerId, bool>& other) const;

  std::map<LayerId, bool> from(const std::map<std::string, bool>& other) const;
};

template <typename Visitor>
void visit_config(const Visitor& v, DsgBackendConfig& config) {
  // TODO(nathan) replace with single param (derive should_log from log_path)
  v.visit("should_log", config.should_log);
  v.visit("log_path", config.log_path);
  v.visit("visualize_place_factors", config.visualize_place_factors);
  v.visit("building_color", config.building_color);
  v.visit("building_semantic_label", config.building_semantic_label);
  v.visit("enable_rooms", config.enable_rooms);
  v.visit("room_finder", config.room_finder);

  v.visit("pgmo", config.pgmo);

  auto dsg_handle = v["dsg"];
  dsg_handle.visit("add_places_to_deformation_graph",
                   config.add_places_to_deformation_graph);
  dsg_handle.visit("optimize_on_lc", config.optimize_on_lc);
  dsg_handle.visit("enable_node_merging", config.enable_node_merging);
  dsg_handle.visit("call_update_periodically", config.call_update_periodically);
  //dsg_handle.visit("merge_update_map", config.merge_update_map, EnableMapConverter());
  dsg_handle.visit("merge_update_map", config.merge_update_map);
  dsg_handle.visit("merge_update_dynamic", config.merge_update_dynamic);
  dsg_handle.visit("places_merge_pos_threshold_m", config.places_merge_pos_threshold_m);
  dsg_handle.visit("places_merge_distance_tolerance_m",
                   config.places_merge_distance_tolerance_m);
}

template <typename Visitor>
void visit_config(const Visitor& v, DsgBackendConfig::PgmoConfig& config) {
  // TODO(nathan) replace with single param (derive should_log from log_path)
  v.visit("should_log", config.should_log);
  v.visit("log_path", config.log_path);
  auto covar_handle = v["covariance"];
  covar_handle.visit("place_mesh", config.place_mesh_variance);
  covar_handle.visit("place_edge", config.place_edge_variance);
  auto rpgo_handle = v["rpgo"];
  rpgo_handle.visit("gnc_fix_prev_inliers", config.gnc_fix_prev_inliers);
  rpgo_handle.visit("verbosity", config.rpgo_verbosity);
  rpgo_handle.visit("solver", config.rpgo_solver);
}

template <typename Visitor>
void visit_config(const Visitor& v, RoomFinder::Config& config) {
  v.visit("min_dilation_m", config.min_dilation_m);
  v.visit("max_dilation_m", config.max_dilation_m);
  v.visit("num_steps", config.num_steps);
  v.visit("min_component_size", config.min_component_size);
  v.visit("room_semantic_label", config.room_semantic_label);
  v.visit("max_kmeans_iters", config.max_kmeans_iters);
  v.visit("room_vote_min_overlap", config.room_vote_min_overlap);
  v.visit("min_room_size", config.min_room_size);
  v.visit("use_sparse_eigen_decomp", config.use_sparse_eigen_decomp);
  v.visit("sparse_decomp_tolerance", config.sparse_decomp_tolerance);
  v.visit("max_modularity_iters", config.max_modularity_iters);
  v.visit("modularity_gamma", config.modularity_gamma);
  v.visit("clustering_mode", config.clustering_mode);

  std::string prefix_string;
  v.visit("room_prefix", prefix_string);
  if (config_parser::is_parser<Visitor>()) {
    config.room_prefix = prefix_string[0];
  }
}

}  // namespace incremental

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
void visit_config(const Visitor& v, DsgLcdConfig& config) {
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
}

}  // namespace lcd

struct DsgParamLogger : config_parser::Logger {

  inline void log_missing(const std::string& message) const override {
    LOG(INFO) << message;
  }

};

template <typename Config>
Config load_config(const ros::NodeHandle& nh, const std::string& ns = "") {
  auto logger = std::make_shared<DsgParamLogger>();
  return config_parser::load_from_ros_nh<Config>(nh, ns, logger);
}

}  // namespace kimera

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
DECLARE_CONFIG_OSTREAM_OPERATOR(kimera::incremental, RoomFinder::Config)
DECLARE_CONFIG_OSTREAM_OPERATOR(kimera::incremental, DsgBackendConfig)
DECLARE_CONFIG_OSTREAM_OPERATOR(kimera::lcd, HistogramConfig<double>)
DECLARE_CONFIG_OSTREAM_OPERATOR(kimera::lcd, LayerRegistrationConfig)
DECLARE_CONFIG_OSTREAM_OPERATOR(kimera::lcd, DescriptorMatchConfig)
DECLARE_CONFIG_OSTREAM_OPERATOR(kimera::lcd, DsgLcdConfig)
