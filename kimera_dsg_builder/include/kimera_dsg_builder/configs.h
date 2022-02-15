#pragma once
#include "kimera_dsg_builder/config_types.h"
#include "kimera_dsg_builder/dsg_lcd_registration.h"
#include "kimera_dsg_builder/incremental_room_finder.h"

#include <KimeraRPGO/SolverParams.h>
#include <hydra_utils/config.h>
#include <hydra_utils/eigen_config_types.h>
#include <voxblox_ros/mesh_vis.h>

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

namespace kimera {
namespace incremental {

template <typename Visitor>
void parse_layer_map(const Visitor& v, const std::map<LayerId, bool>& value) {
  std::map<std::string, bool> raw_values;
  config_parser::visit_config(v, raw_values);

  for (const auto& kv_pair : raw_values) {
    value[KimeraDsgLayers::StringToLayerId(kv_pair.first)] = kv_pair.second;
  }
}

template <typename Visitor>
void display_layer_map(const Visitor& v, const std::map<LayerId, bool>& value) {
  std::map<std::string, bool> raw_values;
  for (const auto& kv_pair : value) {
    raw_values[KimeraDsgLayers::LayerIdToString(kv_pair.first)] = kv_pair.second;
  }

  config_parser::visit_config(v, raw_values);
}

}  // namespace incremental
}  // namespace kimera

namespace kimera {
namespace incremental {

struct DsgFrontendConfig {
  bool should_log = true;
  std::string log_path = "";
  size_t mesh_queue_size = 10;
  size_t min_object_vertices = 20;
  bool prune_mesh_indices = false;
  std::string sensor_frame = "base_link";
  bool enable_lcd = false;
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
void visit_config(const Visitor& v, const DsgFrontendConfig& config) {
  // TODO(nathan) replace with single param (derive should_log from log_path)
  config_parser::visit_config(v["should_log"], config.should_log);
  config_parser::visit_config(v["log_path"], config.log_path);
  config_parser::visit_config(v["mesh_queue_size"], config.mesh_queue_size);
  config_parser::visit_config(v["min_object_vertices"], config.min_object_vertices);
  config_parser::visit_config(v["prune_mesh_indices"], config.prune_mesh_indices);
  config_parser::visit_config(v["sensor_frame"], config.sensor_frame);
  config_parser::visit_config(v["enable_lcd"], config.enable_lcd);
  config_parser::visit_config(v["lcd_agent_horizon_s"], config.lcd_agent_horizon_s);
  config_parser::visit_config(v["descriptor_creation_horizon_m"],
                              config.descriptor_creation_horizon_m);
  config_parser::visit_config(v["mesh_ns"], config.mesh_ns);
}

template <typename Visitor>
void visit_config(const Visitor& v, const DsgBackendConfig& config) {
  // TODO(nathan) replace with single param (derive should_log from log_path)
  config_parser::visit_config(v["should_log"], config.should_log);
  config_parser::visit_config(v["log_path"], config.log_path);
  config_parser::visit_config(v["visualize_place_factors"],
                              config.visualize_place_factors);
  config_parser::visit_config(v["building_color"], config.building_color);
  config_parser::visit_config(v["building_semantic_label"],
                              config.building_semantic_label);
  config_parser::visit_config(v["enable_rooms"], config.enable_rooms);
  config_parser::visit_config(v["room_finder"], config.enable_rooms);

  config_parser::visit_config(v["pgmo"], config.pgmo);

  // TODO(nathan) handle these namespaces better
  auto dsg_handle = v["dsg"];
  config_parser::visit_config(dsg_handle["add_places_to_deformation_graph"],
                              config.add_places_to_deformation_graph);
  config_parser::visit_config(dsg_handle["optimize_on_lc"], config.optimize_on_lc);
  config_parser::visit_config(dsg_handle["enable_node_merging"],
                              config.enable_node_merging);
  config_parser::visit_config(dsg_handle["call_update_periodically"],
                              config.call_update_periodically);
  // TODO(nathan) replace with sfinae enable_if dispatch
  if (config_parser::is_parser<Visitor>()) {
    parse_layer_map(dsg_handle["merge_update_map"], config.merge_update_map);
  } else {
    display_layer_map(dsg_handle["merge_update_map"], config.merge_update_map);
  }
  config_parser::visit_config(dsg_handle["merge_update_dynamic"],
                              config.merge_update_dynamic);
  config_parser::visit_config(dsg_handle["places_merge_pos_threshold_m"],
                              config.places_merge_pos_threshold_m);
  config_parser::visit_config(dsg_handle["places_merge_distance_tolerance_m"],
                              config.places_merge_distance_tolerance_m);
}

template <typename Visitor>
void visit_config(const Visitor& v, const DsgBackendConfig::PgmoConfig& config) {
  // TODO(nathan) replace with single param (derive should_log from log_path)
  config_parser::visit_config(v["should_log"], config.should_log);
  config_parser::visit_config(v["log_path"], config.log_path);
  auto covar_handle = v["covariance"];
  config_parser::visit_config(covar_handle["place_mesh"], config.place_mesh_variance);
  config_parser::visit_config(covar_handle["place_edge"], config.place_edge_variance);
  auto rpgo_handle = v["rpgo"];
  config_parser::visit_config(rpgo_handle["gnc_fix_prev_inliers"],
                              config.gnc_fix_prev_inliers);
  config_parser::visit_config(rpgo_handle["verbosity"], config.rpgo_verbosity);
  config_parser::visit_config(rpgo_handle["solver"], config.rpgo_solver);
}

template <typename Visitor>
void visit_config(const Visitor& v, const RoomFinder::Config& config) {
  config_parser::visit_config(v["min_dilation_m"], config.min_dilation_m);
  config_parser::visit_config(v["max_dilation_m"], config.max_dilation_m);
  config_parser::visit_config(v["num_steps"], config.num_steps);
  config_parser::visit_config(v["min_component_size"], config.min_component_size);
  config_parser::visit_config(v["room_prefix"], config.room_prefix);
  config_parser::visit_config(v["max_kmeans_iters"], config.max_kmeans_iters);
  config_parser::visit_config(v["room_vote_min_overlap"], config.room_vote_min_overlap);
  config_parser::visit_config(v["min_room_size"], config.min_room_size);
  config_parser::visit_config(v["use_sparse_eigen_decomp"],
                              config.use_sparse_eigen_decomp);
  config_parser::visit_config(v["sparse_decomp_tolerance"],
                              config.sparse_decomp_tolerance);
  config_parser::visit_config(v["max_modularity_iters"], config.max_modularity_iters);
  config_parser::visit_config(v["modularity_gamma"], config.modularity_gamma);
  config_parser::visit_config(v["clustering_mode"], config.clustering_mode);
}

}  // namespace incremental
}  // namespace kimera

namespace teaser {

template <typename Visitor>
void visit_config(const Visitor& v,
                  const teaser::RobustRegistrationSolver::Params& config) {
  config_parser::visit_config(v["estimate_scaling"], config.estimate_scaling);
  config_parser::visit_config(v["cbar2"], config.cbar2);
  config_parser::visit_config(v["rotation_gnc_factor"], config.rotation_gnc_factor);
  config_parser::visit_config(v["rotation_max_iterations"],
                              config.rotation_max_iterations);
  config_parser::visit_config(v["kcore_heuristic_threshold"],
                              config.kcore_heuristic_threshold);
  config_parser::visit_config(v["inlier_selection_mode"], config.inlier_selection_mode);
  config_parser::visit_config(v["max_clique_time_limit"], config.max_clique_time_limit);
}

}  // namespace teaser

DECLARE_CONFIG_OSTREAM_OPERATOR(teaser, RobustRegistrationSolver::Params)
DECLARE_CONFIG_OSTREAM_OPERATOR(kimera::incremental, RoomFinder::Config)
DECLARE_CONFIG_OSTREAM_OPERATOR(kimera::incremental, DsgBackendConfig)
