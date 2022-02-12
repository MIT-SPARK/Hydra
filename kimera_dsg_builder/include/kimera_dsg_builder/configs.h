#pragma once
#include "kimera_dsg_builder/incremental_room_finder.h"

#include <KimeraRPGO/SolverParams.h>
#include <hydra_utils/config.h>
#include <voxblox_ros/mesh_vis.h>

#include <sstream>

namespace kimera {
namespace incremental {

RoomFinder::Config::ClusterMode getRoomClusterModeFromString(const std::string& mode);

void readRosParam(const ros::NodeHandle& nh,
                  const std::string& name,
                  RoomFinder::Config::ClusterMode& mode);

std::ostream& operator<<(std::ostream& out, RoomFinder::Config::ClusterMode mode);

}  // namespace incremental
}  // namespace kimera

namespace KimeraRPGO {

Verbosity getRpgoVerbosityFromString(const std::string& mode);

void readRosParam(const ros::NodeHandle& nh, const std::string& name, Verbosity& mode);

std::ostream& operator<<(std::ostream& out, Verbosity mode);

Solver getRpgoSolverFromString(const std::string& mode);

void readRosParam(const ros::NodeHandle& nh, const std::string& name, Solver& mode);

std::ostream& operator<<(std::ostream& out, Solver mode);

}  // namespace KimeraRPGO

namespace YAML {

using RoomClusterMode = kimera::incremental::RoomFinder::Config::ClusterMode;

template <>
struct convert<RoomClusterMode> {
  static Node encode(const RoomClusterMode& rhs) {
    std::stringstream ss;
    ss << rhs;
    return Node(ss.str());
  }

  static bool decode(const Node& node, RoomClusterMode& rhs) {
    if (node.IsNull()) {
      return false;
    }
    rhs = kimera::incremental::getRoomClusterModeFromString(node.as<std::string>());
    return true;
  }
};

template <>
struct convert<KimeraRPGO::Verbosity> {
  static Node encode(const KimeraRPGO::Verbosity& rhs) {
    std::stringstream ss;
    ss << rhs;
    return Node(ss.str());
  }

  static bool decode(const Node& node, KimeraRPGO::Verbosity& rhs) {
    if (node.IsNull()) {
      return false;
    }
    rhs = KimeraRPGO::getRpgoVerbosityFromString(node.as<std::string>());
    return true;
  }
};

template <>
struct convert<KimeraRPGO::Solver> {
  static Node encode(const KimeraRPGO::Solver& rhs) {
    std::stringstream ss;
    ss << rhs;
    return Node(ss.str());
  }

  static bool decode(const Node& node, KimeraRPGO::Solver& rhs) {
    if (node.IsNull()) {
      return false;
    }
    rhs = KimeraRPGO::getRpgoSolverFromString(node.as<std::string>());
    return true;
  }
};

}  // namespace YAML

namespace kimera {
namespace incremental {

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
void parse_layer_map(Visitor v, std::map<LayerId, bool>& value) {
  std::map<std::string, bool> raw_values;
  config_parser::visit_config(v, raw_values);

  for (const auto& kv_pair : raw_values) {
    value[KimeraDsgLayers::StringToLayerId(kv_pair.first)] = kv_pair.second;
  }
}

template <typename Visitor>
void display_layer_map(Visitor v, std::map<LayerId, bool>& value) {
  std::map<std::string, bool> raw_values;
  for (const auto& kv_pair : value) {
    raw_values[KimeraDsgLayers::LayerIdToString(kv_pair.first)] = kv_pair.second;
  }

  config_parser::visit_config(v, raw_values);
}

template <typename Visitor,
          typename std::enable_if<config_parser::is_parser<Visitor>::value,
                                  bool>::type = true>
void handle_color(Visitor v, SemanticNodeAttributes::ColorVector& value) {
  std::vector<int> raw_values;
  config_parser::visit_config(v, raw_values);

  if (raw_values.size() < 3) {
    return;
  }

  value << static_cast<uint8_t>(raw_values[0]), static_cast<uint8_t>(raw_values[1]),
      static_cast<uint8_t>(raw_values[2]);
}

template <
    typename Visitor,
    typename std::enable_if<std::negation<config_parser::is_parser<Visitor>>::value,
                            bool>::type = true>
void handle_color(Visitor v, SemanticNodeAttributes::ColorVector& value) {
  std::vector<uint8_t> raw_values(3);
  raw_values[0] = value(0);
  raw_values[1] = value(0);
  raw_values[2] = value(0);
  config_parser::visit_config(v, raw_values);
}

template <typename Visitor>
void visit_config(Visitor& v, DsgBackendConfig& config) {
  // TODO(nathan) replace with single param (derive should_log from log_path)
  config_parser::visit_config(v["should_log"], config.should_log);
  config_parser::visit_config(v["log_path"], config.log_path);
  config_parser::visit_config(v["visualize_place_factors"],
                              config.visualize_place_factors);
  handle_color(v["building_color"], config.building_color);
  config_parser::visit_config(v["building_semantic_label"],
                              config.building_semantic_label);
  config_parser::visit_config(v["enable_rooms"], config.enable_rooms);
  config_parser::visit_config(v["room_finder"], config.enable_rooms);

  config_parser::visit_config(v["pgmo"], config.pgmo);

  auto dsg_handle = v["dsg"];
  config_parser::visit_config(dsg_handle["add_places_to_deformation_graph"],
                              config.add_places_to_deformation_graph);
  config_parser::visit_config(dsg_handle["optimize_on_lc"], config.optimize_on_lc);
  config_parser::visit_config(dsg_handle["enable_node_merging"],
                              config.enable_node_merging);
  config_parser::visit_config(dsg_handle["call_update_periodically"],
                              config.call_update_periodically);
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
void visit_config(Visitor& v, DsgBackendConfig::PgmoConfig& config) {
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
void visit_config(Visitor& v, RoomFinder::Config& config) {
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

DECLARE_CONFIG_OSTREAM_OPERATOR(RoomFinder::Config)
DECLARE_CONFIG_OSTREAM_OPERATOR(DsgBackendConfig)

}  // namespace incremental
}  // namespace kimera
