#pragma once
#include "kimera_topology/gvd_integrator.h"

#include <hydra_utils/config.h>
#include <voxblox_ros/mesh_vis.h>
#include <sstream>

DECLARE_CONFIG_ENUM(voxblox,
                    ColorMode,
                    {ColorMode::kColor, "color"},
                    {ColorMode::kHeight, "height"},
                    {ColorMode::kNormals, "normals"},
                    {ColorMode::kGray, "gray"},
                    {ColorMode::kLambert, "lambert"},
                    {ColorMode::kLambertColor, "lambert_color"});

DECLARE_CONFIG_ENUM(kimera::topology,
                    ParentUniquenessMode,
                    {ParentUniquenessMode::ANGLE, "ANGLE"},
                    {ParentUniquenessMode::L1_DISTANCE, "L1_DISTANCE"},
                    {ParentUniquenessMode::L1_THEN_ANGLE, "L1_THEN_ANGLE"});

namespace voxblox {

template <typename Visitor>
void visit_config(const Visitor& v, const MeshIntegratorConfig& config) {
  config_parser::visit_config(v["use_color"], config.use_color);
  config_parser::visit_config(v["min_weight"], config.min_weight);
  config_parser::visit_config(v["integrator_threads"], config.integrator_threads);
}

}  // namespace voxblox



namespace kimera {
namespace topology {

struct TopologyServerConfig {
  double update_period_s = 1.0;
  bool show_stats = true;
  bool clear_distant_blocks = true;
  double dense_representation_radius_m = 5.0;
  bool publish_archived = true;

  voxblox::ColorMode mesh_color_mode = voxblox::ColorMode::kLambertColor;
  std::string world_frame = "world";
};

template <typename Visitor>
void visit_config(const Visitor& v, const VoronoiCheckConfig& config) {
  config_parser::visit_config(v["mode"], config.mode);
  config_parser::visit_config(v["min_distance_m"], config.min_distance_m);
  config_parser::visit_config(v["parent_l1_separation"], config.parent_l1_separation);
  config_parser::visit_config(v["parent_cos_angle_separation"],
                              config.parent_cos_angle_separation);
}

template <typename Visitor>
void visit_config(const Visitor& v, const GraphExtractorConfig& config) {
  config_parser::visit_config(v["min_extra_basis"], config.min_extra_basis);
  config_parser::visit_config(v["min_vertex_basis"], config.min_vertex_basis);
  config_parser::visit_config(v["merge_new_nodes"], config.merge_new_nodes);
  config_parser::visit_config(v["node_merge_distance_m"], config.node_merge_distance_m);
  config_parser::visit_config(v["edge_splitting_merge_nodes"],
                              config.edge_splitting_merge_nodes);
  config_parser::visit_config(v["max_edge_split_iterations"],
                              config.max_edge_split_iterations);
  config_parser::visit_config(v["max_edge_deviation"], config.max_edge_deviation);
  config_parser::visit_config(v["add_freespace_edges"], config.add_freespace_edges);
  config_parser::visit_config(v["freespace_active_neighborhood_hops"],
                              config.freespace_active_neighborhood_hops);
  config_parser::visit_config(v["freespace_edge_num_neighbors"],
                              config.freespace_edge_num_neighbors);
  config_parser::visit_config(v["freespace_edge_min_clearance_m"],
                              config.freespace_edge_min_clearance_m);
  config_parser::visit_config(v["add_component_connection_edges"],
                              config.add_component_connection_edges);
  config_parser::visit_config(v["connected_component_window"],
                              config.connected_component_window);
  config_parser::visit_config(v["connected_component_hops"],
                              config.connected_component_hops);
  config_parser::visit_config(v["component_nearest_neighbors"],
                              config.component_nearest_neighbors);
  config_parser::visit_config(v["component_max_edge_length_m"],
                              config.component_max_edge_length_m);
  config_parser::visit_config(v["component_min_clearance_m"],
                              config.component_min_clearance_m);
  config_parser::visit_config(v["remove_isolated_nodes"], config.remove_isolated_nodes);
}

template <typename Visitor>
void visit_config(const Visitor& v, const GvdIntegratorConfig& config) {
  config_parser::visit_config(v["max_distance_m"], config.max_distance_m);
  config_parser::visit_config(v["min_distance_m"], config.min_distance_m);
  config_parser::visit_config(v["min_diff_m"], config.min_diff_m);
  config_parser::visit_config(v["min_weight"], config.min_weight);
  config_parser::visit_config(v["num_buckets"], config.num_buckets);
  config_parser::visit_config(v["multi_queue"], config.multi_queue);
  config_parser::visit_config(v["positive_distance_only"],
                              config.positive_distance_only);
  config_parser::visit_config(v["parent_derived_distance"],
                              config.parent_derived_distance);
  config_parser::visit_config(v["min_basis_for_extraction"],
                              config.min_basis_for_extraction);
  config_parser::visit_config(v["voronoi_config"], config.voronoi_config);
  config_parser::visit_config(v["mesh_integrator_config"],
                              config.mesh_integrator_config);
  config_parser::visit_config(v["graph_extractor_config"],
                              config.graph_extractor_config);
  config_parser::visit_config(v["extract_graph"], config.extract_graph);
  config_parser::visit_config(v["mesh_only"], config.mesh_only);
}

template <typename Visitor>
void visit_config(const Visitor& v, const TopologyServerConfig& config) {
  config_parser::visit_config(v["update_period_s"], config.update_period_s);
  config_parser::visit_config(v["show_stats"], config.show_stats);
  config_parser::visit_config(v["dense_representation_radius_m"],
                              config.dense_representation_radius_m);
  config_parser::visit_config(v["publish_archived"], config.publish_archived);
  config_parser::visit_config(v["mesh_color_mode"], config.mesh_color_mode);
  config_parser::visit_config(v["world_frame"], config.world_frame);
}


}  // namespace topology
}  // namespace kimera

DECLARE_CONFIG_OSTREAM_OPERATOR(voxblox, MeshIntegratorConfig)
DECLARE_CONFIG_OSTREAM_OPERATOR(kimera::topology, TopologyServerConfig)
DECLARE_CONFIG_OSTREAM_OPERATOR(kimera::topology, VoronoiCheckConfig)
DECLARE_CONFIG_OSTREAM_OPERATOR(kimera::topology, GraphExtractorConfig)
DECLARE_CONFIG_OSTREAM_OPERATOR(kimera::topology, GvdIntegratorConfig)
