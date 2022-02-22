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
void visit_config(const Visitor& v, MeshIntegratorConfig& config) {
  v.visit("use_color", config.use_color);
  v.visit("min_weight", config.min_weight);
  v.visit("integrator_threads", config.integrator_threads);
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
void visit_config(const Visitor& v, VoronoiCheckConfig& config) {
  v.visit("mode", config.mode);
  v.visit("min_distance_m", config.min_distance_m);
  v.visit("parent_l1_separation", config.parent_l1_separation);
  v.visit("parent_cos_angle_separation", config.parent_cos_angle_separation);
}

template <typename Visitor>
void visit_config(const Visitor& v, GraphExtractorConfig& config) {
  v.visit("min_extra_basis", config.min_extra_basis);
  v.visit("min_vertex_basis", config.min_vertex_basis);
  v.visit("merge_new_nodes", config.merge_new_nodes);
  v.visit("node_merge_distance_m", config.node_merge_distance_m);
  v.visit("edge_splitting_merge_nodes", config.edge_splitting_merge_nodes);
  v.visit("max_edge_split_iterations", config.max_edge_split_iterations);
  v.visit("max_edge_deviation", config.max_edge_deviation);
  v.visit("add_freespace_edges", config.add_freespace_edges);
  v.visit("freespace_active_neighborhood_hops", config.freespace_active_neighborhood_hops);
  v.visit("freespace_edge_num_neighbors", config.freespace_edge_num_neighbors);
  v.visit("freespace_edge_min_clearance_m", config.freespace_edge_min_clearance_m);
  v.visit("add_component_connection_edges", config.add_component_connection_edges);
  v.visit("connected_component_window", config.connected_component_window);
  v.visit("connected_component_hops", config.connected_component_hops);
  v.visit("component_nearest_neighbors", config.component_nearest_neighbors);
  v.visit("component_max_edge_length_m", config.component_max_edge_length_m);
  v.visit("component_min_clearance_m", config.component_min_clearance_m);
  v.visit("remove_isolated_nodes", config.remove_isolated_nodes);
}

template <typename Visitor>
void visit_config(const Visitor& v, GvdIntegratorConfig& config) {
  v.visit("max_distance_m", config.max_distance_m);
  v.visit("min_distance_m", config.min_distance_m);
  v.visit("min_diff_m", config.min_diff_m);
  v.visit("min_weight", config.min_weight);
  v.visit("num_buckets", config.num_buckets);
  v.visit("multi_queue", config.multi_queue);
  v.visit("positive_distance_only", config.positive_distance_only);
  v.visit("parent_derived_distance", config.parent_derived_distance);
  v.visit("min_basis_for_extraction", config.min_basis_for_extraction);
  v.visit("voronoi_config", config.voronoi_config);
  v.visit("mesh_integrator_config", config.mesh_integrator_config);
  v.visit("graph_extractor", config.graph_extractor_config);
  v.visit("extract_graph", config.extract_graph);
  v.visit("mesh_only", config.mesh_only);
}

template <typename Visitor>
void visit_config(const Visitor& v, TopologyServerConfig& config) {
  v.visit("update_period_s", config.update_period_s);
  v.visit("show_stats", config.show_stats);
  v.visit("dense_representation_radius_m", config.dense_representation_radius_m);
  v.visit("publish_archived", config.publish_archived);
  v.visit("mesh_color_mode", config.mesh_color_mode);
  v.visit("world_frame", config.world_frame);
}

}  // namespace topology
}  // namespace kimera

DECLARE_CONFIG_OSTREAM_OPERATOR(voxblox, MeshIntegratorConfig)
DECLARE_CONFIG_OSTREAM_OPERATOR(kimera::topology, TopologyServerConfig)
DECLARE_CONFIG_OSTREAM_OPERATOR(kimera::topology, VoronoiCheckConfig)
DECLARE_CONFIG_OSTREAM_OPERATOR(kimera::topology, GraphExtractorConfig)
DECLARE_CONFIG_OSTREAM_OPERATOR(kimera::topology, GvdIntegratorConfig)
