#include "kimera_topology/config_parser.h"

#include <type_traits>

namespace kimera {
namespace topology {

template <typename T>
struct is_base_ros_param : std::false_type {};

template <>
struct is_base_ros_param<std::string> : std::true_type {};

template <>
struct is_base_ros_param<double> : std::true_type {};

template <>
struct is_base_ros_param<float> : std::true_type {};

template <>
struct is_base_ros_param<int> : std::true_type {};

template <>
struct is_base_ros_param<bool> : std::true_type {};

template <typename T, std::enable_if_t<is_base_ros_param<T>::value, bool> = true>
bool parseParam(const ros::NodeHandle& nh, const std::string& name, T& value) {
  return nh.getParam(name, value);
}

template <typename T, std::enable_if_t<is_base_ros_param<T>::value, bool> = true>
bool parseParam(const ros::NodeHandle& nh,
                const std::string& name,
                std::vector<T>& value) {
  return nh.getParam(name, value);
}

template <typename T,
          typename Bounds,
          std::enable_if_t<std::is_integral<T>::value, bool> = true>
bool parseParam(const ros::NodeHandle& nh,
                const std::string& name,
                T& value,
                const Bounds lo_value,
                const Bounds hi_value) {
  int placeholder = 0;  // putting default value into int is overflow prone
  bool had_param = nh.getParam(name, placeholder);
  if (!had_param) {
    return false;
  }

  if (sizeof(T) < sizeof(int)) {
    // avoid overflow on uint16_t and smaller
    int old_placeholder = placeholder;
    placeholder =
        std::clamp(placeholder, static_cast<int>(lo_value), static_cast<int>(hi_value));
    if (placeholder != old_placeholder) {
      ROS_WARN_STREAM("Parameter "
                      << nh.resolveName(name) << " had a value " << old_placeholder
                      << " which was outside the bounds of [" << lo_value << ", "
                      << hi_value << "]");
    }
  }

  value = static_cast<T>(placeholder);

  if (sizeof(T) >= sizeof(int)) {
    // avoid clamping issues with values larger than int
    T old_value = value;
    value = std::clamp(value, static_cast<T>(lo_value), static_cast<T>(hi_value));
    if (value != old_value) {
      ROS_WARN_STREAM("Parameter " << nh.resolveName(name) << " had a value "
                                   << old_value << " which was outside the bounds of ["
                                   << lo_value << ", " << hi_value << "]");
    }
  }
  return had_param;
}

template <typename T,
          std::enable_if_t<!is_base_ros_param<T>::value && std::is_integral<T>::value,
                           bool> = true>
bool parseParam(const ros::NodeHandle& nh, const std::string& name, T& value) {
  return parseParam(
      nh, name, value, std::numeric_limits<T>::min(), std::numeric_limits<T>::max());
}

bool parseParam(const ros::NodeHandle& nh,
                const std::string& name,
                voxblox::ColorMode& value) {
  std::string placeholder;
  bool had_param = nh.getParam(name, placeholder);
  if (!had_param) {
    return false;
  }

  value = voxblox::getColorModeFromString(placeholder);
  return true;
}

template <typename T>
void outputParamDefault(std::ostream& out, const T& value) {
  out << value;
}

template <>
void outputParamDefault(std::ostream& out, const uint8_t& value) {
  out << static_cast<int>(value);
}

template <>
void outputParamDefault(std::ostream& out, const ParentUniquenessMode& value) {
  switch (value) {
    case ParentUniquenessMode::ANGLE:
      out << "ANGLE";
      break;
    case ParentUniquenessMode::L1_DISTANCE:
      out << "L1_DISTANCE";
      break;
    case ParentUniquenessMode::L1_THEN_ANGLE:
      out << "L1_THEN_ANGLE";
      break;
  }
  out << "(" << static_cast<int>(value) << ")";
}

#define FILL_AND_RETURN_IF_MATCH(name, value, dest) \
  if (#name == value) {                             \
    dest = ParentUniquenessMode::name;              \
    return true;                                    \
  }                                                 \
  static_assert(true, "")

bool parseParam(const ros::NodeHandle& nh,
                const std::string& name,
                ParentUniquenessMode& value) {
  std::string placeholder;
  bool had_param = nh.getParam(name, placeholder);
  if (!had_param) {
    return false;
  }

  std::transform(
      placeholder.begin(), placeholder.end(), placeholder.begin(), [](unsigned char c) {
        return std::toupper(c);
      });

  FILL_AND_RETURN_IF_MATCH(ANGLE, placeholder, value);
  FILL_AND_RETURN_IF_MATCH(L1_DISTANCE, placeholder, value);
  FILL_AND_RETURN_IF_MATCH(L1_THEN_ANGLE, placeholder, value);

  std::stringstream ss;
  ss << "Invalid ParentUniquenessMode: " << placeholder << ". Defaulting to ";
  outputParamDefault(ss, value);
  ROS_WARN_STREAM(ss.str());
  return true;
}

#undef FILL_PARENT_ENUM_IF_MATCH

#define READ_PARAM(nh, config, name, bounds...)                 \
  if (!parseParam(nh, #name, config.name, ##bounds)) {          \
    std::stringstream ss;                                       \
    ss << nh.resolveName(#name) + " not found! Defaulting to "; \
    outputParamDefault(ss, config.name);                        \
    ROS_DEBUG_STREAM(ss.str());                                 \
  }                                                             \
  static_assert(true, "")

void fillGraphExtractorConfig(const ros::NodeHandle& nh, GraphExtractorConfig& config) {
  READ_PARAM(nh, config, min_extra_basis, 0, 26);
  READ_PARAM(nh, config, min_vertex_basis, 0, 26);
  READ_PARAM(nh, config, merge_new_nodes);
  READ_PARAM(nh, config, node_merge_distance_m);
  READ_PARAM(nh, config, edge_splitting_merge_nodes);
  READ_PARAM(nh, config, max_edge_split_iterations);
  READ_PARAM(nh, config, max_edge_deviation);
  READ_PARAM(nh, config, add_freespace_edges);
  READ_PARAM(nh, config, freespace_active_neighborhood_hops);
  READ_PARAM(nh, config, freespace_edge_num_neighbors);
  READ_PARAM(nh, config, freespace_edge_min_clearance_m);
  READ_PARAM(nh, config, add_component_connection_edges);
  READ_PARAM(nh, config, connected_component_window);
  READ_PARAM(nh, config, connected_component_hops);
  READ_PARAM(nh, config, component_nodes_to_check);
  READ_PARAM(nh, config, component_nearest_neighbors);
  READ_PARAM(nh, config, component_max_edge_length_m);
  READ_PARAM(nh, config, component_min_clearance_m);
  READ_PARAM(nh, config, remove_isolated_nodes);
}

void fillGvdIntegratorConfigFields(const ros::NodeHandle& nh,
                                   GvdIntegratorConfig& config) {
  READ_PARAM(nh, config, max_distance_m);
  READ_PARAM(nh, config, min_distance_m);
  READ_PARAM(nh, config, min_diff_m);
  READ_PARAM(nh, config, min_weight);
  READ_PARAM(nh, config, num_buckets);
  READ_PARAM(nh, config, multi_queue);
  READ_PARAM(nh, config, positive_distance_only);
  READ_PARAM(nh, config, parent_derived_distance);
  READ_PARAM(nh, config, min_basis_for_extraction, 0, 26);
  READ_PARAM(nh, config, extract_graph);
  READ_PARAM(nh, config, mesh_only);
};

void fillVoronoiCheckConfig(const ros::NodeHandle& nh, VoronoiCheckConfig& config) {
  READ_PARAM(nh, config, mode);
  READ_PARAM(nh, config, min_distance_m);
  READ_PARAM(nh, config, parent_l1_separation);
  READ_PARAM(nh, config, parent_cos_angle_separation);
}

void fillGvdIntegratorConfig(const ros::NodeHandle& nh, GvdIntegratorConfig& config) {
  fillGvdIntegratorConfigFields(nh, config);
  fillVoronoiCheckConfig(ros::NodeHandle(nh, "voronoi_check"), config.voronoi_config);
  fillGraphExtractorConfig(ros::NodeHandle(nh, "graph_extractor"),
                           config.graph_extractor_config);
  // TODO(nathan) mesh integration
}

void fillTopologyServerConfig(const ros::NodeHandle& nh, TopologyServerConfig& config) {
  READ_PARAM(nh, config, update_period_s);
  READ_PARAM(nh, config, show_stats);
  READ_PARAM(nh, config, clear_distant_blocks);
  READ_PARAM(nh, config, dense_representation_radius_m);
  READ_PARAM(nh, config, publish_archived);
  READ_PARAM(nh, config, mesh_color_mode);
  READ_PARAM(nh, config, world_frame);
}

void fillTopologyServerVizConfig(const ros::NodeHandle& nh,
                                 TopologyVisualizerConfig& config) {
  READ_PARAM(nh, config, world_frame);
  READ_PARAM(nh, config, topology_marker_ns);
  READ_PARAM(nh, config, show_block_outlines);
  READ_PARAM(nh, config, use_gvd_block_outlines);
  READ_PARAM(nh, config, outline_scale);
}

GvdVisualizerConfig getGvdVisualizerConfig(const std::string& ns) {
  ros::NodeHandle nh(ns);
  GvdVisualizerConfig config;
  READ_PARAM(nh, config, visualization_type, 0, 2);
  READ_PARAM(nh, config, color_nearest_vertices);
  READ_PARAM(nh, config, gvd_alpha);
  READ_PARAM(nh, config, gvd_min_distance);
  READ_PARAM(nh, config, gvd_max_distance);
  READ_PARAM(nh, config, basis_threshold);
  READ_PARAM(nh, config, min_num_basis);
  READ_PARAM(nh, config, max_num_basis);
  READ_PARAM(nh, config, gvd_mode, 0, 2);
  READ_PARAM(nh, config, esdf_alpha);
  READ_PARAM(nh, config, slice_height);
  READ_PARAM(nh, config, esdf_min_distance);
  READ_PARAM(nh, config, esdf_max_distance);
  return config;
}

#undef READ_PARAM

#define SHOW_PARAM(config, param) " - " #param ": " << config.param << std::endl

#define SHOW_NS_PARAM(config, ns, param) \
  " - " << ns << "/" #param ": " << config.param << std::endl

#define SHOW_BYTE_PARAM(config, param) \
  " - " #param ": " << static_cast<int>(config.param) << std::endl

#define SHOW_NS_BYTE_PARAM(config, ns, param) \
  " - " << ns << "/" #param ": " << static_cast<int>(config.param) << std::endl

#define SHOW_BOOL_PARAM(config, param) \
  " - " #param ": " << (config.param ? "yes" : "no") << std::endl

#define SHOW_NS_BOOL_PARAM(config, ns, param) \
  " - " << ns << "/" #param ": " << (config.param ? "yes" : "no") << std::endl

void showConfig(const GvdIntegratorConfig& config) {
  LOG(INFO)
      << "GVD Config: " << std::endl
      << SHOW_PARAM(config, max_distance_m) << SHOW_PARAM(config, min_distance_m)
      << SHOW_PARAM(config, min_diff_m) << SHOW_PARAM(config, min_weight)
      << SHOW_PARAM(config, num_buckets) << SHOW_BOOL_PARAM(config, multi_queue)
      << SHOW_BOOL_PARAM(config, positive_distance_only)
      << SHOW_BOOL_PARAM(config, parent_derived_distance)
      << SHOW_BOOL_PARAM(config, extract_graph) << SHOW_BOOL_PARAM(config, mesh_only)
      << SHOW_BYTE_PARAM(config, min_basis_for_extraction)
      << SHOW_NS_PARAM(config.voronoi_config, "voronoi", min_distance_m)
      << SHOW_NS_PARAM(config.voronoi_config, "voronoi", parent_l1_separation)
      << SHOW_NS_PARAM(config.voronoi_config, "voronoi", parent_cos_angle_separation)
      << SHOW_NS_BYTE_PARAM(config.graph_extractor_config, "graph", min_extra_basis)
      << SHOW_NS_BYTE_PARAM(config.graph_extractor_config, "graph", min_vertex_basis)
      << SHOW_NS_BOOL_PARAM(config.graph_extractor_config, "graph", merge_new_nodes)
      << SHOW_NS_PARAM(config.graph_extractor_config, "graph", node_merge_distance_m)
      << SHOW_NS_BOOL_PARAM(
             config.graph_extractor_config, "graph", edge_splitting_merge_nodes)
      << SHOW_NS_PARAM(
             config.graph_extractor_config, "graph", max_edge_split_iterations)
      << SHOW_NS_PARAM(config.graph_extractor_config, "graph", max_edge_deviation)
      << SHOW_NS_BOOL_PARAM(config.graph_extractor_config, "graph", add_freespace_edges)
      << SHOW_NS_PARAM(
             config.graph_extractor_config, "graph", freespace_active_neighborhood_hops)
      << SHOW_NS_PARAM(
             config.graph_extractor_config, "graph", freespace_edge_num_neighbors)
      << SHOW_NS_PARAM(
             config.graph_extractor_config, "graph", freespace_edge_min_clearance_m)
      << SHOW_NS_BOOL_PARAM(
             config.graph_extractor_config, "graph", add_component_connection_edges)
      << SHOW_NS_PARAM(
             config.graph_extractor_config, "graph", connected_component_window)
      << SHOW_NS_PARAM(config.graph_extractor_config, "graph", connected_component_hops)
      << SHOW_NS_PARAM(config.graph_extractor_config, "graph", component_nodes_to_check)
      << SHOW_NS_PARAM(
             config.graph_extractor_config, "graph", component_nearest_neighbors)
      << SHOW_NS_PARAM(
             config.graph_extractor_config, "graph", component_max_edge_length_m)
      << SHOW_NS_PARAM(
             config.graph_extractor_config, "graph", component_min_clearance_m)
      << SHOW_NS_BOOL_PARAM(
             config.graph_extractor_config, "graph", remove_isolated_nodes);
}

#undef SHOW_PARAM
#undef SHOW_NS_PARAM
#undef SHOW_BOOL_PARAM
#undef SHOW_NS_BOOL_PARAM
#undef SHOW_BYTE_PARAM
#undef SHOW_NS_BYTE_PARAM

}  // namespace topology
}  // namespace kimera
