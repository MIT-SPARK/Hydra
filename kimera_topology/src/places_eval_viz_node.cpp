#include <kimera_dsg/dynamic_scene_graph.h>
#include <kimera_dsg_visualizer/colormap_utils.h>
#include <kimera_topology/GvdVisualizerConfig.h>
#include <nav_msgs/LoadMap.h>
#include <voxblox/io/layer_io.h>
#include <yaml-cpp/yaml.h>
#include "kimera_topology/config_parser.h"
#include "kimera_topology/gvd_integrator.h"
#include "kimera_topology/topology_server_visualizer.h"

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <std_srvs/Empty.h>

DEFINE_int32(voxels_per_side, 16, "voxels per side");
DEFINE_double(voxel_size, 0.1, "voxel size");
DEFINE_double(max_distance_m, 4.5, "max distance");
DEFINE_string(tsdf_file, "", "tsdf file to read");
DEFINE_string(dsg_file, "", "dsg file to read");
DEFINE_string(gvd_config, "", "gvd integrator config");
DEFINE_bool(suppress_data, true, "collapse lists to mean");
DEFINE_double(offset, 2.0, "places offset");
DEFINE_double(max_error, 2.0, "maximum error for colormap");
DEFINE_double(min_cmap_distance, 0.0, "min distance for colormap");
DEFINE_double(max_cmap_distance, 2.0, "max distance for colormap");
DEFINE_int32(basis_threshold, 2, "basis threshold");

using namespace kimera;
using namespace kimera::topology;
using visualization_msgs::Marker;
using visualization_msgs::MarkerArray;
using voxblox::Layer;
using voxblox::TsdfVoxel;

#define READ_PARAM_AUTO(config, node, name)                                        \
  if (node[#name]) {                                                               \
    config.name = node[#name].as<decltype(config.name)>();                         \
  } else {                                                                         \
    LOG(INFO) << "Missing param: " << #name << ". Defaulting to: " << config.name; \
  }                                                                                \
  static_assert(true, "")

#define READ_PARAM(config, node, name, type)                                       \
  if (node[#name]) {                                                               \
    config.name = node[#name].as<type>();                                          \
  } else {                                                                         \
    LOG(INFO) << "Missing param: " << #name << ". Defaulting to: " << config.name; \
  }                                                                                \
  static_assert(true, "")

VoronoiCheckConfig readVoronoiConfig(const YAML::Node& config_root) {
  VoronoiCheckConfig config;
  READ_PARAM_AUTO(config, config_root, min_distance_m);
  READ_PARAM_AUTO(config, config_root, parent_l1_separation);
  READ_PARAM_AUTO(config, config_root, parent_cos_angle_separation);
  return config;
}

GraphExtractorConfig readGraphConfig(const YAML::Node& config_root) {
  GraphExtractorConfig config;
  READ_PARAM(config, config_root, min_extra_basis, uint32_t);
  READ_PARAM(config, config_root, min_vertex_basis, uint32_t);
  READ_PARAM_AUTO(config, config_root, merge_new_nodes);
  READ_PARAM_AUTO(config, config_root, node_merge_distance_m);
  READ_PARAM_AUTO(config, config_root, edge_splitting_merge_nodes);
  READ_PARAM_AUTO(config, config_root, max_edge_split_iterations);
  READ_PARAM_AUTO(config, config_root, max_edge_deviation);
  READ_PARAM_AUTO(config, config_root, add_freespace_edges);
  READ_PARAM_AUTO(config, config_root, freespace_active_neighborhood_hops);
  READ_PARAM_AUTO(config, config_root, freespace_edge_num_neighbors);
  READ_PARAM_AUTO(config, config_root, freespace_edge_min_clearance_m);
  READ_PARAM_AUTO(config, config_root, add_component_connection_edges);
  READ_PARAM_AUTO(config, config_root, connected_component_window);
  READ_PARAM_AUTO(config, config_root, connected_component_hops);
  READ_PARAM_AUTO(config, config_root, component_nodes_to_check);
  READ_PARAM_AUTO(config, config_root, component_nearest_neighbors);
  READ_PARAM_AUTO(config, config_root, component_max_edge_length_m);
  READ_PARAM_AUTO(config, config_root, component_min_clearance_m);
  READ_PARAM_AUTO(config, config_root, remove_isolated_nodes);
  return config;
}

GvdIntegratorConfig readGvdConfig(const std::string& filename) {
  YAML::Node config_root = YAML::LoadFile(filename);

  GvdIntegratorConfig config;
  config.max_distance_m = FLAGS_max_distance_m;
  READ_PARAM_AUTO(config, config_root, min_distance_m);
  READ_PARAM_AUTO(config, config_root, min_diff_m);
  READ_PARAM_AUTO(config, config_root, min_weight);
  READ_PARAM_AUTO(config, config_root, num_buckets);
  READ_PARAM_AUTO(config, config_root, multi_queue);
  READ_PARAM_AUTO(config, config_root, positive_distance_only);
  READ_PARAM_AUTO(config, config_root, parent_derived_distance);
  READ_PARAM(config, config_root, min_basis_for_extraction, uint32_t);

  if (config_root["voronoi_check"]) {
    config.voronoi_config = readVoronoiConfig(config_root["voronoi_check"]);
  }

  config.extract_graph = true;
  if (config_root["graph_extractor"]) {
    config.graph_extractor_config = readGraphConfig(config_root["graph_extractor"]);
  }
  config.mesh_only = false;

  return config;
}

ColormapConfig readColormapConfig(const std::string& filename) {
  YAML::Node config_root = YAML::LoadFile(filename);

  ColormapConfig config;
  READ_PARAM_AUTO(config, config_root, min_hue);
  READ_PARAM_AUTO(config, config_root, max_hue);
  READ_PARAM_AUTO(config, config_root, min_saturation);
  READ_PARAM_AUTO(config, config_root, max_saturation);
  READ_PARAM_AUTO(config, config_root, min_luminance);
  READ_PARAM_AUTO(config, config_root, max_luminance);
  return config;
}

VisualizerConfig readVisualizerConfig(const std::string& filename) {
  YAML::Node config_root = YAML::LoadFile(filename);

  VisualizerConfig config;
  READ_PARAM_AUTO(config, config_root, layer_z_step);
  READ_PARAM_AUTO(config, config_root, mesh_edge_break_ratio);
  READ_PARAM_AUTO(config, config_root, mesh_layer_offset);
  READ_PARAM_AUTO(config, config_root, collapse_layers);
  READ_PARAM_AUTO(config, config_root, color_places_by_distance);
  config.places_colormap_min_distance = FLAGS_min_cmap_distance;
  config.places_colormap_max_distance = FLAGS_max_cmap_distance;
  return config;
}

LayerConfig readLayerConfig(const std::string& filename) {
  YAML::Node config_root = YAML::LoadFile(filename);

  LayerConfig config;
  READ_PARAM_AUTO(config, config_root, z_offset_scale);
  READ_PARAM_AUTO(config, config_root, visualize);
  READ_PARAM_AUTO(config, config_root, marker_scale);
  READ_PARAM_AUTO(config, config_root, marker_alpha);
  READ_PARAM_AUTO(config, config_root, use_sphere_marker);
  READ_PARAM_AUTO(config, config_root, use_label);
  READ_PARAM_AUTO(config, config_root, use_label);
  READ_PARAM_AUTO(config, config_root, label_height);
  READ_PARAM_AUTO(config, config_root, label_scale);
  READ_PARAM_AUTO(config, config_root, use_bounding_box);
  READ_PARAM_AUTO(config, config_root, bounding_box_alpha);
  READ_PARAM_AUTO(config, config_root, use_edge_source);
  READ_PARAM_AUTO(config, config_root, interlayer_edge_scale);
  READ_PARAM_AUTO(config, config_root, interlayer_edge_alpha);
  READ_PARAM_AUTO(config, config_root, interlayer_edge_use_color);
  READ_PARAM_AUTO(config, config_root, interlayer_edge_insertion_skip);
  READ_PARAM_AUTO(config, config_root, intralayer_edge_scale);
  READ_PARAM_AUTO(config, config_root, intralayer_edge_alpha);
  READ_PARAM_AUTO(config, config_root, intralayer_edge_insertion_skip);
  return config;
}

struct ToggleFunctor {
  bool show_error = true;

  bool call(std_srvs::Empty::Request&, std_srvs::Empty::Response&) {
    show_error = !show_error;
    return true;
  }
};

struct SliceLevelFunctor {
  double slice_level = 1.0;

  bool call(nav_msgs::LoadMap::Request& req, nav_msgs::LoadMap::Response&) {
    slice_level = std::stod(req.map_url);
    return true;
  }
};

std::map<NodeId, double> get_errors(const SceneGraphLayer& places,
                                    const Layer<GvdVoxel>& gvd_layer) {
  std::map<NodeId, double> node_errors;
  for (const auto& id_node_pair : places.nodes()) {
    Eigen::Vector3d position = id_node_pair.second->attributes().position;
    voxblox::Point vox_pos = position.cast<float>();
    const GvdVoxel* voxel = gvd_layer.getVoxelPtrByCoordinates(vox_pos);
    if (!voxel || !voxel->observed) {
      continue;
    }

    const double node_distance =
        id_node_pair.second->attributes<PlaceNodeAttributes>().distance;
    node_errors[id_node_pair.first] = std::abs(voxel->distance - node_distance);
  }

  return node_errors;
}

void visualize_places(ros::Publisher& pub,
                      const SceneGraphLayer& layer,
                      const std::string& ns,
                      double offset,
                      std::map<NodeId, double>* errors = nullptr) {
  if (layer.nodes().empty()) {
    return;
  }

  std::string dsg_path = ros::package::getPath("kimera_dsg_builder");

  ColormapConfig colormap =
      readColormapConfig(ros::package::getPath("kimera_dsg_builder") +
                         "/config/incremental_visualizer/colormap.yaml");
  ColormapConfig error_colormap =
      readColormapConfig(ros::package::getPath("kimera_dsg_builder") +
                         "/config/incremental_visualizer/error_colormap.yaml");
  VisualizerConfig graph_cfg = readVisualizerConfig(
      ros::package::getPath("kimera_dsg_visualizer") + "/config/visualizer.yaml");
  LayerConfig layer_cfg =
      readLayerConfig(ros::package::getPath("kimera_dsg_builder") +
                      "/config/incremental_visualizer/places_layer.yaml");
  layer_cfg.z_offset_scale = offset;

  std_msgs::Header header;
  header.stamp = ros::Time::now();
  header.frame_id = "world";

  MarkerArray markers;

  Marker node_marker;
  if (!errors) {
    node_marker = makeCentroidMarkers(
        header, layer_cfg, layer, graph_cfg, ns + "_nodes", colormap);
  } else {
    node_marker = makeCentroidMarkers(
        header,
        layer_cfg,
        layer,
        graph_cfg,
        ns + "_nodes",
        [&](const SceneGraphNode& node) -> NodeColor {
          if (!errors->count(node.id)) {
            return NodeColor::Zero();
          }

          double ratio = errors->at(node.id) / FLAGS_max_error;
          ratio = ratio > 1.0 ? 1.0 : ratio;
          return dsg_utils::interpolateColorMap(error_colormap, ratio);
        });
  }

  markers.markers.push_back(node_marker);

  if (!layer.edges().empty()) {
    Marker edge_marker = makeLayerEdgeMarkers(
        header, layer_cfg, layer, graph_cfg, NodeColor::Zero(), ns + "_edges");
    markers.markers.push_back(edge_marker);
  }

  pub.publish(markers);
}

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "places_eval_viz_node");
  ros::NodeHandle nh("");

  FLAGS_minloglevel = 3;
  FLAGS_logtostderr = 1;
  FLAGS_colorlogtostderr = 1;

  google::SetUsageMessage("utility for comparing places graph to TSDF");
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);

  if (FLAGS_tsdf_file == "") {
    LOG(FATAL) << "TSDF file is required!";
  }

  if (FLAGS_dsg_file == "") {
    LOG(FATAL) << "DSG file is required!";
  }

  if (FLAGS_gvd_config == "") {
    LOG(FATAL) << "GVD config is required!";
  }

  Layer<TsdfVoxel>::Ptr tsdf(
      new Layer<TsdfVoxel>(FLAGS_voxel_size, FLAGS_voxels_per_side));
  const auto strat = Layer<TsdfVoxel>::BlockMergingStrategy::kReplace;
  if (!voxblox::io::LoadBlocksFromFile(FLAGS_tsdf_file, strat, true, tsdf.get())) {
    LOG(FATAL) << "Failed to load TSDF from: " << FLAGS_tsdf_file;
  }

  kimera::DynamicSceneGraph graph;
  graph.load(FLAGS_dsg_file);

  GvdIntegratorConfig gvd_config = readGvdConfig(FLAGS_gvd_config);
  showConfig(gvd_config);

  Layer<GvdVoxel>::Ptr gvd_layer(
      new Layer<GvdVoxel>(FLAGS_voxel_size, FLAGS_voxels_per_side));
  MeshLayer::Ptr mesh_layer(new MeshLayer(FLAGS_voxel_size * FLAGS_voxels_per_side));
  GvdIntegrator integrator(gvd_config, tsdf.get(), gvd_layer, mesh_layer);
  integrator.updateFromTsdfLayer(false, true, true);

  CHECK(graph.hasLayer(KimeraDsgLayers::PLACES));
  const SceneGraphLayer& places = graph.getLayer(KimeraDsgLayers::PLACES).value();
  auto errors = get_errors(places, *gvd_layer);

  ros::Publisher gt_pub =
      nh.advertise<visualization_msgs::MarkerArray>("gt_places", 1, true);

  ros::Publisher est_pub =
      nh.advertise<visualization_msgs::MarkerArray>("est_places", 1, true);

  ros::Publisher gvd_pub =
      nh.advertise<visualization_msgs::MarkerArray>("gvd", 1, true);

  ros::Publisher esdf_pub =
      nh.advertise<visualization_msgs::MarkerArray>("esdf", 1, true);

  ros::Publisher vxblx_mesh_pub =
      nh.advertise<voxblox_msgs::Mesh>("vxblx_mesh", 1, true);

  ColormapConfig colormap =
      readColormapConfig(ros::package::getPath("kimera_dsg_builder") +
                         "/config/incremental_visualizer/colormap.yaml");

  GvdVisualizerConfig gvd_viz_config;
  gvd_viz_config.gvd_min_distance = FLAGS_min_cmap_distance;
  gvd_viz_config.gvd_max_distance = FLAGS_max_cmap_distance;
  gvd_viz_config.gvd_alpha = 0.6;
  gvd_viz_config.esdf_min_distance = FLAGS_min_cmap_distance;
  gvd_viz_config.esdf_max_distance = FLAGS_max_cmap_distance;
  gvd_viz_config.esdf_alpha = 1.0;
  gvd_viz_config.basis_threshold = FLAGS_basis_threshold;

  MarkerArray gvd_msg;
  gvd_msg.markers.push_back(makeGvdMarker(gvd_viz_config, colormap, *gvd_layer));
  gvd_msg.markers.front().header.frame_id = "world";
  gvd_msg.markers.front().header.stamp = ros::Time::now();
  gvd_pub.publish(gvd_msg);

  voxblox_msgs::Mesh mesh_msg;
  generateVoxbloxMeshMsg(mesh_layer, voxblox::ColorMode::kColor, &mesh_msg);
  mesh_msg.header.frame_id = "world";
  mesh_msg.header.stamp = ros::Time::now();
  vxblx_mesh_pub.publish(mesh_msg);

  visualize_places(
      gt_pub, integrator.getGraphExtractor().getGraph(), "gt", 0.5 * FLAGS_offset);
  visualize_places(est_pub, places, "est", FLAGS_offset, &errors);

  ToggleFunctor functor;
  ros::ServiceServer server =
      nh.advertiseService("toggle_visualizer", &ToggleFunctor::call, &functor);

  SliceLevelFunctor slice_functor;
  ros::ServiceServer slice_server =
      nh.advertiseService("esdf_slice_level", &SliceLevelFunctor::call, &slice_functor);

  ros::Rate r(10);
  while (ros::ok()) {
    visualize_places(
        est_pub, places, "est", FLAGS_offset, functor.show_error ? &errors : nullptr);

    MarkerArray esdf_msg;
    gvd_viz_config.slice_height = slice_functor.slice_level;
    esdf_msg.markers.push_back(makeEsdfMarker(gvd_viz_config, colormap, *gvd_layer));
    esdf_msg.markers.front().header.frame_id = "world";
    esdf_msg.markers.front().header.stamp = ros::Time::now();
    esdf_pub.publish(esdf_msg);

    r.sleep();
    ros::spinOnce();
  }

  return 0;
}
