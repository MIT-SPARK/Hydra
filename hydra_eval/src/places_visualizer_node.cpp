#include "kimera_topology/topology_server_visualizer.h"

#include <kimera_dsg/dynamic_scene_graph.h>
#include <kimera_dsg_visualizer/colormap_utils.h>
#include <kimera_topology/GvdVisualizerConfig.h>
#include <nav_msgs/LoadMap.h>
#include <voxblox/io/layer_io.h>

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

namespace kimera_dsg_visualizer {

template <typename Visitor>
void visit_config(const Visitor& v, ColormapConfig& config) {
  v.visit("min_hue", config.min_hue);
  v.visit("max_hue", config.max_hue);
  v.visit("min_saturation", config.min_saturation);
  v.visit("max_saturation", config.max_saturation);
  v.visit("min_luminance", config.min_luminance);
  v.visit("max_luminance", config.max_luminance);
}

template <typename Visitor>
void visit_config(const Visitor& v, VisualizerConfig& config) {
  v.visit("layer_z_step", config.layer_z_step);
  v.visit("mesh_edge_break_ratio", config.mesh_edge_break_ratio);
  v.visit("mesh_layer_offset", config.mesh_layer_offset);
  v.visit("collapse_layers", config.collapse_layers);
  v.visit("color_places_by_distance", config.color_places_by_distance);
}

template <typename Visitor>
void visit_config(const Visitor& v, LayerConfig& config) {
  v.visit("z_offset_scale", config.z_offset_scale);
  v.visit("visualize", config.visualize);
  v.visit("marker_scale", config.marker_scale);
  v.visit("marker_alpha", config.marker_alpha);
  v.visit("use_sphere_marker", config.use_sphere_marker);
  v.visit("use_label", config.use_label);
  v.visit("label_height", config.label_height);
  v.visit("label_scale", config.label_scale);
  v.visit("use_bounding_box", config.use_bounding_box);
  v.visit("bounding_box_alpha", config.bounding_box_alpha);
  v.visit("use_edge_source", config.use_edge_source);
  v.visit("interlayer_edge_scale", config.interlayer_edge_scale);
  v.visit("interlayer_edge_alpha", config.interlayer_edge_alpha);
  v.visit("interlayer_edge_use_color", config.interlayer_edge_use_color);
  v.visit("interlayer_edge_insertion_skip", config.interlayer_edge_insertion_skip);
  v.visit("intralayer_edge_scale", config.intralayer_edge_scale);
  v.visit("intralayer_edge_alpha", config.intralayer_edge_alpha);
  v.visit("intralayer_edge_insertion_skip", config.intralayer_edge_insertion_skip);
}

}  // namespace kimera_dsg_visualizer

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

  auto colormap = config_parser::load_from_yaml<ColormapConfig>(
      dsg_path + "/config/incremental_visualizer/colormap.yaml");
  auto error_colormap = config_parser::load_from_yaml<ColormapConfig>(
      dsg_path + "/config/incremental_visualizer/error_colormap.yaml");
  auto graph_cfg = config_parser::load_from_yaml<VisualizerConfig>(
      dsg_path + "/config/visualizer.yaml");
  graph_cfg.places_colormap_min_distance = FLAGS_min_cmap_distance;
  graph_cfg.places_colormap_max_distance = FLAGS_max_cmap_distance;

  auto layer_cfg = config_parser::load_from_yaml<LayerConfig>(
      dsg_path + "/config/incremental_visualizer/places_layer.yaml");
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

  DynamicSceneGraph graph;
  graph.load(FLAGS_dsg_file);

  auto gvd_config =
      config_parser::load_from_yaml<GvdIntegratorConfig>(FLAGS_gvd_config);
  gvd_config.max_distance_m = FLAGS_max_distance_m;
  gvd_config.extract_graph = true;
  gvd_config.mesh_only = false;

  LOG(INFO) << "Gvd Config: " << std::endl << gvd_config;

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

  auto colormap = config_parser::load_from_yaml<ColormapConfig>(
      ros::package::getPath("kimera_dsg_builder") +
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
