#include <kimera_dsg/dynamic_scene_graph.h>
#include <voxblox/io/layer_io.h>
#include <yaml-cpp/yaml.h>
#include "kimera_topology/config_parser.h"
#include "kimera_topology/gvd_integrator.h"

#include <gflags/gflags.h>
#include <glog/logging.h>

DEFINE_int32(voxels_per_side, 16, "voxels per side");
DEFINE_double(voxel_size, 0.1, "voxel size");
DEFINE_double(max_distance_m, 4.5, "max distance");
DEFINE_string(tsdf_file, "", "tsdf file to read");
DEFINE_string(dsg_file, "", "dsg file to read");
DEFINE_string(gvd_config, "", "gvd integrator config");
DEFINE_bool(extract_graph, false, "extract graph from GVD");
DEFINE_bool(suppress_data, false, "collapse lists to mean");

using voxblox::Layer;
using voxblox::TsdfVoxel;

namespace kimera {
namespace topology {

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

  config.extract_graph = FLAGS_extract_graph;
  if (config_root["graph_extractor"]) {
    config.graph_extractor_config = readGraphConfig(config_root["graph_extractor"]);
  }
  config.mesh_only = false;

  return config;
}

void eval_layer(const GvdIntegratorConfig& gvd_config,
                const SceneGraphLayer& places,
                const Layer<GvdVoxel>& gvd_layer) {
  size_t missing = 0;
  size_t unobserved = 0;
  size_t num_correct = 0;
  std::vector<double> dist_errors;
  double mean = 0.0;
  for (const auto& id_node_pair : places.nodes()) {
    Eigen::Vector3d position = id_node_pair.second->attributes().position;
    voxblox::Point vox_pos = position.cast<float>();
    const GvdVoxel* voxel = gvd_layer.getVoxelPtrByCoordinates(vox_pos);
    if (!voxel) {
      missing++;
      continue;
    }

    if (!voxel->observed) {
      unobserved++;
      continue;
    }

    if (voxel->num_extra_basis >= gvd_config.min_basis_for_extraction) {
      num_correct++;
    }

    const double node_distance =
        id_node_pair.second->attributes<PlaceNodeAttributes>().distance;
    dist_errors.push_back(std::abs(voxel->distance - node_distance));
    mean += std::abs(voxel->distance - node_distance);
  }

  mean /= num_correct;

  if (FLAGS_suppress_data) {
    auto min = std::min_element(dist_errors.begin(), dist_errors.end());
    auto max = std::max_element(dist_errors.begin(), dist_errors.end());

    nlohmann::json json_results = {
        {"missing", missing},
        {"correct", num_correct},
        {"dist_errors", mean},
        {"total", places.numNodes()},
        {"min", *min},
        {"max", *max},
    };
    std::cout << json_results << std::endl;
  } else {
    nlohmann::json json_results = {
        {"missing", missing},
        {"correct", num_correct},
        {"dist_errors", dist_errors},
        {"total", places.numNodes()},
    };
    std::cout << json_results << std::endl;
  }
}  // namespace topology

// TODO(nathan) export and test
void eval_places(const DynamicSceneGraph& graph, const Layer<TsdfVoxel>::Ptr& tsdf) {
  GvdIntegratorConfig gvd_config = readGvdConfig(FLAGS_gvd_config);
  showConfig(gvd_config);

  Layer<GvdVoxel>::Ptr gvd_layer(
      new Layer<GvdVoxel>(FLAGS_voxel_size, FLAGS_voxels_per_side));
  MeshLayer::Ptr mesh_layer(new MeshLayer(FLAGS_voxel_size * FLAGS_voxels_per_side));
  GvdIntegrator integrator(gvd_config, tsdf.get(), gvd_layer, mesh_layer);
  integrator.updateFromTsdfLayer(false, true, true);

  CHECK(graph.hasLayer(KimeraDsgLayers::PLACES));
  const SceneGraphLayer& places = graph.getLayer(KimeraDsgLayers::PLACES).value();
  eval_layer(gvd_config, places, *gvd_layer);

  if (gvd_config.extract_graph) {
    std::cout << "gvd integrator graph: ";
    eval_layer(gvd_config, integrator.getGraphExtractor().getGraph(), *gvd_layer);

    DynamicSceneGraph new_graph;
    const SceneGraphLayer& new_places = integrator.getGraphExtractor().getGraph();

    for (const auto& id_node_pair : new_places.nodes()) {
      const SceneGraphNode& other_node = *id_node_pair.second;
      PlaceNodeAttributes::Ptr new_attrs(
          new PlaceNodeAttributes(other_node.attributes<PlaceNodeAttributes>()));
      new_graph.emplaceNode(
          KimeraDsgLayers::PLACES, other_node.id, std::move(new_attrs));
    }

    for (const auto& id_edge_pair : new_places.edges()) {
      const auto& edge = id_edge_pair.second;
      SceneGraphEdgeInfo::Ptr info(new SceneGraphEdgeInfo(*id_edge_pair.second.info));
      new_graph.insertEdge(edge.source, edge.target, std::move(info));
    }

    new_graph.save("/tmp/dsg.json");

    DynamicSceneGraph new_dsg;
    new_dsg.load("/tmp/dsg.json");

    std::cout << "serialized places: ";
    eval_layer(gvd_config,
               new_dsg.getLayer(KimeraDsgLayers::PLACES).value().get(),
               *gvd_layer);
  }
}

}  // namespace topology
}  // namespace kimera

int main(int argc, char* argv[]) {
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

  kimera::topology::eval_places(graph, tsdf);
  return 0;
}
