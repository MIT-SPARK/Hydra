#include "kimera_topology/configs.h"

#include <kimera_dsg/dynamic_scene_graph.h>
#include <voxblox/io/layer_io.h>
#include <nanoflann.hpp>

#include <gflags/gflags.h>
#include <glog/logging.h>

DEFINE_int32(voxels_per_side, 16, "voxels per side");
DEFINE_double(voxel_size, 0.1, "voxel size");
DEFINE_double(max_distance_m, 4.5, "max distance");
DEFINE_string(tsdf_file, "", "tsdf file to read");
DEFINE_string(dsg_file, "", "dsg file to read");
DEFINE_string(gvd_config, "", "gvd integrator config");

using kimera::DynamicSceneGraph;
using kimera::KimeraDsgLayers;
using kimera::NodeId;
using kimera::PlaceNodeAttributes;
using kimera::SceneGraphLayer;
using kimera::topology::GvdIntegrator;
using kimera::topology::GvdIntegratorConfig;
using kimera::topology::GvdVoxel;
using nanoflann::KDTreeSingleIndexAdaptor;
using nanoflann::KDTreeSingleIndexDynamicAdaptor;
using nanoflann::L2_Simple_Adaptor;
using voxblox::Layer;
using voxblox::MeshLayer;
using voxblox::TsdfVoxel;

void fillGvdPositions(const GvdIntegratorConfig& gvd_config,
                      const Layer<GvdVoxel>& layer,
                      voxblox::AlignedVector<Eigen::Vector3d>& result) {
  result.clear();

  voxblox::BlockIndexList blocks;
  layer.getAllAllocatedBlocks(&blocks);

  for (const auto& idx : blocks) {
    const auto& block = layer.getBlockByIndex(idx);
    for (size_t i = 0; i < block.num_voxels(); ++i) {
      const auto& voxel = block.getVoxelByLinearIndex(i);
      if (!voxel.observed ||
          voxel.num_extra_basis < gvd_config.min_basis_for_extraction) {
        continue;
      }

      result.push_back(block.computeCoordinatesFromLinearIndex(i).cast<double>());
    }
  }
}

struct VoxelKdTreeAdaptor {
  VoxelKdTreeAdaptor(const voxblox::AlignedVector<Eigen::Vector3d>& positions)
      : positions(positions) {}

  inline size_t kdtree_get_point_count() const { return positions.size(); }

  inline double kdtree_get_pt(const size_t idx, const size_t dim) const {
    return positions[idx](dim);
  }

  template <class T>
  bool kdtree_get_bbox(T&) const {
    return false;
  }

  voxblox::AlignedVector<Eigen::Vector3d> positions;
};

struct DistanceFinder {
  using Dist = L2_Simple_Adaptor<double, VoxelKdTreeAdaptor>;
  using KDTree = KDTreeSingleIndexAdaptor<Dist, VoxelKdTreeAdaptor, 3, size_t>;

  DistanceFinder(voxblox::AlignedVector<Eigen::Vector3d>& positions)
      : adaptor(positions) {
    kdtree.reset(new KDTree(3, adaptor));
    kdtree->buildIndex();
  }

  double distance(const Eigen::Vector3d& pos) {
    size_t idx;
    double dist;
    size_t num_found = kdtree->knnSearch(pos.data(), 1, &idx, &dist);
    return num_found ? std::sqrt(dist) : std::numeric_limits<double>::quiet_NaN();
  }

  VoxelKdTreeAdaptor adaptor;
  std::unique_ptr<KDTree> kdtree;
};

void eval_layer(const GvdIntegratorConfig& gvd_config,
                const SceneGraphLayer& places,
                const Layer<GvdVoxel>& gvd_layer) {
  voxblox::AlignedVector<Eigen::Vector3d> gvd_positions;
  fillGvdPositions(gvd_config, gvd_layer, gvd_positions);
  DistanceFinder finder(gvd_positions);

  double mean = 0.0;
  size_t missing = 0;
  size_t unobserved = 0;
  size_t valid = 0;
  std::vector<double> dist_errors;
  std::vector<double> dist_to_closest;
  std::vector<NodeId> node_order;
  for (const auto& id_node_pair : places.nodes()) {
    Eigen::Vector3d position = id_node_pair.second->attributes().position;
    dist_to_closest.push_back(finder.distance(position));
    node_order.push_back(id_node_pair.first);

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

    ++valid;
    const double node_distance =
        id_node_pair.second->attributes<PlaceNodeAttributes>().distance;
    dist_errors.push_back(std::abs(voxel->distance - node_distance));
    mean += std::abs(voxel->distance - node_distance);
  }

  mean /= valid;
  auto min = std::min_element(dist_errors.begin(), dist_errors.end());
  auto max = std::max_element(dist_errors.begin(), dist_errors.end());

  nlohmann::json json_results = {
      {"missing", missing},
      {"valid", valid},
      {"dist_errors", dist_errors},
      {"closest_dists", dist_to_closest},
      {"total", places.numNodes()},
      {"mean", mean},
      {"nodes", node_order},
      {"min", valid ? *min : std::numeric_limits<double>::quiet_NaN()},
      {"max", valid ? *max : std::numeric_limits<double>::quiet_NaN()},
  };
  std::cout << json_results << std::endl;
}

void eval_places(const DynamicSceneGraph& graph, const Layer<TsdfVoxel>::Ptr& tsdf) {
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
  eval_layer(gvd_config, places, *gvd_layer);
}

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

  DynamicSceneGraph graph;
  graph.load(FLAGS_dsg_file);

  eval_places(graph, tsdf);
  return 0;
}
