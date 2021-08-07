#include "kimera_dsg_builder/voxblox_utils.h"
#include "kimera_dsg_builder/common.h"

#include <kimera_dsg/node_attributes.h>
#include <kimera_pgmo/utils/CommonFunctions.h>
#include <kimera_pgmo/utils/VoxbloxUtils.h>

#include <voxblox/mesh/mesh_integrator.h>
#include <voxblox_ros/mesh_vis.h>
#include <voxblox_ros/ros_params.h>
#include <voxblox_skeleton/io/skeleton_io.h>
#include <voxblox_skeleton/skeleton.h>
#include <voxblox_ros/mesh_pcl.h>

#include <glog/logging.h>

namespace kimera {

namespace utils {

using namespace voxblox;

void fillLayerFromSkeleton(const std::string& filepath,
                           SceneGraph* scene_graph) {
  CHECK(scene_graph);

  SparseSkeletonGraph skeleton;
  CHECK(voxblox::io::loadSparseSkeletonGraphFromFile(filepath, &skeleton));

  std::vector<int64_t> vertex_ids;
  skeleton.getAllVertexIds(&vertex_ids);

  for (const auto& idx : vertex_ids) {
    const SkeletonVertex& vertex = skeleton.getVertex(idx);

    // TODO(nathan) basis points aren't actually propagated through
    PlaceNodeAttributes::Ptr attrs =
        std::make_unique<PlaceNodeAttributes>(vertex.distance, 0);
    attrs->semantic_label = kPlaceSemanticLabel;
    attrs->position << vertex.point[0], vertex.point[1], vertex.point[2];
    attrs->color << 255u, 0u, 0u;

    scene_graph->emplaceNode(to_underlying(KimeraDsgLayers::PLACES),
                             NodeSymbol('P', idx),
                             std::move(attrs));
  }

  std::vector<int64_t> edge_ids;
  skeleton.getAllEdgeIds(&edge_ids);

  for (const auto& edge_id : edge_ids) {
    const SkeletonEdge& edge = skeleton.getEdge(edge_id);
    if (edge.start_vertex == edge.end_vertex) {
      continue;
    }

    scene_graph->insertEdge(NodeSymbol('P', edge.start_vertex),
                            NodeSymbol('P', edge.end_vertex));
  }
}

#define READ_PARAM(nh, config, field, default)    \
  if (!nh.param(#field, config.field, default)) { \
    VLOG(1) << "missing value for " << #field     \
            << ". defaulting to: " << default;    \
  }                                               \
  static_assert(true, "")

std::optional<VoxbloxConfig> loadVoxbloxConfig(const ros::NodeHandle& nh) {
  VoxbloxConfig config;
  config.esdf_config = getEsdfIntegratorConfigFromRosParam(nh);

  if (!nh.getParam("voxel_size", config.voxel_size)) {
    LOG(ERROR) << "Missing voxel size under namespace " << nh.getNamespace();
    return std::nullopt;
  }

  int voxels_per_side;
  if (!nh.getParam("voxels_per_side", voxels_per_side)) {
    LOG(ERROR) << "Missing voxels per side under namespace "
               << nh.getNamespace();
    return std::nullopt;
  }
  config.voxels_per_side = static_cast<size_t>(voxels_per_side);

  READ_PARAM(nh, config, tsdf_file, std::string(""));
  READ_PARAM(nh, config, esdf_file, std::string(""));
  READ_PARAM(nh, config, mesh_file, std::string(""));
  READ_PARAM(nh, config, load_esdf, true);
  READ_PARAM(nh, config, load_mesh, true);
  return config;
}

#undef READ_PARAM

namespace {

inline bool loadEsdfFromFile(const VoxbloxConfig& config,
                             Layer<EsdfVoxel>::Ptr& esdf) {
  esdf.reset(new Layer<EsdfVoxel>(config.voxel_size, config.voxels_per_side));
  const auto strat = Layer<EsdfVoxel>::BlockMergingStrategy::kReplace;
  return voxblox::io::LoadBlocksFromFile(
      config.esdf_file, strat, true, esdf.get());
}

inline bool loadMeshFromFile(const VoxbloxConfig& config,
                             pcl::PolygonMesh::Ptr& mesh) {
  mesh.reset(new pcl::PolygonMesh());
  kimera_pgmo::ReadMeshFromPly(config.mesh_file, mesh);
  return true;
}

inline void makeEsdfFromTsdf(const VoxbloxConfig& config,
                             Layer<TsdfVoxel>& tsdf,
                             Layer<EsdfVoxel>::Ptr& esdf) {
  esdf.reset(new Layer<EsdfVoxel>(config.voxel_size, config.voxels_per_side));

  EsdfIntegrator integrator(config.esdf_config, &tsdf, esdf.get());
  integrator.setFullEuclidean(true);
  integrator.updateFromTsdfLayerBatch();
}

inline void makeMeshFromTsdf(const Layer<TsdfVoxel>& tsdf,
                             pcl::PolygonMesh::Ptr& mesh,
                             ros::Publisher* mesh_pub) {
  MeshIntegratorConfig config;
  MeshLayer voxblox_mesh(tsdf.block_size());
  MeshIntegrator<TsdfVoxel> integrator(config, tsdf, &voxblox_mesh);
  integrator.generateMesh(false, false);

  voxblox_msgs::Mesh::Ptr mesh_msg(new voxblox_msgs::Mesh());
  mesh_msg->header.stamp = ros::Time::now();
  mesh_msg->header.frame_id = "world";
  generateVoxbloxMeshMsg(&voxblox_mesh, ColorMode::kColor, mesh_msg.get());
  if (mesh_pub != nullptr) {
    mesh_pub->publish(*mesh_msg);
  }

  mesh.reset(new pcl::PolygonMesh());
  // TODO(nathan) not sure why this isn't working
  //*mesh = kimera_pgmo::VoxbloxToPolygonMesh(mesh_msg);
  Mesh full_mesh;
  convertMeshLayerToMesh(voxblox_mesh, &full_mesh, true, 1.0e-10f);

  pcl::PointCloud<pcl::PointXYZRGBA> vertices;
  vertices.reserve(full_mesh.size());
  for (size_t i = 0; i < full_mesh.size(); ++i) {
    pcl::PointXYZRGBA point;
    point.x = full_mesh.vertices.at(i).x();
    point.y = full_mesh.vertices.at(i).y();
    point.z = full_mesh.vertices.at(i).z();
    if (full_mesh.hasColors()) {
      point.r = full_mesh.colors.at(i).r;
      point.g = full_mesh.colors.at(i).g;
      point.b = full_mesh.colors.at(i).b;
      point.a = 255;
    }
    vertices.push_back(point);
  }
  pcl::toPCLPointCloud2(vertices, mesh->cloud);

  pcl::Vertices curr_vertices;
  for (const auto& idx : full_mesh.indices) {
    curr_vertices.vertices.push_back(idx);
    if (curr_vertices.vertices.size() == 3) {
      mesh->polygons.push_back(curr_vertices);
      curr_vertices.vertices.clear();
    }
  }
}

}  // namespace

bool loadVoxbloxInfo(const VoxbloxConfig& config,
                     Layer<EsdfVoxel>::Ptr& esdf,
                     pcl::PolygonMesh::Ptr& mesh,
                     ros::Publisher* mesh_pub) {
  if (!config.load_esdf && !config.load_mesh) {
    LOG(ERROR) << "Invalid config for loading voxblox files";
    return false;
  }

  bool have_esdf = !config.load_esdf;
  if (!have_esdf && !config.esdf_file.empty()) {
    have_esdf = loadEsdfFromFile(config, esdf);
    if (!have_esdf) {
      LOG(WARNING) << "Failed to load esdf from: " << config.esdf_file;
      LOG(WARNING) << "Will try and reconstruct from the TSDF";
    }
  }

  bool have_mesh = !config.load_mesh;
  if (!have_mesh && !config.mesh_file.empty()) {
    have_mesh = loadMeshFromFile(config, mesh);
  }

  if (have_mesh && have_esdf) {
    return true;
  }

  Layer<TsdfVoxel> tsdf(config.voxel_size, config.voxels_per_side);
  const auto strat = Layer<TsdfVoxel>::BlockMergingStrategy::kReplace;
  if (!voxblox::io::LoadBlocksFromFile(config.tsdf_file, strat, true, &tsdf)) {
    LOG(ERROR) << "Failed to load TSDF layer from " << config.tsdf_file;
    return false;
  }

  if (!have_esdf) {
    makeEsdfFromTsdf(config, tsdf, esdf);
  }

  if (!have_mesh) {
    makeMeshFromTsdf(tsdf, mesh, mesh_pub);
  }

  return true;
}

}  // namespace utils

}  // namespace kimera
