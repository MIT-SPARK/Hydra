#include "kimera_dsg_builder/voxblox_utils.h"
#include "kimera_dsg_builder/common.h"

#include <kimera_dsg/node_attributes.h>
#include <kimera_pgmo/utils/CommonFunctions.h>
#include <kimera_pgmo/utils/VoxbloxUtils.h>
#include <kimera_topology/config_parser.h>
#include <kimera_topology/gvd_integrator.h>

#include <voxblox/mesh/mesh_integrator.h>
#include <voxblox_ros/mesh_pcl.h>
#include <voxblox_ros/mesh_vis.h>
#include <voxblox_ros/ros_params.h>

#include <voxblox_skeleton/skeleton_generator.h>

#include <glog/logging.h>

namespace kimera {

namespace utils {

using namespace voxblox;
using namespace topology;

#define READ_PARAM(nh, config, field, default)                                   \
  if (!nh.param(#field, config.field, default)) {                                \
    VLOG(1) << "missing value for " << #field << ". defaulting to: " << default; \
  }                                                                              \
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
    LOG(ERROR) << "Missing voxels per side under namespace " << nh.getNamespace();
    return std::nullopt;
  }
  config.voxels_per_side = static_cast<size_t>(voxels_per_side);

  READ_PARAM(nh, config, tsdf_file, std::string(""));
  READ_PARAM(nh, config, esdf_file, std::string(""));
  READ_PARAM(nh, config, mesh_file, std::string(""));
  READ_PARAM(nh, config, gvd_namespace, std::string("gvd_integrator"));
  READ_PARAM(nh, config, load_esdf, true);
  READ_PARAM(nh, config, load_mesh, true);
  return config;
}

#undef READ_PARAM

inline bool loadEsdfFromFile(const VoxbloxConfig& config, Layer<EsdfVoxel>::Ptr& esdf) {
  esdf.reset(new Layer<EsdfVoxel>(config.voxel_size, config.voxels_per_side));
  const auto strat = Layer<EsdfVoxel>::BlockMergingStrategy::kReplace;
  return voxblox::io::LoadBlocksFromFile(config.esdf_file, strat, true, esdf.get());
}

inline bool loadMeshFromFile(const VoxbloxConfig& config, pcl::PolygonMesh::Ptr& mesh) {
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

void makeMeshFromTsdf(const Layer<TsdfVoxel>& tsdf,
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

  // Mesh full_mesh;
  // convertMeshLayerToMesh(voxblox_mesh, &full_mesh, true, 1.0e-10f);

  pcl::PointCloud<pcl::PointXYZRGBA> vertices;
  // vertices.reserve(full_mesh.size());
  voxblox::BlockIndexList blocks;
  voxblox_mesh.getAllAllocatedMeshes(&blocks);
  for (const auto& mesh_idx : blocks) {
    const auto& full_mesh = voxblox_mesh.getMeshByIndex(mesh_idx);
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

    pcl::Vertices curr_vertices;
    for (const auto& idx : full_mesh.indices) {
      curr_vertices.vertices.push_back(idx);
      if (curr_vertices.vertices.size() == 3) {
        mesh->polygons.push_back(curr_vertices);
        curr_vertices.vertices.clear();
      }
    }
  }

  pcl::toPCLPointCloud2(vertices, mesh->cloud);
}

void makePlacesFromTsdf(const VoxbloxConfig& config,
                        Layer<TsdfVoxel>* tsdf,
                        SceneGraph* graph) {
  CHECK(graph);
  CHECK(tsdf);

  GvdIntegratorConfig gvd_config;
  fillGvdIntegratorConfig(ros::NodeHandle(config.gvd_namespace), gvd_config);
  topology::showConfig(gvd_config);

  Layer<GvdVoxel>::Ptr gvd(
      new Layer<GvdVoxel>(tsdf->voxel_size(), tsdf->voxels_per_side()));
  MeshLayer::Ptr mesh(new MeshLayer(tsdf->block_size()));

  GvdIntegrator integrator(gvd_config, tsdf, gvd, mesh);
  // do batch update of gvd
  integrator.updateFromTsdfLayer(false, true, true);

  const SceneGraphLayer& places_layer = integrator.getGraph();
  for (const auto& id_node_pair : places_layer.nodes()) {
    const SceneGraphNode& other_node = *id_node_pair.second;
    PlaceNodeAttributes::Ptr new_attrs(
        new PlaceNodeAttributes(other_node.attributes<PlaceNodeAttributes>()));
    graph->emplaceNode(KimeraDsgLayers::PLACES, other_node.id, std::move(new_attrs));
  }

  for (const auto& id_edge_pair : places_layer.edges()) {
    const auto& edge = id_edge_pair.second;
    SceneGraphEdgeInfo::Ptr info(new SceneGraphEdgeInfo(*id_edge_pair.second.info));
    graph->insertEdge(edge.source, edge.target, std::move(info));
  }
}

void makePlacesFromEsdfVoxblox(const VoxbloxConfig& config,
                               const Layer<EsdfVoxel>::Ptr& esdf,
                               SceneGraph* graph) {
  voxblox::SkeletonGenerator generator;
  generator.setEsdfLayer(esdf.get());
  generator.setMinSeparationAngle(config.min_separation_angle);
  generator.setGenerateByLayerNeighbors(config.generate_by_layer_neighbors);
  generator.setNumNeighborsForEdge(config.num_neighbors_for_edge);
  generator.setMinGvdDistance(config.min_gvd_distance);
  generator.generateSkeleton();
  generator.generateSparseGraph();

  const SparseSkeletonGraph& skeleton = generator.getSparseGraph();

  std::vector<int64_t> vertex_ids;
  skeleton.getAllVertexIds(&vertex_ids);

  for (const auto& idx : vertex_ids) {
    const auto& vertex = skeleton.getVertex(idx);

    auto attrs = std::make_unique<PlaceNodeAttributes>(vertex.distance, 0);
    attrs->semantic_label = kPlaceSemanticLabel;
    attrs->position << vertex.point[0], vertex.point[1], vertex.point[2];
    graph->emplaceNode(KimeraDsgLayers::PLACES, NodeSymbol('p', idx), std::move(attrs));
  }

  std::vector<int64_t> edge_ids;
  skeleton.getAllEdgeIds(&edge_ids);

  for (const auto& edge_id : edge_ids) {
    const auto& edge = skeleton.getEdge(edge_id);
    if (edge.start_vertex == edge.end_vertex) {
      continue;
    }

    graph->insertEdge(NodeSymbol('p', edge.start_vertex),
                      NodeSymbol('p', edge.end_vertex));
  }
}

bool updateFromTsdf(const VoxbloxConfig& config,
                    Layer<TsdfVoxel>& tsdf,
                    Layer<EsdfVoxel>::Ptr& esdf,
                    pcl::PolygonMesh::Ptr& mesh,
                    SceneGraph* graph) {
  makeEsdfFromTsdf(config, tsdf, esdf);

  makeMeshFromTsdf(tsdf, mesh, nullptr);

  LOG(INFO) << "Starting places extraction. May take a while";
  makePlacesFromEsdfVoxblox(config, esdf, graph);
  LOG(INFO) << "Finished places extraction.";
  return true;
}

bool loadVoxbloxInfo(const VoxbloxConfig& config,
                     Layer<EsdfVoxel>::Ptr& esdf,
                     pcl::PolygonMesh::Ptr& mesh,
                     ros::Publisher* mesh_pub,
                     SceneGraph* graph) {
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

  if (config.load_places) {
    if (!graph) {
      LOG(ERROR) << "Scene graph pointer invalid.";
      return false;
    }

    LOG(INFO) << "Starting places extraction. May take a while";
    makePlacesFromTsdf(config, &tsdf, graph);
    LOG(INFO) << "Finished places extraction.";
  }

  return true;
}

}  // namespace utils

}  // namespace kimera
