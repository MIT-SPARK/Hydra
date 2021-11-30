#pragma once
#include <ros/ros.h>

#include <kimera_dsg/scene_graph.h>
#include <pcl/PolygonMesh.h>
#include <voxblox/integrator/esdf_integrator.h>

namespace kimera {

namespace utils {

struct VoxbloxConfig {
  voxblox::EsdfIntegrator::Config esdf_config;
  std::string tsdf_file;
  std::string esdf_file;
  std::string mesh_file;
  std::string gvd_namespace;
  bool load_places = false;
  bool load_esdf;
  bool load_mesh;
  double voxel_size;
  size_t voxels_per_side;
  double min_gvd_distance = 0.05;
  double min_separation_angle = 0.78;
  bool generate_by_layer_neighbors = false;
  size_t num_neighbors_for_edge = 18;
};

std::optional<VoxbloxConfig> loadVoxbloxConfig(const ros::NodeHandle& nh);

void makeMeshFromTsdf(const voxblox::Layer<voxblox::TsdfVoxel>& tsdf,
                      pcl::PolygonMesh::Ptr& mesh,
                      ros::Publisher* mesh_pub);

bool loadVoxbloxInfo(const VoxbloxConfig& config,
                     voxblox::Layer<voxblox::EsdfVoxel>::Ptr& esdf,
                     pcl::PolygonMesh::Ptr& mesh,
                     ros::Publisher* mesh_pub = nullptr,
                     SceneGraph* graph = nullptr);

bool updateFromTsdf(const VoxbloxConfig& config,
                    voxblox::Layer<voxblox::TsdfVoxel>& tsdf,
                    voxblox::Layer<voxblox::EsdfVoxel>::Ptr& esdf,
                    pcl::PolygonMesh::Ptr& mesh,
                    SceneGraph* graph);

}  // namespace utils

}  // namespace kimera
