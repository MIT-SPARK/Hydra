#pragma once
#include <ros/ros.h>

#include <kimera_dsg/scene_graph.h>
#include <pcl/PolygonMesh.h>
#include <voxblox/integrator/esdf_integrator.h>

namespace kimera {

namespace utils {

void fillLayerFromSkeleton(const std::string& skeleton_file,
                           SceneGraph* scene_graph);

struct VoxbloxConfig {
  voxblox::EsdfIntegrator::Config esdf_config;
  std::string tsdf_file;
  std::string esdf_file;
  std::string mesh_file;
  bool load_esdf;
  bool load_mesh;
  double voxel_size;
  size_t voxels_per_side;
};

std::optional<VoxbloxConfig> loadVoxbloxConfig(const ros::NodeHandle& nh);

bool loadVoxbloxInfo(const VoxbloxConfig& config,
                     voxblox::Layer<voxblox::EsdfVoxel>::Ptr& esdf,
                     pcl::PolygonMesh::Ptr& mesh,
                     ros::Publisher* mesh_pub = nullptr);

}  // namespace utils

}  // namespace kimera
