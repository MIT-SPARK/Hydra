#include "kimera_topology_test/test_fixtures.h"
#include "kimera_topology_test/test_visualizations.h"

#include <gtest/gtest.h>

#include <kimera_topology/gvd_integrator.h>

namespace kimera {
namespace topology {

using test_helpers::SingleBlockTestFixture;
using voxblox::Colors;
using voxblox::Pointcloud;
using voxblox::SimpleTsdfIntegrator;
using voxblox::Transformation;
using voxblox::TsdfIntegratorBase;
using voxblox::VoxelIndex;

class GvdVizFixture : public test_helpers::GvdTestFixture,
                      public test_helpers::TestVisualizer {
 public:
  GvdVizFixture() : test_helpers::GvdTestFixture(), test_helpers::TestVisualizer() {}

  void showPointcloud(size_t index) {
    Transformation world_T_camera = getPose(index);

    Eigen::Vector2i resolution;
    resolution << depth_camera_width, depth_camera_height;

    Colors colors;
    Pointcloud camera_pointcloud;
    world.getPointcloudFromTransform(world_T_camera,
                                     resolution,
                                     depth_camera_fov,
                                     depth_camera_max_distance,
                                     &camera_pointcloud,
                                     &colors);

    visualizePointcloud(camera_pointcloud);
  }

  void showGvd(const Layer<GvdVoxel>& layer) { visualizeGvd(layer); }

  void showTsdf(const TsdfIntegratorBase::Config& config,
                const Layer<TsdfVoxel>& layer) {
    visualizeTsdf(layer, config.default_truncation_distance);
  }

  void showState(const Layer<GvdVoxel>& layer, size_t index) {
    Eigen::Vector2i resolution;
    resolution << depth_camera_width, depth_camera_height;
    visualize(layer,
              getPose(index),
              index,
              resolution,
              depth_camera_fov,
              depth_camera_max_distance,
              0.2,
              10.0);
  }
};

// Optional test to help with visualization
TEST_F(GvdVizFixture, DISABLED_VisualizeTestEsdf) {
  const float voxel_size = 0.1f;
  const int voxels_per_side = 16;

  TsdfIntegratorBase::Config tsdf_config;
  Layer<TsdfVoxel>::Ptr tsdf_layer(new Layer<TsdfVoxel>(voxel_size, voxels_per_side));
  SimpleTsdfIntegrator tsdf_integrator(tsdf_config, tsdf_layer.get());

  GvdIntegratorConfig gvd_config;
  gvd_config.min_distance_m = tsdf_config.default_truncation_distance;
  gvd_config.max_distance_m = 10.0;
  Layer<GvdVoxel>::Ptr gvd_layer(new Layer<GvdVoxel>(voxel_size, voxels_per_side));
  GvdIntegrator gvd_integrator(gvd_config, tsdf_layer, gvd_layer);

  for (size_t i = 0; i < num_poses; ++i) {
    updateTsdfIntegrator(tsdf_integrator, i);
    gvd_integrator.updateFromTsdfLayer(true);

    showState(*gvd_layer, i);
    showPointcloud(i);
    showTsdf(tsdf_config, *tsdf_layer);
    showGvd(*gvd_layer);
  }

  waitForVisualization();
  SUCCEED();
}

}  // namespace topology
}  // namespace kimera
