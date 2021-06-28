#include <kimera_topology_test/test_fixtures.h>
#include <voxblox/core/voxel.h>
#include <voxblox/integrator/esdf_integrator.h>
#include <voxblox/integrator/tsdf_integrator.h>
#include <voxblox/simulation/simulation_world.h>

using namespace voxblox;

namespace kimera {
namespace topology {
namespace test_helpers {

void EsdfTestFixture::SetUp() { setupWorld(); }

void EsdfTestFixture::updateTsdfIntegrator(TsdfIntegratorBase& integrator,
                                           size_t index) {
  Transformation world_T_camera = getPose(index);
  VLOG(10) << "world_T_camera: " << std::endl << world_T_camera;

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

  VLOG(10) << "camera pointcloud: " << camera_pointcloud.size() << " points";

  Pointcloud world_pointcloud;
  transformPointcloud(world_T_camera.inverse(), camera_pointcloud, &world_pointcloud);
  VLOG(10) << "world pointcloud: " << world_pointcloud.size() << " points";

  integrator.integratePointCloud(world_T_camera, world_pointcloud, colors);
}

Point EsdfTestFixture::getCenter() const { return Point(0.0, 0.0, 2.0); }

void EsdfTestFixture::setupWorld() {
  world.setBounds(Point(-5.0, -5.0, -1.0), Point(5.0, 5.0, 6.0));
  world.addObject(std::make_unique<Cylinder>(getCenter(), 2.0, 4.0, Color::Red()));
  world.addGroundLevel(0.0);
}

Transformation EsdfTestFixture::getPose(size_t index) const {
  double angle =
      2.0 * M_PI * (static_cast<double>(index) / static_cast<double>(num_angles));
  Point position(
      pose_radius * std::sin(angle), pose_radius * std::cos(angle), pose_height);

  Point direction = getCenter() - position;
  double yaw = std::atan2(direction.y(), direction.x());

  Quaternion rotation =
      Quaternion(Eigen::AngleAxis<float>(yaw, Point::UnitZ()) *
                 Eigen::AngleAxis<float>(camera_pitch, Point::UnitY()));

  return Transformation(rotation, position);
}

GvdTestFixture::GvdTestFixture() : EsdfTestFixture() {
  depth_camera_max_distance = 2.0;
  depth_camera_fov = M_PI / 7.0;
  pose_radius = 2.5;
  pose_height = 1.2;
  num_angles = 50;
  num_poses = 2;
  camera_pitch = 0.6;
  depth_camera_width = 240;
  depth_camera_height = 320;
}

Transformation GvdTestFixture::getPose(size_t) const {
  return EsdfTestFixture::getPose(0);
}

void GvdTestFixture::setupWorld() {
  world.setBounds(Point(-5.0, -5.0, -1.0), Point(5.0, 5.0, 6.0));
  world.addObject(
      std::make_unique<Cube>(Point(0.0, 0.0, 0.0), Point(1.0, 1.0, 2.0), Color::Red()));
  world.addGroundLevel(0.0);
}

void SingleBlockTestFixture::setTsdfVoxel(int x,
                                          int y,
                                          int z,
                                          float distance,
                                          float weight) {
  CHECK_LT(x, voxels_per_side);
  CHECK_LT(y, voxels_per_side);
  CHECK_LT(z, voxels_per_side);

  VoxelIndex v_index;
  v_index << x, y, z;

  auto& voxel = tsdf_block->getVoxelByVoxelIndex(v_index);
  voxel.distance = distance;
  voxel.weight = weight;
}

const GvdVoxel& SingleBlockTestFixture::getGvdVoxel(int x, int y, int z) {
  CHECK_LT(x, voxels_per_side);
  CHECK_LT(y, voxels_per_side);
  CHECK_LT(z, voxels_per_side);

  VoxelIndex v_index;
  v_index << x, y, z;

  return gvd_block->getVoxelByVoxelIndex(v_index);
}

void SingleBlockTestFixture::SetUp() {
  gvd_config.min_distance_m = truncation_distance;
  gvd_config.max_distance_m = 10.0;

  tsdf_layer.reset(new Layer<TsdfVoxel>(voxel_size, voxels_per_side));
  gvd_layer.reset(new Layer<GvdVoxel>(voxel_size, voxels_per_side));

  BlockIndex block_index = BlockIndex::Zero();
  tsdf_block = tsdf_layer->allocateBlockPtrByIndex(block_index);
  gvd_block = gvd_layer->allocateBlockPtrByIndex(block_index);
  tsdf_block->updated().set();

  for (int x = 0; x < voxels_per_side; ++x) {
    for (int y = 0; y < voxels_per_side; ++y) {
      for (int z = 0; z < voxels_per_side; ++z) {
        const bool is_edge = (x == 0) || (y == 0) || (z == 0);
        setTsdfVoxel(x, y, z, is_edge ? 0.0 : truncation_distance);
      }
    }
  }
}

}  // namespace test_helpers
}  // namespace topology
}  // namespace kimera
