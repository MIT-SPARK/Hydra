#include <config_utilities/parsing/yaml.h>
#include <config_utilities/validation.h>
#include <gtest/gtest.h>
#include <hydra/reconstruction/mesh_integrator.h>
#include <hydra/reconstruction/volumetric_map.h>

namespace hydra {
namespace {
// Emit one triangle in each pass to exercise finalization across the pass barrier.
class PassIntegrator : public MeshIntegrator {
 public:
  using MeshIntegrator::MeshIntegrator;
  void meshBlockInterior(const BlockIndex& block,
                         const VoxelIndex& index,
                         VolumetricMap& map) const override {
    if (index != VoxelIndex::Zero()) {
      return;
    }
    auto& mesh = map.getMeshLayer().getBlock(block);
    mesh.resizeVertices(3);
    mesh.points = {{0, 0, 0}, {1, 0, 0}, {0, 1, 0}};
    mesh.faces = {{0, 1, 2}};
  }
  void meshBlockExterior(const BlockIndex& block,
                         const VoxelIndex& index,
                         VolumetricMap& map) const override {
    if (index != VoxelIndex(1, 0, 0)) {
      return;
    }
    auto& mesh = map.getMeshLayer().getBlock(block);
    mesh.resizeVertices(6);
    mesh.points[3] = {1.0e-6f, 0, 0};
    mesh.points[4] = {1, 1, 0};
    mesh.points[5] = {0, 1, 0};
    mesh.faces.push_back({3, 4, 5});
  }
};
}  // namespace

TEST(MeshIntegrator, DeduplicatesAfterBothPassesAndCanBeDisabled) {
  for (const auto tolerance : {0.0, 1.0e-5}) {
    VolumetricMap::Config map_config;
    map_config.voxels_per_side = 2;
    VolumetricMap map(map_config);
    const BlockIndex a(0, 0, 0), b(1, 0, 0);
    map.allocateBlock(a);
    map.allocateBlock(b);
    MeshIntegrator::Config config;
    config.integrator_threads = 2;
    config.vertex_merge_tolerance_m = tolerance;
    PassIntegrator integrator(config);
    integrator.generateMesh(map, false, true);
    for (const auto& mesh : map.getMeshLayer()) {
      EXPECT_EQ(mesh.numVertices(), tolerance > 0 ? 4u : 6u);
      EXPECT_EQ(mesh.numFaces(), 2u);
      EXPECT_TRUE(mesh.updated);
      EXPECT_FALSE(map.getTsdfLayer().getBlock(mesh.index).mesh_updated);
    }
    // Unchanged blocks retain their existing geometry on an incremental update.
    map.getMeshLayer().getBlock(b).points[0] = {9, 9, 9};
    map.getTsdfLayer().getBlock(a).mesh_updated = true;
    integrator.generateMesh(map, true, true);
    EXPECT_EQ(map.getMeshLayer().getBlock(b).pos(0), Eigen::Vector3f(9, 9, 9));
    EXPECT_EQ(map.getMeshLayer().getBlock(a).pos(0), Eigen::Vector3f::Zero());
  }
}

TEST(MeshIntegrator, MergeToleranceConfiguration) {
  auto config = config::fromYaml<MeshIntegrator::Config>(
      YAML::Load("integrator_threads: 1\nvertex_merge_tolerance_m: 0.00002"));
  EXPECT_DOUBLE_EQ(config.vertex_merge_tolerance_m, 2.0e-5);
  EXPECT_TRUE(config::isValid(config));
  config.vertex_merge_tolerance_m = -1;
  EXPECT_FALSE(config::isValid(config));
}
}  // namespace hydra
