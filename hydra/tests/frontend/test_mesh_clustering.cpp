#include <config_utilities/validation.h>
#include <gtest/gtest.h>
#include <hydra/frontend/mesh_clustering.h>
#include <hydra/frontend/mesh_delta_clustering.h>
#include <kimera_pgmo/mesh_delta.h>

#include <limits>
#include <numeric>
#include <random>

namespace hydra {
namespace {
using clustering::Clusters;

spark_dsg::Mesh meshOf(const std::vector<Eigen::Vector3f>& points) {
  spark_dsg::Mesh mesh;
  mesh.resizeVertices(points.size());
  for (size_t i = 0; i < points.size(); ++i) {
    mesh.setPos(i, points[i]);
  }
  return mesh;
}

Clusters pclClusters(const clustering::VoxelClusteringConfig& config,
                     const spark_dsg::Mesh& mesh,
                     const std::vector<size_t>& indices) {
  kimera_pgmo::MeshDelta delta({});
  for (size_t i = 0; i < mesh.numVertices(); ++i) {
    delta.addVertex(mesh.pos(i), {});
  }
  const clustering::ClusteringConfig pcl_config{
      config.cluster_tolerance,
      config.min_cluster_size,
      config.max_cluster_size ? config.max_cluster_size : indices.size()};
  auto clusters = clustering::findClusters(pcl_config, delta, indices);
  for (auto& cluster : clusters) {
    std::sort(cluster.begin(), cluster.end());
  }
  std::sort(clusters.begin(), clusters.end());
  return clusters;
}

void checkPclParity(const spark_dsg::Mesh& mesh,
                    const std::vector<size_t>& indices,
                    clustering::VoxelClusteringConfig config) {
  const auto expected = pclClusters(config, mesh, indices);
  EXPECT_EQ(clustering::findClusters(config, mesh, indices), expected);
}
}  // namespace

TEST(MeshClustering, RandomizedPclParityAndSubsetIndices) {
  std::mt19937 random(42);
  std::uniform_real_distribution<float> distribution(-0.3f, 0.3f);
  for (size_t trial = 0; trial < 10; ++trial) {
    std::vector<Eigen::Vector3f> points;
    for (size_t i = 0; i < 600; ++i) {
      points.emplace_back(2.0f * (i % 5) + distribution(random),
                          distribution(random),
                          distribution(random));
    }
    const auto mesh = meshOf(points);
    std::vector<size_t> indices;
    for (size_t i = 0; i < points.size(); i += 2) {
      indices.push_back(i);
    }
    std::shuffle(indices.begin(), indices.end(), random);
    clustering::VoxelClusteringConfig config;
    config.min_cluster_size = 1;
    config.cluster_tolerance = 0.15;
    checkPclParity(mesh, indices, config);
  }
}

TEST(MeshClustering, StrictRadiusDuplicatesAndNegativeCells) {
  const auto below = std::nextafter(0.25f, 0.0f);
  for (const auto distance : {below, 0.25f, std::nextafter(0.25f, 1.0f)}) {
    const auto mesh = meshOf({{-distance, 0, 0}, {0, 0, 0}, {0, 0, 0}});
    clustering::VoxelClusteringConfig config;
    config.min_cluster_size = 1;
    checkPclParity(mesh, {0, 1, 2}, config);
    const auto result = clustering::findClusters(config, mesh, {0, 1, 2});
    EXPECT_EQ(result.size(), distance < 0.25f ? 1u : 2u);
  }
}

TEST(MeshClustering, FullTraversalBeforeSizeFiltering) {
  const auto mesh = meshOf({{0, 0, 0},
                            {0.1f, 0, 0},
                            {0.2f, 0, 0},
                            {0.3f, 0, 0},
                            {2, 0, 0},
                            {2.1f, 0, 0},
                            {4, 0, 0}});
  clustering::VoxelClusteringConfig config;
  config.min_cluster_size = 2;
  config.max_cluster_size = 3;
  checkPclParity(mesh, {0, 1, 2, 3, 4, 5, 6}, config);
  EXPECT_EQ(clustering::findClusters(config, mesh, {0, 1, 2, 3, 4, 5, 6}),
            (Clusters{{4, 5}}));

  config.max_cluster_size = 0;
  checkPclParity(mesh, {0, 1, 2, 3, 4, 5, 6}, config);
  EXPECT_EQ(clustering::findClusters(config, mesh, {0, 1, 2, 3, 4, 5, 6}),
            (Clusters{{0, 1, 2, 3}, {4, 5}}));
}

TEST(MeshClustering, SizeLimitValidation) {
  clustering::VoxelClusteringConfig config;
  EXPECT_EQ(config.max_cluster_size, 0u);
  EXPECT_TRUE(config::isValid(config));
  config.min_cluster_size = 2;
  EXPECT_TRUE(config::isValid(config));
  config.max_cluster_size = 1;
  EXPECT_FALSE(config::isValid(config));
  EXPECT_THROW(clustering::findClusters(config, meshOf({}), {}), std::invalid_argument);
  config.max_cluster_size = 2;
  EXPECT_TRUE(config::isValid(config));
}

TEST(MeshClustering, EmptyAndInvalidInput) {
  const auto mesh = meshOf({{std::numeric_limits<float>::quiet_NaN(), 0, 0}});
  clustering::VoxelClusteringConfig config;
  EXPECT_TRUE(clustering::findClusters(config, mesh, {}).empty());
  EXPECT_THROW(clustering::findClusters(config, mesh, {0}), std::invalid_argument);
  EXPECT_THROW(clustering::findClusters(config, mesh, {1}), std::invalid_argument);
  config.cluster_tolerance = 0;
  EXPECT_THROW(clustering::findClusters(config, mesh, {}), std::invalid_argument);
}

}  // namespace hydra
