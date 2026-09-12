#include "hydra/frontend/mesh_clustering.h"

#include <config_utilities/config.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <unordered_map>

namespace hydra::clustering {
namespace {
using Cell = std::array<int64_t, 3>;

struct CellHash {
  size_t operator()(const Cell& key) const {
    size_t result = 0;
    for (const auto value : key) {
      result ^=
          std::hash<int64_t>{}(value) + 0x9e3779b9 + (result << 6) + (result >> 2);
    }
    return result;
  }
};

Cell cellFor(const Eigen::Vector3f& p, double resolution) {
  Cell cell;
  for (size_t i = 0; i < 3; ++i) {
    const auto value = std::floor(static_cast<double>(p[i]) / resolution);
    // Reserve room for neighbor offsets and reject conversions outside int64.
    if (value <= static_cast<double>(std::numeric_limits<int64_t>::min()) ||
        value >= static_cast<double>(std::numeric_limits<int64_t>::max())) {
      throw std::out_of_range("Clustering cell coordinate exceeds int64 range");
    }
    cell[i] = static_cast<int64_t>(value);
  }
  return cell;
}

// Use the same float arithmetic as PointXYZ / PCL's FLANN search.
float distanceSquared(const Eigen::Vector3f& a, const Eigen::Vector3f& b) {
  const float x = a.x() - b.x();
  const float y = a.y() - b.y();
  const float z = a.z() - b.z();
  return x * x + y * y + z * z;
}

template <typename Search>
Clusters extract(const VoxelClusteringConfig& config,
                 const std::vector<size_t>& indices,
                 const Search& search) {
  std::vector<uint8_t> visited(indices.size(), 0);
  std::vector<size_t> queue;
  Clusters clusters;
  for (size_t seed = 0; seed < indices.size(); ++seed) {
    if (visited[seed]) {
      continue;
    }

    visited[seed] = 1;
    queue.clear();
    queue.push_back(seed);
    for (size_t cursor = 0; cursor < queue.size(); ++cursor) {
      search(queue[cursor], visited, [&](size_t next) {
        if (!visited[next]) {
          visited[next] = 1;
          queue.push_back(next);
        }
      });
    }

    // Complete traversal even when the component is too large.
    if (queue.size() < config.min_cluster_size ||
        (config.max_cluster_size && queue.size() > config.max_cluster_size)) {
      continue;
    }

    auto& component = clusters.emplace_back();
    component.reserve(queue.size());
    for (const auto i : queue) {
      component.push_back(indices[i]);
    }
    std::sort(component.begin(), component.end());
  }
  // Stable component order also makes object association independent of traversal.
  std::sort(clusters.begin(), clusters.end());
  return clusters;
}

}  // namespace

void declare_config(VoxelClusteringConfig& config) {
  using namespace config;
  name("VoxelClusteringConfig");
  field(config.cluster_tolerance, "cluster_tolerance");
  field(config.min_cluster_size, "min_cluster_size");
  field(config.max_cluster_size, "max_cluster_size");
  check(config.cluster_tolerance, GT, 0.0, "cluster_tolerance");
  check(config.min_cluster_size, GT, 0, "min_cluster_size");
  checkCondition(config.max_cluster_size == 0 ||
                     config.max_cluster_size >= config.min_cluster_size,
                 "max_cluster_size is zero or at least min_cluster_size");
}

Clusters findClusters(const VoxelClusteringConfig& config,
                      const spark_dsg::Mesh& mesh,
                      const std::vector<size_t>& indices) {
  const float radius_squared =
      static_cast<float>(config.cluster_tolerance * config.cluster_tolerance);
  if (!std::isfinite(radius_squared) || radius_squared <= 0 ||
      config.cluster_tolerance <= 0 || config.min_cluster_size == 0 ||
      (config.max_cluster_size && config.max_cluster_size < config.min_cluster_size)) {
    throw std::invalid_argument("Invalid mesh clustering parameters");
  }
  for (const auto index : indices) {
    if (index >= mesh.numVertices() || !mesh.pos(index).allFinite()) {
      throw std::invalid_argument("Invalid mesh clustering sample");
    }
  }
  Clusters clusters;
  if (indices.empty()) {
    return clusters;
  }

  std::unordered_map<Cell, std::vector<size_t>, CellHash> grid;
  grid.reserve(indices.size());
  for (size_t i = 0; i < indices.size(); ++i) {
    grid[cellFor(mesh.pos(indices[i]), config.cluster_tolerance)].push_back(i);
  }
  clusters =
      extract(config, indices, [&](size_t i, const auto& visited, const auto& visit) {
        const auto& point = mesh.pos(indices[i]);
        const auto cell = cellFor(point, config.cluster_tolerance);
        for (int x = -1; x <= 1; ++x) {
          for (int y = -1; y <= 1; ++y) {
            for (int z = -1; z <= 1; ++z) {
              const auto it = grid.find({cell[0] + x, cell[1] + y, cell[2] + z});
              if (it == grid.end()) {
                continue;
              }
              for (const auto next : it->second) {
                if (!visited[next] &&
                    distanceSquared(point, mesh.pos(indices[next])) < radius_squared) {
                  visit(next);
                }
              }
            }
          }
        }
      });
  return clusters;
}

}  // namespace hydra::clustering
