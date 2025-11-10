/* -----------------------------------------------------------------------------
 * Copyright 2022 Massachusetts Institute of Technology.
 * All Rights Reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  1. Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *
 *  2. Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Research was sponsored by the United States Air Force Research Laboratory and
 * the United States Air Force Artificial Intelligence Accelerator and was
 * accomplished under Cooperative Agreement Number FA8750-19-2-1000. The views
 * and conclusions contained in this document are those of the authors and should
 * not be interpreted as representing the official policies, either expressed or
 * implied, of the United States Air Force or the U.S. Government. The U.S.
 * Government is authorized to reproduce and distribute reprints for Government
 * purposes notwithstanding any copyright notation herein.
 * -------------------------------------------------------------------------- */
#include <gtest/gtest.h>
#include <hydra/places/gvd_integrator.h>
#include <hydra/places/gvd_utilities.h>
#include <hydra/reconstruction/volumetric_map.h>

namespace hydra::places {
namespace {

void updateGvd(GvdIntegrator& integrator,
               const VolumetricMap& map,
               bool clear_updated,
               bool use_all_blocks = false) {
  integrator.updateFromTsdf(
      0, map.getTsdfLayer(), clear_updated, &map.getMeshLayer(), use_all_blocks);
  integrator.updateGvd(0);
}

class GvdIntegratorData {
 public:
  using Coords = std::array<size_t, 3>;
  struct Obs {
    float distance;
    float weight = 0.1f;
  };

  using ObservationCallback =
      std::function<std::optional<Obs>(const Coords&, const VolumetricMap::Config&)>;
  using CheckCallback =
      std::function<void(const Coords&, const GvdVoxel&, const VolumetricMap::Config&)>;

  GvdIntegratorData(float voxel_size = 0.1f,
                    size_t voxels_per_side = 4,
                    float truncation_distance = 0.1f);

  void setup(const ObservationCallback& callback);
  void check(const CheckCallback& callback);

  void setTsdf(const Coords& coords, float distance, float weight = 0.1f);
  void setTsdf(size_t x, size_t y, float distance, float weight = 0.1f);

  const TsdfVoxel& getTsdf(const Coords& coords) const;
  const GvdVoxel& getGvd(const Coords& coords) const;

  const VolumetricMap::Config map_config;
  VolumetricMap map;

  GvdIntegrator::Config gvd_config;
  GvdLayer::Ptr gvd_layer;

  TsdfBlock::Ptr tsdf_block;
  GvdBlock::Ptr gvd_block;
  MeshBlock::Ptr mesh_block;
};

VoxelIndex validateCoords(const GvdIntegratorData::Coords& coords,
                          size_t voxels_per_side) {
  CHECK_LT(coords[0], voxels_per_side);
  CHECK_LT(coords[1], voxels_per_side);
  CHECK_LT(coords[2], voxels_per_side);

  VoxelIndex v_index;
  v_index << coords[0], coords[1], coords[2];
  return v_index;
}

GvdIntegratorData::GvdIntegratorData(float voxel_size,
                                     size_t voxels_per_side,
                                     float truncation_distance)
    : map_config({voxel_size, voxels_per_side, truncation_distance}), map(map_config) {
  gvd_config.min_distance_m = truncation_distance;
  gvd_config.max_distance_m = 10.0;
  gvd_layer.reset(new GvdLayer(voxel_size, voxels_per_side));

  BlockIndex block_index = BlockIndex::Zero();
  tsdf_block = map.getTsdfLayer().allocateBlockPtr(block_index);
  mesh_block = map.getMeshLayer().allocateBlockPtr(block_index);
  gvd_block = gvd_layer->allocateBlockPtr(block_index);
  tsdf_block->setUpdated();
}

void GvdIntegratorData::setup(const ObservationCallback& callback) {
  for (size_t x = 0; x < map_config.voxels_per_side; ++x) {
    for (size_t y = 0; y < map_config.voxels_per_side; ++y) {
      for (size_t z = 0; z < map_config.voxels_per_side; ++z) {
        const Coords coords{x, y, z};
        const auto obs = callback(coords, map_config);
        if (!obs) {
          continue;
        }

        setTsdf(coords, obs->distance, obs->weight);
      }
    }
  }
}

void GvdIntegratorData::check(const CheckCallback& callback) {
  for (size_t x = 0; x < map_config.voxels_per_side; ++x) {
    for (size_t y = 0; y < map_config.voxels_per_side; ++y) {
      for (size_t z = 0; z < map_config.voxels_per_side; ++z) {
        const Coords coords{x, y, z};
        callback(coords, getGvd(coords), map_config);
      }
    }
  }
}

void GvdIntegratorData::setTsdf(size_t x, size_t y, float distance, float weight) {
  setTsdf({x, y, 0}, distance, weight);
}

void GvdIntegratorData::setTsdf(const Coords& coords, float distance, float weight) {
  const auto v_index = validateCoords(coords, map_config.voxels_per_side);
  auto& voxel = tsdf_block->getVoxel(v_index);
  voxel.distance = distance;
  voxel.weight = weight;
  if (std::abs(distance) < map_config.truncation_distance && weight > 0.0f) {
    const auto pos = tsdf_block->getVoxelPosition(v_index);
    mesh_block->points.push_back(pos);
  }
}

const GvdVoxel& GvdIntegratorData::getGvd(const Coords& coords) const {
  const auto v_index = validateCoords(coords, map_config.voxels_per_side);
  return gvd_block->getVoxel(v_index);
}

const TsdfVoxel& GvdIntegratorData::getTsdf(const Coords& coords) const {
  const auto v_index = validateCoords(coords, map_config.voxels_per_side);
  return tsdf_block->getVoxel(v_index);
}

std::string formatCoords(const GvdIntegratorData::Coords& coords) {
  const auto [x, y, z] = coords;
  std::stringstream ss;
  ss << " @ (" << x << ", " << y << ", " << z << ")";
  return ss.str();
}

bool coordsMatchParent(const GvdIntegratorData::Coords& coords,
                       const GlobalIndex& parent,
                       const std::vector<size_t>& indices) {
  bool matches = false;
  for (const auto& idx : indices) {
    matches |= static_cast<int64_t>(coords[idx]) == parent(idx);
  }

  return matches;
}

size_t absDiff(size_t lhs, size_t rhs) {
  return std::max(lhs, rhs) - std::min(lhs, rhs);
}

struct GvdSlice {
  using VoronoiCoords = std::vector<std::pair<size_t, size_t>>;

  GvdSlice(int rows, int cols) : distances(rows, cols), is_voronoi(rows, cols) {}

  static GvdSlice fromData(size_t rows, size_t cols, const GvdIntegratorData& data) {
    GvdSlice result(rows, cols);
    for (size_t r = 0; r < rows; ++r) {
      for (size_t c = 0; c < cols; ++c) {
        const auto& voxel = data.getGvd({c, r, 0});
        result.distances(r, c) = voxel.observed ? voxel.distance : -1.0;
        result.is_voronoi(r, c) = isVoronoi(voxel) ? 1.0 : 0.0;
      }
    }

    return result;
  }

  static GvdSlice fromExpected(const Eigen::MatrixXd distances,
                               const VoronoiCoords& voronoi_coords) {
    GvdSlice expected(distances.rows(), distances.cols());
    expected.distances = distances;
    expected.is_voronoi.setZero();
    for (const auto& [r, c] : voronoi_coords) {
      expected.is_voronoi(r, c) = 1.0;
    }

    return expected;
  }

  void check(const GvdSlice& other) const {
    for (int r = 0; r < is_voronoi.rows(); ++r) {
      for (int c = 0; c < is_voronoi.cols(); ++c) {
        EXPECT_EQ(is_voronoi(r, c), other.is_voronoi(r, c))
            << " @ (" << r << ", " << c << ")";
        EXPECT_NEAR(distances(r, c), other.distances(r, c), 1.0e-2)
            << " @ (" << r << ", " << c << ")";
      }
    }
  }

  Eigen::MatrixXd distances;
  Eigen::MatrixXd is_voronoi;
};

std::ostream& operator<<(std::ostream& out, const GvdSlice& result) {
  for (int r = 0; r < result.distances.rows(); ++r) {
    for (int c = 0; c < result.distances.cols(); ++c) {
      out << "+-------";
    }
    out << "+\n";

    out << "|";
    for (int c = 0; c < result.distances.cols(); ++c) {
      out << (result.is_voronoi(r, c) == 1.0 ? " @" : "  ");
      out << std::setw(5) << std::setprecision(2) << std::fixed << std::setfill(' ')
          << result.distances(r, c);
      out << "|";
    }
    out << "\n";
  }

  for (int c = 0; c < result.distances.cols(); ++c) {
    out << "+-------";
  }
  out << "+";
  return out;
}

}  // namespace

TEST(GvdIntegrator, OccupancyIntegration2DCorrect) {
  GvdIntegratorData data(1.0, 8, 0.1);
  data.setup([](const auto& coords, const auto& config) {
    const auto [x, y, z] = coords;
    if (z > 0 || y >= 4) {
      return GvdIntegratorData::Obs{0.0, 0.0};
    }

    return GvdIntegratorData::Obs{3.0f * config.voxel_size};
  });

  data.setTsdf(0, 0, 0.0);
  data.setTsdf(0, 1, 0.0);
  data.setTsdf(0, 2, 0.0);
  data.setTsdf(0, 3, 0.0);
  data.setTsdf(3, 2, 0.0);

  data.gvd_config.min_diff_m = 0.0;
  data.gvd_config.min_distance_m = 1.0;
  data.gvd_config.max_distance_m = 50.0;
  data.gvd_config.voronoi_config.min_distance_m = 1.0;
  data.gvd_config.voronoi_config.parent_l1_separation = 2.0;

  GvdIntegrator gvd_integrator(data.gvd_config, data.gvd_layer);
  updateGvd(gvd_integrator, data.map, false, true);

  {  // scope for expected lifetime
    // clang-format off
    Eigen::MatrixXd expected_distances(4, 8);
    expected_distances << 0.00, 1.00, 2.00, 2.00, 2.24, 2.83, 3.61, 4.47,
                          0.00, 1.00, 1.41, 1.00, 1.41, 2.24, 3.16, 4.12,
                          0.00, 1.00, 1.00, 0.00, 1.00, 2.00, 3.00, 4.00,
                          0.00, 1.00, 1.41, 1.00, 1.41, 2.24, 3.16, 4.12;
    // clang-format on

    const auto expected = GvdSlice::fromExpected(expected_distances, {{0, 2}, {1, 2}});
    const auto result = GvdSlice::fromData(4, 8, data);
    VLOG(1) << "Result:\n" << result;
    VLOG(1) << "Expected:\n" << expected;
    expected.check(result);
  }

  VLOG(1) << "Perturbing GVD: (3, 2) -> 10.0, (7, 2) -> 0.0";

  // raise the middle obstacle and lower one on the side
  // reset surface flags for previous surfaces
  data.mesh_block->points.clear();
  data.setTsdf(3, 2, 10.0);
  data.setTsdf(7, 2, 0.0);
  data.setTsdf(0, 0, 0.0, 1.0);
  data.setTsdf(0, 1, 0.0, 1.0);
  data.setTsdf(0, 2, 0.0, 1.0);
  data.setTsdf(0, 3, 0.0, 1.0);

  updateGvd(gvd_integrator, data.map, false, true);

  {  // scope for expected lifetime
    // clang-format off
    Eigen::MatrixXd expected_distances(4, 8);
    expected_distances << 0.00, 1.00, 2.00, 3.00, 3.61, 2.83, 2.24, 2.00,
                          0.00, 1.00, 2.00, 3.00, 3.16, 2.24, 1.41, 1.00,
                          0.00, 1.00, 2.00, 3.00, 3.00, 2.00, 1.00, 0.00,
                          0.00, 1.00, 2.00, 3.00, 3.16, 2.24, 1.41, 1.00;
    // clang-format on

    const auto expected = GvdSlice::fromExpected(
        expected_distances, {{0, 4}, {1, 3}, {1, 4}, {2, 3}, {2, 4}, {3, 3}, {3, 4}});
    const auto result = GvdSlice::fromData(4, 8, data);
    VLOG(1) << "Result:\n" << result;
    VLOG(1) << "Expected:\n" << expected;
    expected.check(result);
  }
}

TEST(GvdIntegrator, NegativeIntegration2DCorrect) {
  GvdIntegratorData data(1.0, 8, 0.1);
  data.setup([](const auto& coords, const auto& config) {
    const auto [x, y, z] = coords;
    if (z > 0) {
      return GvdIntegratorData::Obs{0.0, 0.0};
    }

    // exterior border is set to -truncation_distance
    if (x == 0 || y == 0 || x == 7 || y == 7) {
      return GvdIntegratorData::Obs{-2.0f * config.voxel_size};
    }

    // interior border is set to half truncation distance
    if (x == 1 || y == 1 || x == 6 || y == 6) {
      bool is_corner = x == y || (x == 1 && y == 6) || (x == 6 && y == 1);
      GvdIntegratorData::Obs obs{-config.voxel_size};
      if (is_corner) {
        obs.distance *= std::sqrt(2.0);
      }

      return obs;
    }

    // set next ring of distances to 0
    if (x == 2 || y == 2 || x == 5 || y == 5) {
      return GvdIntegratorData::Obs{0.0};
    }

    // set inner four voxels to 1.0
    return GvdIntegratorData::Obs{1.0};
  });

  data.gvd_config.min_diff_m = 0.0;
  data.gvd_config.min_distance_m = 1.0;
  data.gvd_config.max_distance_m = 50.0;
  data.gvd_config.voronoi_config.min_distance_m = 1.0;
  data.gvd_config.voronoi_config.parent_l1_separation = 2.0;
  data.gvd_config.positive_distance_only = false;

  GvdIntegrator gvd_integrator(data.gvd_config, data.gvd_layer);
  updateGvd(gvd_integrator, data.map, false, true);

  Eigen::MatrixXd expected_distances(8, 8);
  // clang-format off
  expected_distances << -2.83, -2.24, -2.00, -2.00, -2.00, -2.00, -2.24, -2.83,
                        -2.24, -1.41, -1.00, -1.00, -1.00, -1.00, -1.41, -2.24,
                        -2.00, -1.00,  0.00,  0.00,  0.00,  0.00, -1.00, -2.00,
                        -2.00, -1.00,  0.00,  1.00,  1.00,  0.00, -1.00, -2.00,
                        -2.00, -1.00,  0.00,  1.00,  1.00,  0.00, -1.00, -2.00,
                        -2.00, -1.00,  0.00,  0.00,  0.00,  0.00, -1.00, -2.00,
                        -2.24, -1.41, -1.00, -1.00, -1.00, -1.00, -1.41, -2.24,
                        -2.83, -2.24, -2.00, -2.00, -2.00, -2.00, -2.24, -2.83;
  // clang-format on

  const auto expected = GvdSlice::fromExpected(expected_distances, {});
  const auto result = GvdSlice::fromData(8, 8, data);
  VLOG(1) << "Result:\n" << result;
  VLOG(1) << "Expected:\n" << expected;
  expected.check(result);
}

TEST(GvdIntegrator, PlaneCorrect) {
  GvdIntegratorData data;
  data.setup([](const auto& coords, const auto& config) {
    const auto [x, y, z] = coords;
    GvdIntegratorData::Obs obs;
    obs.distance = x == 0 ? -0.05 : config.truncation_distance;
    return obs;
  });

  GvdIntegrator gvd_integrator(data.gvd_config, data.gvd_layer);
  updateGvd(gvd_integrator, data.map, true);

  data.check([](const auto& coords, const auto& voxel, const auto& config) {
    const auto [x, y, z] = coords;
    double expected_distance = x == 0 ? -0.05 : x * config.voxel_size;
    EXPECT_NEAR(expected_distance, voxel.distance, 1.0e-6);
    EXPECT_TRUE(voxel.on_surface || voxel.has_parent);
    EXPECT_FALSE(isVoronoi(voxel)) << voxel << formatCoords(coords);
  });
}

TEST(GvdIntegrator, LCorrect) {
  GvdIntegratorData data;
  data.setup([](const auto& coords, const auto& config) {
    const auto [x, y, z] = coords;
    GvdIntegratorData::Obs obs;
    const bool is_edge = (x == 0) || (y == 0);
    obs.distance = is_edge ? -0.05 : config.truncation_distance;
    return obs;
  });

  GvdIntegrator gvd_integrator(data.gvd_config, data.gvd_layer);
  updateGvd(gvd_integrator, data.map, true);
  data.check([](const auto& coords, const auto& voxel, const auto& config) {
    const auto [x, y, z] = coords;

    // 8-connected diagonal plane should be voronoi
    const bool is_edge = (x == 0) || (y == 0);
    double expected_distance = is_edge ? -0.05 : std::min(x, y) * config.voxel_size;
    const bool expected_voronoi = absDiff(x, y) <= 1 && x >= 2 && y >= 2;

    EXPECT_NEAR(expected_distance, voxel.distance, 1.0e-6);
    EXPECT_TRUE(!isVoronoi(voxel) || !voxel.fixed);
    EXPECT_TRUE(voxel.on_surface || voxel.has_parent);
    EXPECT_EQ(expected_voronoi, isVoronoi(voxel)) << voxel << formatCoords(coords);
  });
}

TEST(GvdIntegrator, LargeLCorrect) {
  GvdIntegratorData data(0.1, 8);
  data.setup([](const auto& coords, const auto& config) {
    const auto [x, y, z] = coords;
    GvdIntegratorData::Obs obs;
    const bool is_edge = (x == 0) || (y == 0);
    obs.distance = is_edge ? -0.05 : config.truncation_distance;
    return obs;
  });

  GvdIntegrator gvd_integrator(data.gvd_config, data.gvd_layer);
  updateGvd(gvd_integrator, data.map, true);
  data.check([](const auto& coords, const auto& voxel, const auto& config) {
    const auto [x, y, z] = coords;

    // 8-connected diagonal plane should be voronoi
    const bool is_edge = (x == 0) || (y == 0);
    double expected_distance = is_edge ? -0.05 : std::min(x, y) * config.voxel_size;
    const bool expected_voronoi = absDiff(x, y) <= 1 && x >= 2 && y >= 2;

    EXPECT_NEAR(expected_distance, voxel.distance, 1.0e-6);
    EXPECT_TRUE(!isVoronoi(voxel) || !voxel.fixed);
    EXPECT_TRUE(voxel.on_surface || voxel.has_parent);
    EXPECT_EQ(expected_voronoi, isVoronoi(voxel)) << voxel << formatCoords(coords);
  });
}

TEST(GvdIntegrator, CornerCorrect) {
  GvdIntegratorData data;
  data.setup([](const auto& coords, const auto& config) {
    const auto [x, y, z] = coords;
    GvdIntegratorData::Obs obs;
    const bool is_edge = (x == 0) || (y == 0) || (z == 0);
    obs.distance = is_edge ? -0.05 : config.truncation_distance;
    return obs;
  });

  GvdIntegrator gvd_integrator(data.gvd_config, data.gvd_layer);
  updateGvd(gvd_integrator, data.map, true);
  data.check([](const auto& coords, const auto& voxel, const auto& config) {
    const auto [x, y, z] = coords;

    // upper 2x2 should all be voronoi
    const bool is_edge = (x == 0) || (y == 0) || (z == 0);
    const auto min_coord = std::min(x, std::min(y, z));
    double expected_distance = is_edge ? -0.05 : min_coord * config.voxel_size;
    const bool expected_voronoi = x >= 2 && y >= 2 && z >= 2;

    EXPECT_NEAR(expected_distance, voxel.distance, 1.0e-6);
    EXPECT_TRUE(!isVoronoi(voxel) || !voxel.fixed);
    EXPECT_TRUE(voxel.on_surface || voxel.has_parent);
    EXPECT_EQ(expected_voronoi, isVoronoi(voxel)) << voxel << formatCoords(coords);
  });
}

TEST(GvdIntegrator, RaiseCorrectForSurface) {
  GvdIntegratorData data;
  data.setup([](const auto& coords, const auto& config) {
    GvdIntegratorData::Obs obs;
    const auto [x, y, z] = coords;
    const bool is_edge = (x == 0);
    obs.distance = is_edge ? -0.05 : config.truncation_distance;
    return obs;
  });

  data.gvd_config.min_diff_m = 0.03;
  GvdIntegrator gvd_integrator(data.gvd_config, data.gvd_layer);
  updateGvd(gvd_integrator, data.map, true);

  // trigger a lower wavefront
  data.setTsdf({0, 2, 0}, -0.01);
  // flip value to be raised
  data.setTsdf({0, 2, 2}, -0.09);

  updateGvd(gvd_integrator, data.map, true, true);
  EXPECT_TRUE(data.getGvd({0, 2, 2}).on_surface);

  data.check([&](const auto& coords, const auto& voxel, const auto& config) {
    const auto [x, y, z] = coords;
    const auto& tvoxel = data.getTsdf(coords);

    const bool is_edge = x == 0;
    double expected_distance = is_edge ? tvoxel.distance : x * config.voxel_size;
    EXPECT_NEAR(expected_distance, voxel.distance, 1.0e-6) << formatCoords(coords);
    EXPECT_TRUE(!isVoronoi(voxel) || !voxel.fixed);
    EXPECT_TRUE(voxel.on_surface || voxel.has_parent);
    EXPECT_FALSE(isVoronoi(voxel));
  });
}

TEST(GvdIntegrator, RaiseCorrectForTsdfSurface) {
  GvdIntegratorData data;
  data.setup([](const auto& coords, const auto& config) {
    GvdIntegratorData::Obs obs;
    const auto [x, y, z] = coords;
    const bool is_edge = (x == 0);
    obs.distance = is_edge ? -0.05 : config.truncation_distance;
    return obs;
  });

  data.gvd_config.min_diff_m = 0.03;
  data.gvd_config.surface_threshold_inflation = 1.1;
  data.gvd_config.use_tsdf_for_surface = true;
  GvdIntegrator gvd_integrator(data.gvd_config, data.gvd_layer);
  updateGvd(gvd_integrator, data.map, true);

  // trigger a lower wavefront
  data.setTsdf({0, 2, 0}, -0.01);
  // flip value to be raised
  data.setTsdf({0, 2, 2}, -0.09);

  updateGvd(gvd_integrator, data.map, true, true);
  EXPECT_TRUE(data.getGvd({0, 2, 2}).on_surface);

  data.check([&](const auto& coords, const auto& voxel, const auto& config) {
    const auto [x, y, z] = coords;
    const auto& tvoxel = data.getTsdf(coords);

    const bool is_edge = x == 0;
    double expected_distance = is_edge ? tvoxel.distance : x * config.voxel_size;
    EXPECT_NEAR(expected_distance, voxel.distance, 1.0e-6) << formatCoords(coords);
    EXPECT_TRUE(!isVoronoi(voxel) || !voxel.fixed);
    EXPECT_TRUE(voxel.on_surface || voxel.has_parent);
    EXPECT_FALSE(isVoronoi(voxel));
  });
}

TEST(GvdIntegrator, FrontierDistanceCorrect) {
  GvdIntegratorData data;
  data.setup([](const auto& coords,
                const auto& config) -> std::optional<GvdIntegratorData::Obs> {
    const auto [x, y, z] = coords;
    if (x == 0) {
      return std::nullopt;  // force unobserved voxels
    }

    GvdIntegratorData::Obs obs;
    obs.distance = config.truncation_distance;
    return obs;
  });

  data.gvd_config.integrate_frontiers = true;
  GvdIntegrator gvd_integrator(data.gvd_config, data.gvd_layer);
  updateGvd(gvd_integrator, data.map, true);

  data.check([&](const auto& coords, const auto& voxel, const auto& config) {
    const auto [x, y, z] = coords;
    const auto& tvoxel = data.getTsdf(coords);

    const bool is_edge = x == 0;
    double expected_distance = is_edge ? tvoxel.distance : x * config.voxel_size;
    EXPECT_NEAR(expected_distance, voxel.distance, 1.0e-6) << formatCoords(coords);
    EXPECT_TRUE(!isVoronoi(voxel) || !voxel.fixed);
    EXPECT_FALSE(isVoronoi(voxel));
  });
}

TEST(GvdIntegrator, ParentsCorrect) {
  GvdIntegratorData data(0.1, 8, 0.2);
  data.setup([](const auto& coords, const auto& config) {
    const auto [x, y, z] = coords;
    GvdIntegratorData::Obs obs;

    // creates a corner at the edge of the block
    const bool is_edge = (x == 0) || (y == 0) || (z == 0);
    obs.distance = is_edge ? -0.05 : config.truncation_distance;
    return obs;
  });

  GvdIntegrator gvd_integrator(data.gvd_config, data.gvd_layer);
  updateGvd(gvd_integrator, data.map, true);

  data.check([&](const auto& coords, const auto& voxel, const auto&) {
    const auto [x, y, z] = coords;
    if (x == 0 || y == 0 || z == 0) {
      // marching cubes can't handle sharp corners, so any "edge" (two zeros) or
      // "corner" (three zeros) voxel isn't on the surface
      int num_zeros = (x == 0 ? 1 : 0) + (y == 0 ? 1 : 0) + (z == 0 ? 1 : 0);
      EXPECT_TRUE(num_zeros >= 2 || voxel.on_surface) << voxel << formatCoords(coords);
      return;
    }

    // in general, it's hard to determine what tie-breaking rules are correct
    // (it depends on wavefront traversal order). We allow for multiple different
    // parents instead of trying to predict the wavefront traversal order
    if (x == y && x == z) {
      // at least two parent coordinates will be equal, and the other will be 0
      // this means that the product of the coordinates will be 0
      // the other two checks follow from x + ? = 2 * x <-> ? = x
      uint64_t total = voxel.parent[0] + voxel.parent[1] + voxel.parent[2];
      uint64_t product = voxel.parent[0] * voxel.parent[1] * voxel.parent[2];
      EXPECT_EQ(0u, product);
      EXPECT_EQ(2u * static_cast<uint64_t>(x), total);
      EXPECT_TRUE(coordsMatchParent(coords, voxel.parent, {0, 1, 2}));
    } else if (x == y && z > x) {
      EXPECT_EQ(z, voxel.parent[2]);
      EXPECT_TRUE(voxel.parent[0] == 0u || voxel.parent[1] == 0u);
      EXPECT_TRUE(coordsMatchParent(coords, voxel.parent, {0, 1}));
    } else if (x == z && y > x) {
      EXPECT_EQ(y, voxel.parent[1]);
      EXPECT_TRUE(voxel.parent[0] == 0u || voxel.parent[2] == 0u);
      EXPECT_TRUE(coordsMatchParent(coords, voxel.parent, {0, 2}));
    } else if (y == z && x > z) {
      EXPECT_EQ(x, voxel.parent[0]);
      EXPECT_TRUE(voxel.parent[1] == 0u || voxel.parent[2] == 0u);
      EXPECT_TRUE(coordsMatchParent(coords, voxel.parent, {1, 2}));
    } else {
      GlobalIndex expected_parent;
      if (x < y && x < z) {
        expected_parent << 0, y, z;
      } else if (y < x && y < z) {
        expected_parent << x, 0, z;
      } else {
        expected_parent << x, y, 0;
      }

      EXPECT_EQ(expected_parent, voxel.parent)
          << voxel << formatCoords(coords)
          << ",  expected parent: " << expected_parent.transpose();
    }
  });
}

}  // namespace hydra::places
