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
#include <hydra_topology/gvd_integrator.h>
#include <hydra_topology/gvd_utilities.h>

#include "hydra_topology_test/test_fixtures.h"

namespace hydra {
namespace topology {

using test_helpers::LargeSingleBlockTestFixture;
using test_helpers::SingleBlockTestFixture;
using test_helpers::TestFixture2d;
using voxblox::VoxelIndex;

class ParentTestFixture : public SingleBlockTestFixture {
 public:
  ParentTestFixture() : SingleBlockTestFixture() {}
  virtual ~ParentTestFixture() = default;

  virtual void SetUp() override {
    voxels_per_side = 8;
    truncation_distance = 0.2;
    SingleBlockTestFixture::SetUp();

    for (int r = 1; r < voxels_per_side; ++r) {
      for (int c = 1; c < voxels_per_side; ++c) {
        setTsdfVoxel(1, r, c, 0.1);
        setTsdfVoxel(r, 1, c, 0.1);
        setTsdfVoxel(r, c, 1, 0.1);
      }
    }
  }
};

struct GvdResult {
  GvdResult(int rows, int cols) {
    distances = Eigen::MatrixXd(rows, cols);
    is_voronoi = Eigen::MatrixXd(rows, cols);
  }

  Eigen::MatrixXd distances;
  Eigen::MatrixXd is_voronoi;
};

std::ostream& operator<<(std::ostream& out, const GvdResult& result) {
  out << std::endl;
  for (int r = 0; r < result.distances.rows(); ++r) {
    for (int c = 0; c < result.distances.cols(); ++c) {
      out << "+-------";
    }
    out << "+" << std::endl;

    out << "|";
    for (int c = 0; c < result.distances.cols(); ++c) {
      out << (result.is_voronoi(r, c) == 1.0 ? " @" : "  ");
      out << std::setw(5) << std::setprecision(2) << std::fixed << std::setfill(' ')
          << result.distances(r, c);
      out << "|";
    }
    out << std::endl;
  }

  for (int c = 0; c < result.distances.cols(); ++c) {
    out << "+-------";
  }
  out << "+";

  out << std::endl;

  return out;
}

TEST_F(TestFixture2d, OccupancyIntegrationCorrect) {
  for (int x = 0; x < voxels_per_side; ++x) {
    for (int y = 0; y < 4; ++y) {
      setTsdfVoxel(x, y, 3.0 * voxel_size);
    }
  }

  setTsdfVoxel(0, 0, 0.0);
  setTsdfVoxel(0, 1, 0.0);
  setTsdfVoxel(0, 2, 0.0);
  setTsdfVoxel(0, 3, 0.0);
  setTsdfVoxel(3, 2, 0.0);

  gvd_config.min_diff_m = 0.0;
  gvd_config.min_distance_m = voxel_size;
  gvd_config.max_distance_m = 50.0;
  gvd_config.parent_derived_distance = true;
  gvd_config.voronoi_config.min_distance_m = 1.0;
  gvd_config.voronoi_config.parent_l1_separation = 2.0;
  gvd_config.extract_graph = false;

  GvdIntegrator gvd_integrator(gvd_config, tsdf_layer.get(), gvd_layer, mesh_layer);
  gvd_integrator.updateFromTsdfLayer(false, false);

  GvdResult result(4, 8);
  for (int x = 0; x < voxels_per_side; ++x) {
    for (int y = 0; y < 4; ++y) {
      const auto& voxel = getGvdVoxel(x, y);
      result.distances(y, x) = voxel.observed ? voxel.distance : -1.0;
      result.is_voronoi(y, x) = isVoronoi(voxel) ? 1.0 : 0.0;
    }
  }

  VLOG(1) << "Result: " << result;
  GvdResult expected(4, 8);
  // clang-format off
  expected.distances << 0.00, 1.00, 2.00, 2.00, 2.24, 2.83, 3.61, 4.42,
                        0.00, 1.00, 1.41, 1.00, 1.41, 2.24, 3.16, 4.12,
                        0.00, 1.00, 1.00, 0.00, 1.00, 2.00, 3.00, 4.00,
                        0.00, 1.00, 1.41, 1.00, 1.41, 2.24, 3.16, 4.12;
  // clang-format on
  expected.is_voronoi = Eigen::MatrixXd::Zero(4, 8);
  expected.is_voronoi(0, 2) = 1.0;
  expected.is_voronoi(1, 2) = 1.0;

  for (int r = 0; r < expected.is_voronoi.rows(); ++r) {
    for (int c = 0; c < expected.is_voronoi.cols(); ++c) {
      EXPECT_EQ(expected.is_voronoi(r, c), result.is_voronoi(r, c))
          << " @ (" << r << ", " << c << ")";
      EXPECT_NEAR(expected.distances(r, c), result.distances(r, c), 1.0e-1)
          << " @ (" << r << ", " << c << ")";
    }
  }

  VLOG(1) << "Perturbing GVD: (3, 2) -> 10.0, (7, 2) -> 0.0";

  // raise the middle obstacle and lower one on the side
  setTsdfVoxel(3, 2, 10.0);
  setTsdfVoxel(7, 2, 0.0);
  gvd_integrator.updateFromTsdfLayer(true, false);

  for (int x = 0; x < voxels_per_side; ++x) {
    for (int y = 0; y < 4; ++y) {
      const auto& voxel = getGvdVoxel(x, y);
      result.distances(y, x) = voxel.observed ? voxel.distance : -1.0;
      result.is_voronoi(y, x) = isVoronoi(voxel) ? 1.0 : 0.0;
    }
  }

  // clang-format off
  expected.distances << 0.00, 1.00, 2.00, 3.00, 3.61, 2.83, 2.24, 2.00,
                        0.00, 1.00, 2.00, 3.00, 3.16, 2.24, 1.41, 1.00,
                        0.00, 1.00, 2.00, 3.00, 3.00, 2.00, 1.00, 0.00,
                        0.00, 1.00, 2.00, 3.00, 3.16, 2.24, 1.41, 1.00;
  // clang-format on
  expected.is_voronoi = Eigen::MatrixXd::Zero(4, 8);
  expected.is_voronoi(0, 4) = 1.0;
  expected.is_voronoi(1, 3) = 1.0;
  expected.is_voronoi(1, 4) = 1.0;
  expected.is_voronoi(2, 3) = 1.0;
  expected.is_voronoi(2, 4) = 1.0;
  expected.is_voronoi(3, 3) = 1.0;
  expected.is_voronoi(3, 4) = 1.0;

  for (int r = 0; r < expected.is_voronoi.rows(); ++r) {
    for (int c = 0; c < expected.is_voronoi.cols(); ++c) {
      EXPECT_EQ(expected.is_voronoi(r, c), result.is_voronoi(r, c))
          << " @ (" << r << ", " << c << ")";
      EXPECT_NEAR(expected.distances(r, c), result.distances(r, c), 1.0e-1)
          << " @ (" << r << ", " << c << ")";
    }
  }

  VLOG(1) << "Result: " << result;
}

TEST_F(TestFixture2d, NegativeIntegrationCorrect) {
  // exterior border is set to -truncation_distance
  for (int i = 0; i < voxels_per_side; ++i) {
    setTsdfVoxel(0, i, -2.0 * voxel_size);
    setTsdfVoxel(7, i, -2.0 * voxel_size);
    setTsdfVoxel(i, 0, -2.0 * voxel_size);
    setTsdfVoxel(i, 7, -2.0 * voxel_size);
  }

  for (int i = 1; i < voxels_per_side - 1; ++i) {
    setTsdfVoxel(1, i, -voxel_size);
    setTsdfVoxel(6, i, -voxel_size);
    setTsdfVoxel(i, 1, -voxel_size);
    setTsdfVoxel(i, 6, -voxel_size);
  }
  setTsdfVoxel(1, 1, -std::sqrt(2) * voxel_size);
  setTsdfVoxel(1, 6, -std::sqrt(2) * voxel_size);
  setTsdfVoxel(6, 1, -std::sqrt(2) * voxel_size);
  setTsdfVoxel(6, 6, -std::sqrt(2) * voxel_size);

  for (int i = 2; i < voxels_per_side - 2; ++i) {
    setTsdfVoxel(2, i, 0.0);
    setTsdfVoxel(5, i, 0.0);
    setTsdfVoxel(i, 2, 0.0);
    setTsdfVoxel(i, 5, 0.0);
  }

  setTsdfVoxel(3, 3, 1.0);
  setTsdfVoxel(3, 4, 1.0);
  setTsdfVoxel(4, 3, 1.0);
  setTsdfVoxel(4, 4, 1.0);

  gvd_config.min_diff_m = 0.0;
  gvd_config.min_distance_m = voxel_size;
  gvd_config.max_distance_m = 50.0;
  gvd_config.parent_derived_distance = true;
  gvd_config.voronoi_config.min_distance_m = 1.0;
  gvd_config.voronoi_config.parent_l1_separation = 2.0;
  gvd_config.positive_distance_only = false;
  gvd_config.extract_graph = false;

  GvdIntegrator gvd_integrator(gvd_config, tsdf_layer.get(), gvd_layer, mesh_layer);
  gvd_integrator.updateFromTsdfLayer(false, false);

  GvdResult result(voxels_per_side, voxels_per_side);
  for (int x = 0; x < voxels_per_side; ++x) {
    for (int y = 0; y < voxels_per_side; ++y) {
      const auto& voxel = getGvdVoxel(x, y);
      result.distances(y, x) = voxel.observed ? voxel.distance : -1.0;
      result.is_voronoi(y, x) = isVoronoi(voxel) ? 1.0 : 0.0;
    }
  }

  VLOG(1) << "Result: " << result;
  GvdResult expected(8, 8);
  // clang-format off
  expected.distances << -2.83, -2.24, -2.00, -2.00, -2.00, -2.00, -2.24, -2.83,
                        -2.24, -1.41, -1.00, -1.00, -1.00, -1.00, -1.41, -2.24,
                        -2.00, -1.00,  0.00,  0.00,  0.00,  0.00, -1.00, -2.00,
                        -2.00, -1.00,  0.00,  1.00,  1.00,  0.00, -1.00, -2.00,
                        -2.00, -1.00,  0.00,  1.00,  1.00,  0.00, -1.00, -2.00,
                        -2.00, -1.00,  0.00,  0.00,  0.00,  0.00, -1.00, -2.00,
                        -2.24, -1.41, -1.00, -1.00, -1.00, -1.00, -1.41, -2.24,
                        -2.83, -2.24, -2.00, -2.00, -2.00, -2.00, -2.24, -2.83;
  // clang-format on
  expected.is_voronoi = Eigen::MatrixXd::Zero(8, 8);

  for (int r = 0; r < expected.is_voronoi.rows(); ++r) {
    for (int c = 0; c < expected.is_voronoi.cols(); ++c) {
      EXPECT_EQ(expected.is_voronoi(r, c), result.is_voronoi(r, c))
          << " @ (" << r << ", " << c << ")";
      EXPECT_NEAR(expected.distances(r, c), result.distances(r, c), 1.0e-1)
          << " @ (" << r << ", " << c << ")";
    }
  }
}

TEST_F(SingleBlockTestFixture, PlaneCorrect) {
  GvdIntegrator gvd_integrator(gvd_config, tsdf_layer.get(), gvd_layer, mesh_layer);
  for (int x = 0; x < voxels_per_side; ++x) {
    for (int y = 0; y < voxels_per_side; ++y) {
      for (int z = 0; z < voxels_per_side; ++z) {
        const bool is_edge = (x == 0);
        setTsdfVoxel(x, y, z, is_edge ? 0.0 : truncation_distance);
      }
    }
  }

  gvd_integrator.updateFromTsdfLayer(true);

  for (int x = 0; x < voxels_per_side; ++x) {
    for (int y = 0; y < voxels_per_side; ++y) {
      for (int z = 0; z < voxels_per_side; ++z) {
        const auto& voxel = getGvdVoxel(x, y, z);

        double expected_distance = x * truncation_distance;

        EXPECT_NEAR(expected_distance, voxel.distance, 1.0e-6);
        EXPECT_TRUE(voxel.on_surface || voxel.has_parent);
        EXPECT_FALSE(isVoronoi(voxel))
            << voxel << " @ (" << x << ", " << y << ", " << z << ")";
      }
    }
  }
}

TEST_F(SingleBlockTestFixture, LCorrect) {
  GvdIntegrator gvd_integrator(gvd_config, tsdf_layer.get(), gvd_layer, mesh_layer);
  for (int x = 0; x < voxels_per_side; ++x) {
    for (int y = 0; y < voxels_per_side; ++y) {
      for (int z = 0; z < voxels_per_side; ++z) {
        const bool is_edge = (x == 0) || (y == 0);
        setTsdfVoxel(x, y, z, is_edge ? 0.0 : truncation_distance);
      }
    }
  }

  gvd_integrator.updateFromTsdfLayer(true);

  for (int x = 0; x < voxels_per_side; ++x) {
    for (int y = 0; y < voxels_per_side; ++y) {
      for (int z = 0; z < voxels_per_side; ++z) {
        const auto& voxel = getGvdVoxel(x, y, z);

        double expected_distance = std::min(x, y) * truncation_distance;

        EXPECT_NEAR(expected_distance, voxel.distance, 1.0e-6);
        EXPECT_TRUE(!isVoronoi(voxel) || !voxel.fixed);
        EXPECT_TRUE(voxel.on_surface || voxel.has_parent);

        // 8-connected diagonal plane should be voronoi
        EXPECT_EQ(std::abs(x - y) <= 1 && x >= 2 && y >= 2, isVoronoi(voxel))
            << voxel << " @ (" << x << ", " << y << ", " << z << ")";
      }
    }
  }
}

TEST_F(LargeSingleBlockTestFixture, LCorrect) {
  GvdIntegrator gvd_integrator(gvd_config, tsdf_layer.get(), gvd_layer, mesh_layer);
  for (int x = 0; x < voxels_per_side; ++x) {
    for (int y = 0; y < voxels_per_side; ++y) {
      for (int z = 0; z < voxels_per_side; ++z) {
        const bool is_edge = (x == 0) || (y == 0);
        setTsdfVoxel(x, y, z, is_edge ? 0.0 : truncation_distance);
      }
    }
  }

  gvd_integrator.updateFromTsdfLayer(true);

  for (int x = 0; x < voxels_per_side; ++x) {
    for (int y = 0; y < voxels_per_side; ++y) {
      for (int z = 0; z < voxels_per_side; ++z) {
        const auto& voxel = getGvdVoxel(x, y, z);

        double expected_distance = std::min(x, y) * truncation_distance;

        EXPECT_NEAR(expected_distance, voxel.distance, 1.0e-6);
        EXPECT_TRUE(!isVoronoi(voxel) || !voxel.fixed);
        EXPECT_TRUE(voxel.on_surface || voxel.has_parent);

        // 8-connected diagonal plane should be voronoi
        EXPECT_EQ(std::abs(x - y) <= 1 && x >= 2 && y >= 2, isVoronoi(voxel))
            << voxel << " @ (" << x << ", " << y << ", " << z << ")";
      }
    }
  }
}

TEST_F(SingleBlockTestFixture, CornerCorrect) {
  GvdIntegrator gvd_integrator(gvd_config, tsdf_layer.get(), gvd_layer, mesh_layer);
  gvd_integrator.updateFromTsdfLayer(true);

  for (int x = 0; x < voxels_per_side; ++x) {
    for (int y = 0; y < voxels_per_side; ++y) {
      for (int z = 0; z < voxels_per_side; ++z) {
        const auto& voxel = getGvdVoxel(x, y, z);

        double expected_distance = std::min(x, std::min(y, z)) * truncation_distance;

        EXPECT_NEAR(expected_distance, voxel.distance, 1.0e-6);
        EXPECT_TRUE(!isVoronoi(voxel) || !voxel.fixed);
        EXPECT_TRUE(voxel.on_surface || voxel.has_parent)
            << voxel << " @ (" << x << ", " << y << ", " << z << ")";

        // upper 2x2 should all be voronoi
        EXPECT_EQ(x >= 2 && y >= 2 && z >= 2, isVoronoi(voxel))
            << voxel << " @ (" << x << ", " << y << ", " << z << ")";
      }
    }
  }
}

TEST_F(ParentTestFixture, ParentsCorrect) {
  GvdIntegrator gvd_integrator(gvd_config, tsdf_layer.get(), gvd_layer, mesh_layer);
  gvd_integrator.updateFromTsdfLayer(true);

  for (int x = 0; x < voxels_per_side; ++x) {
    for (int y = 0; y < voxels_per_side; ++y) {
      for (int z = 0; z < voxels_per_side; ++z) {
        const auto& voxel = getGvdVoxel(x, y, z);

        if (x == 0 || y == 0 || z == 0) {
          // marching cubes can't handle sharp corners, so any "edge" (two zeros) or
          // "corner" (three zeros) voxel isn't on the surface
          int num_zeros = (x == 0 ? 1 : 0) + (y == 0 ? 1 : 0) + (z == 0 ? 1 : 0);
          EXPECT_TRUE(num_zeros >= 2 || voxel.on_surface)
              << voxel << " @ (" << x << ", " << y << ", " << z << ")";
          continue;
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
          EXPECT_TRUE(voxel.parent[0] == x || voxel.parent[1] == y ||
                      voxel.parent[2] == z);
        } else if (x == y && z > x) {
          EXPECT_EQ(z, voxel.parent[2]);
          EXPECT_TRUE(voxel.parent[0] == 0u || voxel.parent[1] == 0u);
          EXPECT_TRUE(voxel.parent[0] == x || voxel.parent[1] == y);
        } else if (x == z && y > x) {
          EXPECT_EQ(y, voxel.parent[1]);
          EXPECT_TRUE(voxel.parent[0] == 0u || voxel.parent[2] == 0u);
          EXPECT_TRUE(voxel.parent[0] == x || voxel.parent[2] == z);
        } else if (y == z && x > z) {
          EXPECT_EQ(x, voxel.parent[0]);
          EXPECT_TRUE(voxel.parent[1] == 0u || voxel.parent[2] == 0u);
          EXPECT_TRUE(voxel.parent[1] == y || voxel.parent[2] == z);
        } else {
          GlobalIndex expected_parent;
          if (x < y && x < z) {
            expected_parent << 0, y, z;
          } else if (y < x && y < z) {
            expected_parent << x, 0, z;
          } else {
            expected_parent << x, y, 0;
          }

          EXPECT_EQ(expected_parent, Eigen::Map<const GlobalIndex>(voxel.parent))
              << voxel << " @ (" << x << ", " << y << ", " << z << ")"
              << ",  expected parent: " << expected_parent.transpose();
        }
      }
    }
  }
}

TEST(TestVoxelSize, DISABLED_ShowVoxelSize) {
  LOG(INFO) << "GVD voxel size: " << sizeof(GvdVoxel) << " bytes";
  SUCCEED();
}

}  // namespace topology
}  // namespace hydra
