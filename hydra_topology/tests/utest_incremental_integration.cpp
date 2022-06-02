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
#include <ros/package.h>
#include <voxblox/integrator/tsdf_integrator.h>
#include <voxblox/utils/evaluation_utils.h>

#include "hydra_topology_test/layer_utils.h"
#include "hydra_topology_test/test_fixtures.h"

namespace hydra {
namespace topology {

using namespace voxblox;

using test_helpers::compareLayers;
using test_helpers::LayerComparisonResult;

class IncrementalIntegrationTestFixture : public ::testing::Test {
 public:
  IncrementalIntegrationTestFixture() = default;

  virtual ~IncrementalIntegrationTestFixture() = default;

  void SetUp() override {
    tsdf_layer.reset(new Layer<TsdfVoxel>(voxel_size, voxels_per_side));
    gvd_layer.reset(new Layer<GvdVoxel>(voxel_size, voxels_per_side));
    mesh_layer.reset(new MeshLayer(voxel_size * voxels_per_side));

    tsdf_integrator.reset(new FastTsdfIntegrator(tsdf_config, tsdf_layer.get()));
  }

  Layer<GvdVoxel>::Ptr getBatchGvd() {
    MeshLayer::Ptr mesh(new MeshLayer(voxel_size * voxels_per_side));
    Layer<GvdVoxel>::Ptr gvd(new Layer<GvdVoxel>(voxel_size, voxels_per_side));
    GvdIntegrator integrator(gvd_config, tsdf_layer.get(), gvd, mesh);

    integrator.updateFromTsdfLayer(false, true, true);

    return gvd;
  }

  void integrateTsdf(size_t index) {
    std::string package_path = ros::package::getPath("hydra_topology");
    std::string resource_path = package_path + "/tests/resources";
    std::string num = std::to_string(index);
    if (num.size() < 3) {
      num.insert(num.begin(), 3 - num.size(), '0');
    }

    std::string filename = resource_path + "/pointcloud_" + num + ".csv";

    std::ifstream fin(filename);

    std::vector<std::vector<double>> points;
    bool first_line = true;
    for (std::string line; std::getline(fin, line);) {
      if (first_line) {
        first_line = false;
        continue;
      }

      std::vector<double> point;
      std::stringstream ss(line);

      for (std::string val; std::getline(ss, val, ',');) {
        point.push_back(std::stof(val));
      }

      points.push_back(point);
    }

    std::cout << "Loaded " << points.size() << " points" << std::endl;

    Pointcloud cloud;
    Colors colors;
    for (const auto& point : points) {
      if (point.size() != 6) {
        continue;
      }

      cloud.push_back(Point(point[0], point[1], point[2]));
      colors.push_back(Color(static_cast<uint8_t>(point[3]),
                             static_cast<uint8_t>(point[4]),
                             static_cast<uint8_t>(point[5])));
    }

    std::cout << "Using " << cloud.size() << " points" << std::endl;

    Transformation identity;
    tsdf_integrator->integratePointCloud(identity, cloud, colors);
  }

  float voxel_size = 0.1;
  int voxels_per_side = 16;
  size_t num_poses = 10;
  GvdIntegratorConfig gvd_config;
  TsdfIntegratorBase::Config tsdf_config;

  Layer<TsdfVoxel>::Ptr tsdf_layer;

  Layer<GvdVoxel>::Ptr gvd_layer;
  MeshLayer::Ptr mesh_layer;

  Layer<GvdVoxel>::Ptr batch_gvd_layer;
  std::unique_ptr<FastTsdfIntegrator> tsdf_integrator;
};

TEST_F(IncrementalIntegrationTestFixture, DISABLED_TestBatchSame) {
  GvdIntegrator gvd_integrator(gvd_config, tsdf_layer.get(), gvd_layer, mesh_layer);

  for (size_t i = 0; i < num_poses; ++i) {
    integrateTsdf(i);

    gvd_integrator.updateFromTsdfLayer(true);
    auto batch_layer = getBatchGvd();

    LayerComparisonResult result =
        compareLayers(*gvd_layer, *batch_layer, &test_helpers::gvdVoxelsSame);
    EXPECT_EQ(0u, result.num_missing_lhs);
    EXPECT_EQ(0u, result.num_missing_rhs);
    EXPECT_EQ(0u, result.num_lhs_seen_rhs_unseen);
    EXPECT_EQ(0u, result.num_rhs_seen_lhs_unseen);
    EXPECT_LT(result.rmse, 1.0e-3);

    EXPECT_EQ(batch_layer->getNumberOfAllocatedBlocks(),
              gvd_layer->getNumberOfAllocatedBlocks());
  }
}

}  // namespace topology
}  // namespace hydra
