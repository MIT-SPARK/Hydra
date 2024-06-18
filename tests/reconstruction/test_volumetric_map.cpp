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
#include <hydra/reconstruction/volumetric_map.h>

#include <filesystem>

#include "hydra_test/resources.h"

namespace hydra {

void fillSemanticBlock(SemanticBlock& block, size_t offset) {
  for (size_t i = 0; i < block.numVoxels(); ++i) {
    auto& voxel = block.getVoxel(i);
    if (i % 3 == 0 || i == block.numVoxels() - 1) {
      voxel.empty = false;
      voxel.semantic_label = 2 * i + offset;
      voxel.semantic_likelihoods = Eigen::VectorXf::Constant(
          i / (32 * 32), static_cast<float>(i + offset) / (32 * 32 * 32));
    } else {
      voxel.semantic_label = i + offset;
      voxel.empty = true;
    }
  }
}

void compareVoxels(const SemanticBlock& lhs,
                   const SemanticBlock& rhs) {
  CHECK_EQ(lhs.numVoxels(), rhs.numVoxels());
  for (size_t i = 0; i < lhs.numVoxels(); ++i) {
    SCOPED_TRACE("Voxel " + std::to_string(i));
    auto& v_lhs = lhs.getVoxel(i);
    auto& v_rhs = rhs.getVoxel(i);
    EXPECT_EQ(v_lhs.empty, v_rhs.empty);
    EXPECT_EQ(v_lhs.semantic_label, v_rhs.semantic_label);
    EXPECT_TRUE(v_lhs.semantic_likelihoods.isApprox(v_rhs.semantic_likelihoods));
  }
}

struct VolumetricMapFixture : public ::testing::Test {
  virtual void SetUp() override {
    const auto path = VolumetricMapFixture::filepath();
    if (std::filesystem::exists(path)) {
      std::filesystem::remove_all(path);
      LOG(ERROR) << "test path '" << path.string() << "' previously existed!";
    }

    if (!std::filesystem::create_directories(path)) {
      throw std::runtime_error("unable to create test resource at '" + path.string() +
                               "'");
    }
  }

  static std::filesystem::path filepath() {
    return std::filesystem::path(test::get_resource_path("volumetric_map_tests"));
  }

  virtual ~VolumetricMapFixture() {
    const auto path = VolumetricMapFixture::filepath();
    std::filesystem::remove_all(path);
  }
};

TEST_F(VolumetricMapFixture, SaveLoadEmptyCorrect) {
  const auto map_path = VolumetricMapFixture::filepath() / "map";

  VolumetricMap::Config config;
  config.voxel_size = 0.2;
  config.voxels_per_side = 32;
  config.truncation_distance = 0.5;

  VolumetricMap original(config, true);
  original.save(map_path.string());

  auto result = VolumetricMap::load(map_path.string());
  ASSERT_TRUE(result != nullptr);
  EXPECT_NEAR(result->config.voxel_size, config.voxel_size, 1.0e-9);
  EXPECT_EQ(result->config.voxels_per_side, config.voxels_per_side);
  EXPECT_NEAR(result->config.truncation_distance, config.truncation_distance, 1.0e-9);
  EXPECT_TRUE(result->hasSemantics());
}

TEST_F(VolumetricMapFixture, SaveLoadSemanticsCorrect) {
  const auto map_path = VolumetricMapFixture::filepath() / "map";

  VolumetricMap::Config config;
  config.voxel_size = 0.2;
  config.voxels_per_side = 32;
  config.truncation_distance = 0.5;

  VolumetricMap original(config, true);
  auto semantics = original.getSemanticLayer();
  ASSERT_TRUE(semantics != nullptr);

  const BlockIndex idx1(0, 0, 0);
  auto block1 = semantics->allocateBlockPtr(idx1);
  fillSemanticBlock(*block1, 0);

  const BlockIndex idx2(0, 1, 0);
  auto block2 = semantics->allocateBlockPtr(idx2);
  fillSemanticBlock(*block2, 1);

  original.save(map_path.string());

  auto result = VolumetricMap::load(map_path.string());
  ASSERT_TRUE(result != nullptr);
  EXPECT_NEAR(result->config.voxel_size, config.voxel_size, 1.0e-9);
  EXPECT_EQ(result->config.voxels_per_side, config.voxels_per_side);
  EXPECT_NEAR(result->config.truncation_distance, config.truncation_distance, 1.0e-9);
  EXPECT_TRUE(result->hasSemantics());

  auto result_semantics = original.getSemanticLayer();
  ASSERT_TRUE(result_semantics != nullptr);
  ASSERT_TRUE(result_semantics->hasBlock(idx1));
  ASSERT_TRUE(result_semantics->hasBlock(idx2));
  const auto& result_block1 = result_semantics->getBlock(idx1);
  SCOPED_TRACE("block 1");
  compareVoxels(*block1, result_block1);
  const auto& result_block2 = result_semantics->getBlock(idx2);
  SCOPED_TRACE("block 2");
  compareVoxels(*block2, result_block2);
}

}  // namespace hydra
