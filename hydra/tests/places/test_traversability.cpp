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
#include <hydra/places/block_traversability_clustering.h>
#include <hydra/places/traversability_postprocessing.h>

namespace hydra::places {

using Range = BlockTraversabilityClustering::Range;

TEST(TraversabilityPlaces, Indexing) {
  // Linear indexing.
  TraversabilityBlock block(1.0f, {1, 2, 3}, 10);
  EXPECT_EQ(block.indexFromLinear(0), Index2D(0, 0));
  EXPECT_EQ(block.indexFromLinear(9), Index2D(9, 0));
  EXPECT_EQ(block.indexFromLinear(10), Index2D(0, 1));
  EXPECT_EQ(block.indexFromLinear(99), Index2D(9, 9));

  // Global indexing.
  EXPECT_EQ(block.globalFromLocalIndex({0, 0}), BlockIndex(10, 20, 3));
  EXPECT_EQ(block.globalFromLocalIndex({3, 4}), BlockIndex(13, 24, 3));
  EXPECT_EQ(block.localFromGlobalIndex(BlockIndex(10, 20, 3)), Index2D(0, 0));
  EXPECT_EQ(block.localFromGlobalIndex(BlockIndex(13, 24, 3)), Index2D(3, 4));
  EXPECT_EQ(block.globalFromLocalIndex({-1, -2}), BlockIndex(9, 18, 3));

  // Layer indexing.
  TraversabilityLayer layer(0.1f, 10);
  EXPECT_EQ(layer.blockIndexFromGlobal(BlockIndex(10, 20, 3)), BlockIndex(1, 2, 3));
  EXPECT_EQ(layer.voxelIndexFromGlobal(BlockIndex(10, 20, 3)), Index2D(0, 0));
  EXPECT_EQ(layer.blockIndexFromGlobal(BlockIndex(13, 24, 3)), BlockIndex(1, 2, 3));
  EXPECT_EQ(layer.voxelIndexFromGlobal(BlockIndex(13, 24, 3)), Index2D(3, 4));
  EXPECT_EQ(layer.blockIndexFromGlobal(BlockIndex(9, 18, 3)), BlockIndex(0, 1, 3));
  EXPECT_EQ(layer.voxelIndexFromGlobal(BlockIndex(9, 18, 3)), Index2D(9, 8));
}

TEST(TraversabilityPlaces, Range) {
  const auto range = Range(0, 0, 9, 9);  // Edges are inclusive.
  EXPECT_EQ(range.x_start, 0);
  EXPECT_EQ(range.y_start, 0);
  EXPECT_EQ(range.x_end, 9);
  EXPECT_EQ(range.y_end, 9);
  EXPECT_EQ(range.width(), 10);
  EXPECT_EQ(range.height(), 10);
  EXPECT_EQ(range.area(), 100);
}

TEST(TraversabilityPlaces, RangeProjection) {
  // invertToSide
  auto range = Range(2, 3, 4, 5);
  EXPECT_EQ(range.invertToSide(0, 10, 3), Range(2, 0, 4, 2));  // bottom
  EXPECT_EQ(range.invertToSide(1, 10, 3), Range(0, 3, 1, 5));  // left
  EXPECT_EQ(range.invertToSide(2, 10, 3), Range(2, 6, 4, 8));  // top
  EXPECT_EQ(range.invertToSide(3, 10, 3), Range(5, 3, 7, 5));  // right

  // projectToNextBlock
  range = Range(2, 3, 4, 5);
  EXPECT_EQ(range.projectToNextBlock(0, 10, 3), Range(2, 7, 4, 9));  // bottom
  EXPECT_EQ(range.projectToNextBlock(1, 10, 3), Range(7, 3, 9, 5));  // left
  EXPECT_EQ(range.projectToNextBlock(2, 10, 3), Range(2, 0, 4, 2));  // top
  EXPECT_EQ(range.projectToNextBlock(3, 10, 3), Range(0, 3, 2, 5));  // right
};

TEST(TraversabilityPlaces, RobotFootprintTraversability) {
  using State = spark_dsg::TraversabilityState;
  // 0.1m voxels, 10 voxels per side, blocks covering [-1, 1)^2.
  TraversabilityLayer layer(0.1f, 10);
  for (int x = -1; x <= 0; ++x) {
    for (int y = -1; y <= 0; ++y) {
      auto& block = layer.allocateBlock(BlockIndex(x, y, 0), 10);
      for (auto& voxel : block.voxels) {
        voxel.state = State::INTRAVERSABLE;
      }
    }
  }

  const auto state_at = [&](int x, int y) {
    const BlockIndex block_index(x < 0 ? -1 : 0, y < 0 ? -1 : 0, 0);
    const Index2D local(x - block_index.x() * 10, y - block_index.y() * 10);
    return layer.getBlock(block_index).voxel(local).state;
  };

  // 0.24m x 0.24m footprint centered on the robot.
  RobotFootprintTraversability::Config config;
  config.bbox_min = Eigen::Vector3f(-0.12f, -0.12f, -1.0f);
  config.bbox_max = Eigen::Vector3f(0.12f, 0.12f, 1.0f);
  RobotFootprintTraversability processor(config);

  // Robot in voxel (-1, 0), so the footprint straddles the block boundary at x = 0.
  Eigen::Isometry3f world_T_body = Eigen::Isometry3f::Identity();
  world_T_body.translation() = Eigen::Vector3f(-0.05f, 0.05f, 0.5f);
  processor.addFootprint(world_T_body, layer);
  processor.markTraversable(layer);

  // Footprint covers x in [-0.17, 0.07] and y in [-0.07, 0.17], i.e., 3x3 voxels.
  EXPECT_EQ(processor.footprint().size(), 9u);
  for (int x = -10; x < 10; ++x) {
    for (int y = -10; y < 10; ++y) {
      const bool expected = x >= -2 && x <= 0 && y >= -1 && y <= 1;
      EXPECT_EQ(state_at(x, y) == State::TRAVERSABLE, expected)
          << "x=" << x << ", y=" << y;
    }
  }

  // Footprint persists even after the estimator resets the blocks.
  for (auto& block : layer) {
    block.reset();
  }
  world_T_body.translation() = Eigen::Vector3f(0.55f, 0.55f, 0.5f);
  processor.addFootprint(world_T_body, layer);
  processor.markTraversable(layer);
  EXPECT_EQ(processor.footprint().size(), 18u);
  EXPECT_EQ(state_at(-1, 0), State::TRAVERSABLE);  // From the first pose.
  EXPECT_EQ(state_at(5, 5), State::TRAVERSABLE);   // From the second pose.
  EXPECT_EQ(state_at(3, 3), State::UNKNOWN);

  // Elongated robot rotated by 90 degrees about z: long axis along world y.
  config.bbox_min = Eigen::Vector3f(-0.32f, -0.07f, -1.0f);
  config.bbox_max = Eigen::Vector3f(0.32f, 0.07f, 1.0f);
  RobotFootprintTraversability rotated(config);
  Eigen::Isometry3f rotated_pose = Eigen::Isometry3f::Identity();
  rotated_pose.linear() =
      Eigen::AngleAxisf(M_PI / 2.0f, Eigen::Vector3f::UnitZ()).toRotationMatrix();
  rotated_pose.translation() = Eigen::Vector3f(0.05f, 0.05f, 0.0f);
  rotated.addFootprint(rotated_pose, layer);
  // x in [-0.07, 0.17] -> 3 voxels, y in [-0.32, 0.42] -> 7 voxels.
  EXPECT_EQ(rotated.footprint().size(), 21u);
  EXPECT_TRUE(rotated.footprint().count(Index2D(0, 3)));
  EXPECT_TRUE(rotated.footprint().count(Index2D(0, -3)));
  EXPECT_FALSE(rotated.footprint().count(Index2D(3, 0)));
  EXPECT_FALSE(rotated.footprint().count(Index2D(2, 0)));
}

}  // namespace hydra::places
