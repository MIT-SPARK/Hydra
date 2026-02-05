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

namespace hydra::places {

using spark_dsg::TraversabilityNodeAttributes;
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

}  // namespace hydra::places
