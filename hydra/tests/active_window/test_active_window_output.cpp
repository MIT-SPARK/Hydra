/* -----------------------------------------------------------------------------
 * Copyright 2022 Massachusetts Institute of Technology.
 * all rights reserved
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
#include <hydra/active_window/active_window_output.h>

using namespace spark_dsg;

namespace hydra {
namespace {
MeshBlock::Ptr block(const BlockIndex& index,
                     const std::vector<Eigen::Vector3f>& points,
                     uint32_t label = 1) {
  auto result = std::make_shared<MeshBlock>(1.0f, index, true);
  result->resizeVertices(points.size());
  for (size_t i = 0; i < points.size(); ++i) {
    result->setPos(i, points[i]);
    result->setLabel(i, label);
    result->setColor(i, Color(10, 20, 30));
  }
  return result;
}
MeshBlock::Ptr box(const BlockIndex& index,
                   const Eigen::Vector3f& center,
                   uint32_t label = 1) {
  const auto corners = BoundingBox(Eigen::Vector3f::Constant(0.1f), center).corners();
  return block(index, {corners.begin(), corners.end()}, label);
}
ActiveWindowOutput output(uint64_t timestamp,
                          const std::vector<MeshBlock::ConstPtr>& blocks,
                          const BlockIndices& archived = {}) {
  auto map = std::make_shared<VolumetricMap>(VolumetricMap::Config{});
  for (const auto& block : blocks) {
    map->allocateBlock(block->index);
    map->getMeshLayer().allocateBlock(block->index) = *block;
  }
  ActiveWindowOutput result;
  result.timestamp_ns = timestamp;
  result.archived = archived;
  result.setMap(map);
  return result;
}

}  // namespace

TEST(ActiveWindowOutput, CompactsRepeatedBlocksAndSharesFinalPayload) {
  auto first = output(1, {box({0, 0, 0}, {0, 0, 0})});
  std::weak_ptr<const MeshBlock> previous = first.meshUpdates()[0].blocks[0];
  for (uint64_t t = 2; t < 100; ++t) {
    first.updateFrom(output(t, {box({0, 0, 0}, {float(t), 0, 0})}), false);
  }
  EXPECT_TRUE(previous.expired());
  const auto batches = first.meshUpdates();
  ASSERT_EQ(batches.size(), 1u);
  ASSERT_EQ(batches[0].blocks.size(), 1u);
  EXPECT_EQ(batches[0].blocks[0],
            first.map().getMeshLayer().getBlockPtr(BlockIndex(0, 0, 0)));
  EXPECT_EQ(batches[0].timestamp_ns, 99u);
}

TEST(ActiveWindowOutput, PreservesArchiveBoundariesAndFinalMap) {
  auto first = output(1, {box({0, 0, 0}, {0, 0, 0})});
  first.updateFrom(output(2, {box({0, 0, 0}, {1, 0, 0})}), false);
  first.updateFrom(output(3, {}, {{0, 0, 0}}), false);
  EXPECT_FALSE(first.map().getMeshLayer().hasBlock(BlockIndex(0, 0, 0)));
  first.updateFrom(output(4, {box({0, 0, 0}, {2, 0, 0})}), false);
  first.updateFrom(output(5, {box({0, 0, 0}, {3, 0, 0})}), false);
  const auto batches = first.meshUpdates();
  ASSERT_EQ(batches.size(), 2u);
  EXPECT_EQ(batches[0].timestamp_ns, 2u);
  EXPECT_EQ(batches[1].archived.size(), 1u);
  EXPECT_EQ(batches[1].timestamp_ns, 5u);
  EXPECT_NE(batches[0].blocks[0], batches[1].blocks[0]);
  EXPECT_NEAR(batches[0].blocks[0]->pos(0).x(), 0.95f, 1e-6);
}

TEST(ActiveWindowOutput, CompactsAcrossOtherBlockArchives) {
  auto first = output(1, {box({0, 0, 0}, {0, 0, 0}), box({1, 0, 0}, {1, 0, 0})});
  first.updateFrom(output(2, {}, {{1, 0, 0}}), false);
  first.updateFrom(output(3, {box({0, 0, 0}, {2, 0, 0})}), false);
  const auto batches = first.meshUpdates();
  ASSERT_EQ(batches.size(), 2u);
  ASSERT_EQ(batches[0].blocks.size(), 1u);
  EXPECT_EQ(batches[0].blocks[0]->index, BlockIndex(1, 0, 0));
}

TEST(ActiveWindowOutput, CloneMapAndAlreadyCollatedInputs) {
  auto incoming = output(1, {box({0, 0, 0}, {0, 0, 0})});
  incoming.updateFrom(output(2, {}, {{0, 0, 0}}), false);
  incoming.updateFrom(output(3, {box({0, 0, 0}, {1, 0, 0})}), false);
  ActiveWindowOutput combined;
  combined.updateFrom(std::move(incoming), true);
  const auto batches = combined.meshUpdates();
  ASSERT_EQ(batches.size(), 2u);
  EXPECT_NE(batches[0].blocks[0], batches[1].blocks[0]);
  EXPECT_EQ(batches[1].blocks[0],
            combined.map().getMeshLayer().getBlockPtr(BlockIndex(0, 0, 0)));
}

}  // namespace hydra
