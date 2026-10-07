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
#include <spark_dsg/node_symbol.h>

#include <chrono>

#include "hydra/common/sub_keyframes.h"
#include "hydra/frontend/subkeyframe_anchor.h"

namespace hydra {

namespace {
Eigen::Isometry3d atPosition(double x, double y, double z) {
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.translation() = Eigen::Vector3d(x, y, z);
  return pose;
}

void addAgent(spark_dsg::SceneGraph& graph,
              size_t index,
              uint64_t stamp_ns,
              const Eigen::Vector3d& position) {
  const spark_dsg::NodeSymbol id('a', index);
  auto attrs = std::make_unique<spark_dsg::AgentNodeAttributes>(
      std::chrono::nanoseconds(stamp_ns), Eigen::Quaterniond::Identity(), position, id);
  graph.emplaceNode(graph.getLayerKey(spark_dsg::DsgLayers::AGENTS)->layer,
                    id,
                    std::move(attrs),
                    'a');
}
}  // namespace

TEST(SubkeyframeAnchor, TemporalSelectionSpatialGate) {
  // Temporally-nearest to ts=2100 is the t=2000 anchor. Place it within
  // max_dist so it is accepted, even though other anchors are spatially
  // closer to the sub-keyframe position.
  std::vector<AnchorCandidate> anchors = {
      {1, 1000, atPosition(0, 0, 0)},
      {2, 2000, atPosition(1, 0, 0)},
      {3, 3000, atPosition(0, 0, 0)},
  };
  const Eigen::Vector3d subframe_position(1.1, 0, 0);
  auto idx = selectNearestAnchor(anchors, 2100, subframe_position, /*max_dist_m=*/2.0);
  ASSERT_TRUE(idx.has_value());
  EXPECT_EQ(anchors[*idx].id, 2u);
}

TEST(SubkeyframeAnchor, RejectsWhenTemporallyNearestIsFar) {
  // Temporally-nearest anchor (t=2000) is spatially far from the sub-keyframe,
  // while a temporally-farther anchor (t=3000) is spatially close. Loop-safety
  // requires we reject rather than fall back to the spatially-closer one.
  std::vector<AnchorCandidate> anchors = {
      {1, 1000, atPosition(0, 0, 0)},
      {2, 2000, atPosition(100, 0, 0)},
      {3, 3000, atPosition(0, 0, 0)},
  };
  const Eigen::Vector3d subframe_position(0, 0, 0);
  auto idx = selectNearestAnchor(anchors, 2100, subframe_position, /*max_dist_m=*/2.0);
  EXPECT_FALSE(idx.has_value());
}

TEST(SubkeyframeAnchor, AcceptsPostStopNearAnchor) {
  // Temporally-nearest anchor is far in time (large dt) but spatially at the
  // sub-keyframe (dist ~ 0), e.g. the robot stopped. Should be accepted.
  std::vector<AnchorCandidate> anchors = {
      {1, 1000, atPosition(0, 0, 0)},
  };
  const Eigen::Vector3d subframe_position(0.01, 0, 0);
  auto idx = selectNearestAnchor(
      anchors, 9000000000ULL, subframe_position, /*max_dist_m=*/2.0);
  ASSERT_TRUE(idx.has_value());
  EXPECT_EQ(anchors[*idx].id, 1u);
}

TEST(SubkeyframeAnchor, RelativeTransformComposesBack) {
  Eigen::Isometry3d world_T_anchor = Eigen::Isometry3d::Identity();
  world_T_anchor.translation() = Eigen::Vector3d(1, 0, 0);
  Eigen::Isometry3d world_T_sub = Eigen::Isometry3d::Identity();
  world_T_sub.translation() = Eigen::Vector3d(1.5, 0, 0);

  auto rel = computeRelativeTransform(world_T_anchor, world_T_sub);
  EXPECT_TRUE((world_T_anchor * rel).isApprox(world_T_sub));
  EXPECT_NEAR(rel.translation().x(), 0.5, 1e-9);
}

TEST(SubkeyframeAnchor, BuildsAttrsWithWorldPositionFromAnchor) {
  Eigen::Isometry3d world_T_anchor = Eigen::Isometry3d::Identity();
  world_T_anchor.translation() = Eigen::Vector3d(2, 0, 0);
  Eigen::Isometry3d world_T_sub = Eigen::Isometry3d::Identity();
  world_T_sub.translation() = Eigen::Vector3d(2.3, 0, 0);

  auto attrs = buildSubKeyframeAttrs(/*anchor_id=*/5u,
                                     world_T_anchor,
                                     world_T_sub,
                                     /*ts_ns=*/1234,
                                     "/data/subkf_1234");
  EXPECT_EQ(attrs->anchor_node_id, 5u);
  EXPECT_EQ(attrs->image_folder, "/data/subkf_1234");
  EXPECT_EQ(attrs->timestamp.count(), 1234);
  // initial world position seeded from world_T_sub (refined by backend later)
  EXPECT_TRUE(attrs->position.isApprox(Eigen::Vector3d(2.3, 0, 0)));
  // anchor_t_subframe = anchor^-1 * sub = (0.3, 0, 0)
  EXPECT_NEAR(attrs->anchor_t_subframe.x(), 0.3, 1e-9);
}

// Only anchors within the time window of the newest anchor are kept
TEST(SubkeyframeAnchor, WindowDropsOldAnchors) {
  spark_dsg::SceneGraph graph;
  AnchorWindow window(/*window_ns=*/1000);
  for (size_t i = 0; i < 5; ++i) {
    addAgent(graph, i, 500 * i, Eigen::Vector3d::Zero());
  }

  window.update(graph, {spark_dsg::NodeSymbol('a', 0), spark_dsg::NodeSymbol('a', 1)});
  EXPECT_EQ(window.size(), 2u);

  // newest anchor @ 2000 keeps anchors @ 1000, 1500 and 2000
  window.update(graph,
                {spark_dsg::NodeSymbol('a', 2),
                 spark_dsg::NodeSymbol('a', 3),
                 spark_dsg::NodeSymbol('a', 4)});
  const auto anchors = window.candidates(graph);
  ASSERT_EQ(anchors.size(), 3u);
  EXPECT_EQ(anchors.front().id, spark_dsg::NodeSymbol('a', 2));
  EXPECT_EQ(anchors.front().timestamp_ns, 1000u);
  EXPECT_EQ(anchors.back().id, spark_dsg::NodeSymbol('a', 4));

  // the newest anchor is always kept, e.g., when the robot stopped
  window.update(graph, {});
  EXPECT_EQ(window.size(), 3u);
}

// Candidates use the current poses of the agent nodes (e.g., after optimization) and
// skip nodes that were removed or are not agents
TEST(SubkeyframeAnchor, WindowReadsLatestPoses) {
  spark_dsg::SceneGraph graph;
  addAgent(graph, 0, 100, Eigen::Vector3d::Zero());
  addAgent(graph, 1, 200, Eigen::Vector3d::Zero());
  AnchorWindow window(/*window_ns=*/1000);
  window.update(graph,
                {spark_dsg::NodeSymbol('a', 0),
                 spark_dsg::NodeSymbol('a', 1),
                 spark_dsg::NodeSymbol('a', 7)});
  EXPECT_EQ(window.size(), 2u);

  graph.getNode(spark_dsg::NodeSymbol('a', 1)).attributes().position =
      Eigen::Vector3d(1.0, 2.0, 3.0);
  graph.removeNode(spark_dsg::NodeSymbol('a', 0));
  const auto anchors = window.candidates(graph);
  ASSERT_EQ(anchors.size(), 1u);
  EXPECT_EQ(anchors[0].id, spark_dsg::NodeSymbol('a', 1));
  EXPECT_TRUE(
      anchors[0].world_T_anchor.translation().isApprox(Eigen::Vector3d(1.0, 2.0, 3.0)));

  // selection on the window keeps temporal-nearest semantics
  const auto idx = selectNearestAnchor(
      anchors, 250, Eigen::Vector3d(1.0, 2.0, 3.5), /*max_dist_m=*/1.0);
  ASSERT_TRUE(idx.has_value());
  EXPECT_EQ(anchors[*idx].id, spark_dsg::NodeSymbol('a', 1));
}

// Sub-keyframe indices restart per robot, so ids encode the robot
TEST(SubkeyframeAnchor, NodeIdsUniquePerRobot) {
  const spark_dsg::NodeSymbol first(subKeyframeNodeId('k', 0, 5));
  const spark_dsg::NodeSymbol second(subKeyframeNodeId('k', 1, 5));
  EXPECT_NE(subKeyframeNodeId('k', 0, 5), subKeyframeNodeId('k', 1, 5));
  EXPECT_EQ(first.category(), 'k');
  EXPECT_EQ(first.categoryId(), 5u);
  EXPECT_EQ(second.category(), 'k');
  EXPECT_EQ(second.categoryId(), (size_t(1) << 48) | 5);
}

}  // namespace hydra
