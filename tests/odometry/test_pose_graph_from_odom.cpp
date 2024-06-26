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
#include <hydra/odometry/pose_graph_from_odom.h>

#include "hydra_test/config_guard.h"

namespace hydra {

using pose_graph_tools::PoseGraph;

TEST(PoseGraphFromOdom, GraphBuildingCorrect) {
  test::ConfigGuard guard;

  PoseGraphFromOdom::Config config;
  PoseGraphFromOdom tracker(config);

  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();

  {  // not enough poses for an edge: no pose graphs
    const auto packet = tracker.update(10u, pose);
    EXPECT_FALSE(packet.external_priors);
    EXPECT_EQ(packet.pose_graphs.size(), 0u);
  }

  {  // two input poses: single edge
    const auto packet = tracker.update(15u, pose);
    EXPECT_FALSE(packet.external_priors);
    ASSERT_EQ(packet.pose_graphs.size(), 1u);
    ASSERT_TRUE(packet.pose_graphs.front());
    const auto& graph = *packet.pose_graphs.front();
    ASSERT_EQ(graph.nodes.size(), 2u);
    ASSERT_EQ(graph.edges.size(), 1u);
    const auto& source = graph.nodes.at(0);
    const auto& target = graph.nodes.at(1);
    const auto& edge = graph.edges.at(0);
    EXPECT_EQ(source.stamp_ns, 10u);
    EXPECT_EQ(target.stamp_ns, 15u);
    EXPECT_EQ(edge.stamp_ns, 15u);
    EXPECT_EQ(graph.stamp_ns, 15u);
  }
}

}  // namespace hydra
