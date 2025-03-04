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
#include <hydra/backend/external_loop_closure_receiver.h>
#include <kimera_pgmo/utils/common_functions.h>
#include <spark_dsg/node_attributes.h>
#include <spark_dsg/node_symbol.h>

namespace hydra {

using namespace spark_dsg;
using namespace std::chrono_literals;

using LookupResult = ExternalLoopClosureReceiver::LookupResult;

namespace {

inline NodeAttributes::Ptr makeAttrs(std::chrono::nanoseconds stamp) {
  auto attrs = std::make_unique<AgentNodeAttributes>();
  attrs->timestamp = stamp;
  return attrs;
}

inline void addToQueue(ExternalLoopClosureReceiver::Queue& queue,
                       int from_robot,
                       std::chrono::nanoseconds from_ns,
                       int to_robot,
                       std::chrono::nanoseconds to_ns) {
  pose_graph_tools::PoseGraphEdge edge;
  edge.key_from = from_ns.count();
  edge.robot_from = from_robot;
  edge.key_to = to_ns.count();
  edge.robot_to = to_robot;
  pose_graph_tools::PoseGraph graph;
  graph.edges.push_back(edge);
  queue.push(graph);
}

}  // namespace

TEST(ExternalLoopClosureReceiver, FindClosest) {
  ExternalLoopClosureReceiver::Config config;
  config.layer = "AGENTS";
  ExternalLoopClosureReceiver receiver(config, nullptr);

  DynamicSceneGraph graph;
  {  // no layer: unknown result
    std::chrono::nanoseconds stamp_ns = 100s;
    const auto result = receiver.findClosest(graph, stamp_ns.count(), 0, 1.0);
    EXPECT_EQ(result.status, LookupResult::Status::UNKNOWN);
  }

  graph.addLayer(2, kimera_pgmo::GetRobotPrefix(0));
  {  // empty layer: unknown result
    std::chrono::nanoseconds stamp_ns = 100s;
    const auto result = receiver.findClosest(graph, stamp_ns.count(), 0, 1.0);
    EXPECT_EQ(result.status, LookupResult::Status::UNKNOWN);
  }

  ASSERT_TRUE(
      graph.emplaceNode(2, "a0"_id, makeAttrs(10s), kimera_pgmo::GetRobotPrefix(0)));
  {  // node too early: unknown result
    std::chrono::nanoseconds stamp_ns = 100s;
    const auto result = receiver.findClosest(graph, stamp_ns.count(), 0, 1.0);
    EXPECT_EQ(result.status, LookupResult::Status::UNKNOWN);
  }

  ASSERT_TRUE(
      graph.emplaceNode(2, "a1"_id, makeAttrs(102s), kimera_pgmo::GetRobotPrefix(0)));
  {  // node too late: invalid result
    std::chrono::nanoseconds stamp_ns = 100s;
    const auto result = receiver.findClosest(graph, stamp_ns.count(), 0, 1.0);
    EXPECT_EQ(result.status, LookupResult::Status::INVALID);
  }

  {  // no max diff: valid result
    std::chrono::nanoseconds stamp_ns = 100s;
    const auto result = receiver.findClosest(graph, stamp_ns.count(), 0, 0.0);
    EXPECT_EQ(result.status, LookupResult::Status::VALID);
    EXPECT_EQ(result.id, "a1"_id);
  }

  {  // no max diff: picks earlier node
    std::chrono::nanoseconds stamp_ns = 20s;
    const auto result = receiver.findClosest(graph, stamp_ns.count(), 0, 0.0);
    EXPECT_EQ(result.status, LookupResult::Status::VALID);
    EXPECT_EQ(result.id, "a0"_id);
  }

  {  // different robot: unknown
    std::chrono::nanoseconds stamp_ns = 20s;
    const auto result = receiver.findClosest(graph, stamp_ns.count(), 1, 0.0);
    EXPECT_EQ(result.status, LookupResult::Status::UNKNOWN);
  }

  graph.addLayer(2, kimera_pgmo::GetRobotPrefix(0));
  ASSERT_TRUE(
      graph.emplaceNode(2, "b0"_id, makeAttrs(20s), kimera_pgmo::GetRobotPrefix(1)));
  ASSERT_TRUE(
      graph.emplaceNode(2, "b1"_id, makeAttrs(23s), kimera_pgmo::GetRobotPrefix(1)));

  {  // different robot: valid time
    std::chrono::nanoseconds stamp_ns = 22s;
    const auto result = receiver.findClosest(graph, stamp_ns.count(), 1, 2.0);
    EXPECT_EQ(result.status, LookupResult::Status::VALID);
    EXPECT_EQ(result.id, "b1"_id);
  }
}

TEST(ExternalLoopClosureReceiver, QueueCorrect) {
  ExternalLoopClosureReceiver::Config config;
  config.max_time_difference = 2.0;
  ExternalLoopClosureReceiver::Queue queue;
  config.layer = "AGENTS";
  ExternalLoopClosureReceiver receiver(config, &queue);

  DynamicSceneGraph graph;
  graph.addLayer(2, kimera_pgmo::GetRobotPrefix(0));
  graph.addLayer(2, kimera_pgmo::GetRobotPrefix(1));
  graph.emplaceNode(2, "a0"_id, makeAttrs(10s), kimera_pgmo::GetRobotPrefix(0));
  graph.emplaceNode(2, "a1"_id, makeAttrs(102s), kimera_pgmo::GetRobotPrefix(0));

  addToQueue(queue, 0, 11s, 0, 101s);
  addToQueue(queue, 0, 11s, 1, 21s);

  {  // first loop closure is available
    std::vector<std::pair<NodeId, NodeId>> results;
    std::vector<std::pair<NodeId, NodeId>> expected{{"a1"_id, "a0"_id}};
    receiver.update(graph, [&results](NodeId to, NodeId from, const gtsam::Pose3) {
      results.push_back({to, from});
    });
    EXPECT_EQ(results, expected);
  }

  graph.emplaceNode(2, "b0"_id, makeAttrs(20s), kimera_pgmo::GetRobotPrefix(1));
  {  // one node of edge is still unknown
    std::vector<std::pair<NodeId, NodeId>> results;
    std::vector<std::pair<NodeId, NodeId>> expected;
    receiver.update(graph, [&results](NodeId to, NodeId from, const gtsam::Pose3) {
      results.push_back({to, from});
    });
    EXPECT_EQ(results, expected);
  }

  graph.emplaceNode(2, "b1"_id, makeAttrs(23s), kimera_pgmo::GetRobotPrefix(1));
  {  // both nodes are known
    std::vector<std::pair<NodeId, NodeId>> results;
    std::vector<std::pair<NodeId, NodeId>> expected{{"b0"_id, "a0"_id}};
    receiver.update(graph, [&results](NodeId to, NodeId from, const gtsam::Pose3) {
      results.push_back({to, from});
    });
    EXPECT_EQ(results, expected);
  }
}

}  // namespace hydra
