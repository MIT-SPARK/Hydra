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
#include <hydra/frontend/frontend_module.h>

#include "hydra_test/resources.h"

namespace hydra {

using pose_graph_tools::PoseGraph;
using pose_graph_tools::PoseGraphNode;

namespace {

inline SharedDsgInfo::Ptr makeSharedDsg() {
  const LayerId mesh_layer_id = 1;
  const std::map<LayerId, char> layer_id_map{{DsgLayers::OBJECTS, 'o'},
                                             {DsgLayers::PLACES, 'p'},
                                             {DsgLayers::ROOMS, 'r'},
                                             {DsgLayers::BUILDINGS, 'b'}};
  return SharedDsgInfo::Ptr(new SharedDsgInfo(layer_id_map, mesh_layer_id));
}

inline FrontendConfig getDefaultConfig() {
  FrontendConfig to_return;
  to_return.should_log = false;
  to_return.filter_places = false;
  to_return.validate_vertices = false;

  to_return.semantic_label_file = hydra::test::get_default_semantic_map();
  return to_return;
}

ReconstructionOutput::Ptr getMsg() {
  auto msg = std::make_shared<ReconstructionOutput>();
  msg->timestamp_ns = 0;
  msg->places.reset(new hydra_msgs::ActiveLayer());
  msg->mesh.reset(new hydra_msgs::ActiveMesh());
  msg->current_position = Eigen::Vector3d::Zero();
  return msg;
}

void addPoseGraph(ReconstructionOutput& msg,
                  uint64_t timestamp_ns,
                  size_t index,
                  const Eigen::Vector3d& position) {
  PoseGraph pose_graph;
  pose_graph.header.stamp.fromNSec(timestamp_ns);
  pose_graph.header.frame_id = "world";

  pose_graph.nodes.push_back(PoseGraphNode());
  auto& node = pose_graph.nodes.back();
  node.header.stamp.fromNSec(timestamp_ns);
  node.header.frame_id = "world";
  node.robot_id = 0;
  node.key = index;
  node.pose.orientation.w = 1.0;
  node.pose.orientation.x = 0.0;
  node.pose.orientation.y = 0.0;
  node.pose.orientation.z = 0.0;
  node.pose.position.x = position.x();
  node.pose.position.y = position.y();
  node.pose.position.z = position.z();

  msg.pose_graphs.push_back(PoseGraph::ConstPtr(new PoseGraph(pose_graph)));
}

void addPlaces(ReconstructionOutput& output, const SceneGraphLayer& graph) {
  output.places.reset(new hydra_msgs::ActiveLayer());
  auto& places = const_cast<hydra_msgs::ActiveLayer&>(*output.places);
  places.header.stamp.fromNSec(output.timestamp_ns);
  places.header.frame_id = "world";

  std::unordered_set<NodeId> active_nodes;
  for (const auto& id_node_pair : graph.nodes()) {
    active_nodes.insert(id_node_pair.first);
  }

  places.layer_contents = graph.serializeLayer(active_nodes);
}

void addPlaceNode(IsolatedSceneGraphLayer& graph,
                  size_t index,
                  const Eigen::Vector3d& position) {
  auto attrs = std::make_unique<PlaceNodeAttributes>();
  attrs->position = position;
  attrs->is_active = true;
  graph.emplaceNode(NodeSymbol('p', index), std::move(attrs));
}

}  // namespace

TEST(FrontendModuleTests, TestAgentEdges) {
  auto prefix = RobotPrefixConfig(0);
  auto config = getDefaultConfig();
  auto dsg = makeSharedDsg();
  auto state = std::make_shared<SharedModuleState>();
  auto frontend = std::make_shared<FrontendModule>(prefix, config, dsg, state);
  auto queue = frontend->getQueue();

  IsolatedSceneGraphLayer places(2);

  auto msg = getMsg();
  addPlaces(*msg, places);
  queue->push(msg);
  ASSERT_TRUE(frontend->spinOnce());

  msg = getMsg();
  addPlaces(*msg, places);
  addPoseGraph(*msg, 10, 0, Eigen::Vector3d(1.0, 2.0, 3.0));
  queue->push(msg);
  ASSERT_TRUE(frontend->spinOnce());

  {
    const auto& agents = dsg->graph->getLayer(DsgLayers::AGENTS, prefix.key);
    ASSERT_EQ(agents.numNodes(), 1u);
    const DynamicSceneGraphNode& agent = agents.getNodeByIndex(0).value();
    EXPECT_FALSE(agent.hasParent());
  }

  addPlaceNode(places, 1, Eigen::Vector3d(1.0, 2.0, 3.0));

  msg = getMsg();
  addPlaces(*msg, places);
  queue->push(msg);
  ASSERT_TRUE(frontend->spinOnce());

  {
    const auto& agents = dsg->graph->getLayer(DsgLayers::AGENTS, prefix.key);
    ASSERT_EQ(agents.numNodes(), 1u);
    const DynamicSceneGraphNode& agent = agents.getNodeByIndex(0).value();
    EXPECT_TRUE(agent.hasParent());
  }
}

}  // namespace hydra
