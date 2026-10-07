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
#include <hydra/backend/updates/update_gt_rooms_functor.h>
#include <spark_dsg/node_attributes.h>
#include <spark_dsg/node_symbol.h>

#include <limits>

#include "hydra_test/config_guard.h"
#include "hydra_test/resources.h"
#include "hydra_test/shared_dsg_fixture.h"

namespace hydra {
namespace {

using namespace spark_dsg;

UpdateGtRoomsFunctor::Config gtConfig() {
  UpdateGtRoomsFunctor::Config config;
  config.ground_truth_rooms_path = test::get_resource_path("rooms/gt_extents.yaml");
  return config;
}

void addPlace(SceneGraph& graph, NodeId id, double x, NodeAttributes::Ptr attrs) {
  attrs->position = Eigen::Vector3d(x, 0.0, 0.0);
  graph.emplaceNode(DsgLayers::PLACES, id, std::move(attrs));
}

}  // namespace

TEST(UpdateGtRoomsFunctorTests, AssignsWithoutDistancesAndExcludesFrontiers) {
  test::ConfigGuard guard(false);
  GlobalInfo::init(PipelineConfig{});
  auto dsg = test::makeSharedDsg();
  auto& graph = *dsg->graph;
  addPlace(graph, "x0"_id, 0.0, std::make_unique<NodeAttributes>());
  addPlace(graph, "p0"_id, 1.0, std::make_unique<PlaceNodeAttributes>());
  auto trav = std::make_unique<TraversabilityNodeAttributes>();
  trav->distance = std::numeric_limits<double>::quiet_NaN();
  addPlace(graph, "t0"_id, 4.0, std::move(trav));
  addPlace(graph, "p1"_id, 8.0, std::make_unique<PlaceNodeAttributes>(-1.0, 0));
  addPlace(graph, "x1"_id, 20.0, std::make_unique<NodeAttributes>());
  for (const auto active : {true, false}) {
    auto frontier = std::make_unique<PlaceNodeAttributes>(2.0, 3);
    frontier->real_place = false;
    frontier->active_frontier = active;
    addPlace(graph, active ? "f0"_id : "f1"_id, 0.0, std::move(frontier));
  }

  graph.insertEdge("p0"_id, "t0"_id);
  UpdateGtRoomsFunctor functor(gtConfig());
  functor.call(graph, *dsg, std::make_shared<UpdateInfo>());
  const auto& rooms = graph.getLayer(DsgLayers::ROOMS);
  ASSERT_EQ(rooms.numNodes(), 2u);
  EXPECT_EQ(graph.getNode("x0"_id).getParent(), "R0"_id);
  EXPECT_EQ(graph.getNode("p0"_id).getParent(), "R0"_id);
  EXPECT_EQ(graph.getNode("t0"_id).getParent(), "R1"_id);
  EXPECT_EQ(graph.getNode("p1"_id).getParent(), "R1"_id);
  EXPECT_FALSE(graph.getNode("x1"_id).getParent());
  EXPECT_FALSE(graph.getNode("f0"_id).getParent());
  EXPECT_FALSE(graph.getNode("f1"_id).getParent());
  EXPECT_EQ(rooms.numEdges(), 1u);
  EXPECT_TRUE(rooms.hasEdge("R0"_id, "R1"_id));
  EXPECT_TRUE(rooms.getNode("R0"_id).attributes().position.isApprox(
      Eigen::Vector3d(0.5, 0.0, 0.0)));
  EXPECT_TRUE(rooms.getNode("R1"_id).attributes().position.isApprox(
      Eigen::Vector3d(6.0, 0.0, 0.0)));
}

TEST(UpdateGtRoomsFunctorTests, ReplacesAssignmentsAndRemovesUnoccupiedRooms) {
  test::ConfigGuard guard(false);
  GlobalInfo::init(PipelineConfig{});
  auto dsg = test::makeSharedDsg();
  auto& graph = *dsg->graph;
  addPlace(graph, "p0"_id, 0.0, std::make_unique<PlaceNodeAttributes>());
  addPlace(graph, "p1"_id, 4.0, std::make_unique<PlaceNodeAttributes>());
  graph.insertEdge("p0"_id, "p1"_id);
  UpdateGtRoomsFunctor functor(gtConfig());
  const auto info = std::make_shared<UpdateInfo>();
  functor.call(graph, *dsg, info);
  ASSERT_TRUE(graph.hasEdge("R0"_id, "R1"_id));

  graph.getNode("p0"_id).attributes().position.x() = 8.0;
  graph.getNode("p1"_id).attributes<PlaceNodeAttributes>().real_place = false;
  functor.call(graph, *dsg, info);
  EXPECT_FALSE(graph.hasNode("R0"_id));
  ASSERT_TRUE(graph.hasNode("R1"_id));
  EXPECT_EQ(graph.getNode("p0"_id).getParent(), "R1"_id);
  EXPECT_FALSE(graph.getNode("p1"_id).getParent());
  EXPECT_EQ(graph.getLayer(DsgLayers::ROOMS).numEdges(), 0u);

  graph.getNode("p0"_id).attributes().position.x() = 20.0;
  functor.call(graph, *dsg, info);
  EXPECT_EQ(graph.getLayer(DsgLayers::ROOMS).numNodes(), 0u);
  EXPECT_FALSE(graph.getNode("p0"_id).getParent());
}

}  // namespace hydra
