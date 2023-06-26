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
#include <hydra/backend/merge_handler.h>

namespace hydra {

using ObjectAttrs = ObjectNodeAttributes;
using PlaceAttrs = PlaceNodeAttributes;
using dsg_updates::UpdateObjectsFunctor;
using dsg_updates::UpdatePlacesFunctor;

void setBoundingBox(DynamicSceneGraph& graph,
                    NodeId node_id,
                    const Eigen::Vector3f& min,
                    const Eigen::Vector3f& max,
                    bool update_node) {
  auto vertices = graph.getMeshVertices();
  size_t index = 2 * NodeSymbol(node_id).categoryId();
  if (vertices->size() < index + 2) {
    vertices->resize(index + 2);
  }

  auto& min_point = vertices->at(index);
  min_point.x = min.x();
  min_point.y = min.y();
  min_point.z = min.z();

  auto& max_point = vertices->at(index + 1);
  max_point.x = max.x();
  max_point.y = max.y();
  max_point.z = max.z();

  if (!update_node) {
    return;
  }

  const SceneGraphNode& node = graph.getNode(node_id).value();
  SemanticNodeAttributes& attrs = node.attributes<SemanticNodeAttributes>();
  attrs.position = ((max + min) / 2.0).cast<double>();
  attrs.bounding_box = BoundingBox(min, max);
}

// test that single merges to valid targets work
TEST(MergeHandlerTests, TestValidMergeNoUndo) {
  DynamicSceneGraph graph;
  graph.emplaceNode(DsgLayers::OBJECTS, "O1"_id, std::make_unique<ObjectAttrs>());
  graph.emplaceNode(DsgLayers::OBJECTS, "O2"_id, std::make_unique<ObjectAttrs>());

  std::map<NodeId, NodeId> proposed_merges{{"O1"_id, "O2"_id}};

  // object / place updaters only needed for undoing merges
  MergeHandler handler(nullptr, nullptr, false);
  handler.updateMerges(proposed_merges, graph);

  EXPECT_EQ(handler.mergedNodes(), proposed_merges);
  EXPECT_EQ(graph.numNodes(), 1u);
  EXPECT_FALSE(graph.hasNode("O1"_id));
  EXPECT_TRUE(graph.hasNode("O2"_id));

  std::map<NodeId, std::set<NodeId>> expected_parents{{"O2"_id, {"O1"_id}}};
  EXPECT_EQ(handler.mergeParentNodes(), expected_parents);
}

// test that nodes that are propsed to merge to each other only get merged once
TEST(MergeHandlerTests, TestOpposingMergeNoUndo) {
  DynamicSceneGraph graph;
  graph.emplaceNode(DsgLayers::OBJECTS, "O1"_id, std::make_unique<ObjectAttrs>());
  graph.emplaceNode(DsgLayers::OBJECTS, "O2"_id, std::make_unique<ObjectAttrs>());

  std::map<NodeId, NodeId> proposed_merges{{"O1"_id, "O2"_id}, {"O2"_id, "O1"_id}};

  // object / place updaters only needed for undoing merges
  MergeHandler handler(nullptr, nullptr, false);
  handler.updateMerges(proposed_merges, graph);

  // handler iterates through proposed merges in reverse
  std::map<NodeId, NodeId> expected_merges{{"O2"_id, "O1"_id}};
  EXPECT_EQ(handler.mergedNodes(), expected_merges);
  EXPECT_EQ(graph.numNodes(), 1u);
  EXPECT_FALSE(graph.hasNode("O2"_id));
  EXPECT_TRUE(graph.hasNode("O1"_id));

  std::map<NodeId, std::set<NodeId>> expected_parents{{"O1"_id, {"O2"_id}}};
  EXPECT_EQ(handler.mergeParentNodes(), expected_parents);
}

// test that nodes that have been merged to previously update their merge children to
// the right merge parent
TEST(MergeHandlerTests, TestChainedMergeNoUndo) {
  DynamicSceneGraph graph;
  graph.emplaceNode(DsgLayers::OBJECTS, "O1"_id, std::make_unique<ObjectAttrs>());
  graph.emplaceNode(DsgLayers::OBJECTS, "O2"_id, std::make_unique<ObjectAttrs>());
  graph.emplaceNode(DsgLayers::OBJECTS, "O3"_id, std::make_unique<ObjectAttrs>());

  std::map<NodeId, NodeId> proposed_merges{{"O2"_id, "O1"_id}, {"O3"_id, "O2"_id}};

  // object / place updaters only needed for undoing merges
  MergeHandler handler(nullptr, nullptr, false);
  handler.updateMerges(proposed_merges, graph);

  std::map<NodeId, NodeId> expected_merges{{"O2"_id, "O1"_id}, {"O3"_id, "O1"_id}};
  EXPECT_EQ(handler.mergedNodes(), expected_merges);
  EXPECT_EQ(graph.numNodes(), 1u);
  EXPECT_FALSE(graph.hasNode("O3"_id));
  EXPECT_FALSE(graph.hasNode("O2"_id));
  EXPECT_TRUE(graph.hasNode("O1"_id));

  std::map<NodeId, std::set<NodeId>> expected_parents{{"O1"_id, {"O2"_id, "O3"_id}}};
  EXPECT_EQ(handler.mergeParentNodes(), expected_parents);
}

// test that nodes that have been merged to previously update their merge children to
// the right merge parent
TEST(MergeHandlerTests, TestMergedTargetNoUndo) {
  DynamicSceneGraph graph;
  graph.emplaceNode(DsgLayers::OBJECTS, "O1"_id, std::make_unique<ObjectAttrs>());
  graph.emplaceNode(DsgLayers::OBJECTS, "O2"_id, std::make_unique<ObjectAttrs>());
  graph.emplaceNode(DsgLayers::OBJECTS, "O3"_id, std::make_unique<ObjectAttrs>());

  std::map<NodeId, NodeId> proposed_merges{{"O1"_id, "O2"_id}, {"O2"_id, "O3"_id}};

  // object / place updaters only needed for undoing merges
  MergeHandler handler(nullptr, nullptr, false);
  handler.updateMerges(proposed_merges, graph);

  std::map<NodeId, NodeId> expected_merges{{"O1"_id, "O3"_id}, {"O2"_id, "O3"_id}};
  EXPECT_EQ(handler.mergedNodes(), expected_merges);
  EXPECT_EQ(graph.numNodes(), 1u);
  EXPECT_FALSE(graph.hasNode("O1"_id));
  EXPECT_FALSE(graph.hasNode("O2"_id));
  EXPECT_TRUE(graph.hasNode("O3"_id));

  std::map<NodeId, std::set<NodeId>> expected_parents{{"O3"_id, {"O1"_id, "O2"_id}}};
  EXPECT_EQ(handler.mergeParentNodes(), expected_parents);
}

// test that removed nodes in the "frontend" graph get cleared appropriately
TEST(MergeHandlerTests, TestRemovedNodesNoUndo) {
  DynamicSceneGraph graph;
  graph.emplaceNode(DsgLayers::OBJECTS, "O1"_id, std::make_unique<ObjectAttrs>());
  graph.emplaceNode(DsgLayers::OBJECTS, "O2"_id, std::make_unique<ObjectAttrs>());
  graph.emplaceNode(DsgLayers::OBJECTS, "O3"_id, std::make_unique<ObjectAttrs>());
  graph.emplaceNode(DsgLayers::OBJECTS, "O4"_id, std::make_unique<ObjectAttrs>());
  graph.emplaceNode(DsgLayers::OBJECTS, "O5"_id, std::make_unique<ObjectAttrs>());
  auto frontend_graph = graph.clone();
  frontend_graph->removeNode("O4"_id);
  frontend_graph->removeNode("O3"_id);

  std::map<NodeId, NodeId> proposed_merges{
      {"O2"_id, "O1"_id}, {"O3"_id, "O2"_id}, {"O4"_id, "O5"_id}};

  // object / place updaters only needed for undoing merges
  MergeHandler handler(nullptr, nullptr, false);
  handler.updateMerges(proposed_merges, graph);

  {  // variable scope
    std::map<NodeId, NodeId> expected_merges{
        {"O2"_id, "O1"_id}, {"O3"_id, "O1"_id}, {"O4"_id, "O5"_id}};
    EXPECT_EQ(handler.mergedNodes(), expected_merges);

    std::map<NodeId, std::set<NodeId>> expected_parents{{"O1"_id, {"O2"_id, "O3"_id}},
                                                        {"O5"_id, {"O4"_id}}};
    EXPECT_EQ(handler.mergeParentNodes(), expected_parents);
  }

  handler.updateFromUnmergedGraph(*frontend_graph);

  {  // variable scope
    std::map<NodeId, NodeId> expected_merges{{"O2"_id, "O1"_id}};
    EXPECT_EQ(handler.mergedNodes(), expected_merges);
    std::map<NodeId, std::set<NodeId>> expected_parents{{"O1"_id, {"O2"_id}}};
    EXPECT_EQ(handler.mergeParentNodes(), expected_parents);
  }
}

// test that single merges to valid targets work when undos are posible
TEST(MergeHandlerTests, TestValidMergeUndo) {
  DynamicSceneGraph graph;
  graph.emplaceNode(DsgLayers::OBJECTS, "O1"_id, std::make_unique<ObjectAttrs>());
  graph.emplaceNode(DsgLayers::OBJECTS, "O2"_id, std::make_unique<ObjectAttrs>());

  std::map<NodeId, NodeId> proposed_merges{{"O1"_id, "O2"_id}};

  auto object_functor = std::make_shared<UpdateObjectsFunctor>();
  auto places_functor = std::make_shared<UpdatePlacesFunctor>(0.5, 0.1);
  MergeHandler handler(object_functor, places_functor, true);
  handler.updateMerges(proposed_merges, graph);

  EXPECT_EQ(handler.mergedNodes(), proposed_merges);
  EXPECT_EQ(graph.numNodes(), 1u);
  EXPECT_FALSE(graph.hasNode("O1"_id));
  EXPECT_TRUE(graph.hasNode("O2"_id));

  std::map<NodeId, std::set<NodeId>> expected_parents{{"O2"_id, {"O1"_id}}};
  EXPECT_EQ(handler.mergeParentNodes(), expected_parents);
}

TEST(MergeHandlerTests, TestUndoMergePlaces) {
  DynamicSceneGraph graph;
  graph.emplaceNode(DsgLayers::PLACES, "p1"_id, std::make_unique<PlaceAttrs>(1.0, 2));
  graph.emplaceNode(DsgLayers::PLACES, "p2"_id, std::make_unique<PlaceAttrs>(1.0, 2));
  graph.emplaceNode(DsgLayers::PLACES, "p3"_id, std::make_unique<PlaceAttrs>(1.0, 2));
  graph.emplaceNode(DsgLayers::PLACES, "p4"_id, std::make_unique<PlaceAttrs>(1.0, 2));
  graph.emplaceNode(DsgLayers::PLACES, "p5"_id, std::make_unique<PlaceAttrs>(1.0, 2));
  graph.emplaceNode(DsgLayers::ROOMS, "r1"_id, std::make_unique<NodeAttributes>());
  graph.emplaceNode(DsgLayers::ROOMS, "r2"_id, std::make_unique<NodeAttributes>());
  graph.getNode("p1"_id)->get().attributes().position = Eigen::Vector3d::Zero();
  graph.getNode("p2"_id)->get().attributes().position = Eigen::Vector3d::Zero();
  graph.getNode("p2"_id)->get().attributes().is_active = true;
  graph.getNode("p5"_id)->get().attributes().is_active = false;
  graph.insertEdge("p1"_id, "p3"_id);
  graph.insertEdge("p2"_id, "p4"_id);
  graph.insertEdge("r1"_id, "p1"_id);
  graph.insertEdge("r2"_id, "p2"_id);
  graph.initMesh();

  std::map<NodeId, NodeId> proposed_merges{{"p2"_id, "p1"_id}, {"p5"_id, "p1"_id}};

  auto object_functor = std::make_shared<UpdateObjectsFunctor>();
  auto places_functor = std::make_shared<UpdatePlacesFunctor>(0.5, 0.1);
  MergeHandler handler(object_functor, places_functor, true);
  handler.updateMerges(proposed_merges, graph);

  EXPECT_EQ(handler.mergedNodes(), proposed_merges);
  EXPECT_TRUE(graph.hasEdge("p1"_id, "p3"_id));
  EXPECT_TRUE(graph.hasEdge("p1"_id, "p4"_id));

  gtsam::Values values;
  values.insert("p1"_id, gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(4.0, 5.0, 6.0)));
  values.insert("p2"_id, gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(7.0, 8.0, 9.0)));
  // this gets used through checking all the children
  values.insert("p5"_id, gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0.0, 0.0, 0.0)));

  UpdateInfo info{&values, nullptr, false, 0, false};
  handler.checkAndUndo(graph, info);

  std::map<NodeId, NodeId> expected_merges{{"p5"_id, "p1"_id}};
  EXPECT_EQ(handler.mergedNodes(), expected_merges);
  EXPECT_TRUE(graph.hasNode("p1"_id));
  EXPECT_TRUE(graph.hasNode("p2"_id));
  EXPECT_TRUE(graph.hasEdge("p1"_id, "p3"_id));
  EXPECT_TRUE(graph.hasEdge("p1"_id, "r1"_id));
  EXPECT_TRUE(graph.hasEdge("p2"_id, "r2"_id));

  // attributes are cached for non-active nodes
  Eigen::Vector3d expected_pos1(0.0, 0.0, 0.0);
  EXPECT_NEAR(0.0, (expected_pos1 - graph.getPosition("p1"_id)).norm(), 1.0e-6);
  Eigen::Vector3d expected_pos2(7.0, 8.0, 9.0);
  EXPECT_NEAR(0.0, (expected_pos2 - graph.getPosition("p2"_id)).norm(), 1.0e-6);
}

TEST(MergeHandlerTests, TestUndoMergeObjects) {
  DynamicSceneGraph graph;
  graph.emplaceNode(DsgLayers::OBJECTS, "O1"_id, std::make_unique<ObjectAttrs>());
  graph.emplaceNode(DsgLayers::OBJECTS, "O2"_id, std::make_unique<ObjectAttrs>());
  graph.getNode("O1"_id)->get().attributes().is_active = false;
  graph.getNode("O2"_id)->get().attributes().is_active = true;
  graph.initMesh();

  setBoundingBox(graph,
                 "O1"_id,
                 Eigen::Vector3f(0.0, 0.0, 0.0),
                 Eigen::Vector3f(1.0, 1.0, 1.0),
                 true);
  setBoundingBox(graph,
                 "O2"_id,
                 Eigen::Vector3f(2.0, 2.0, 2.0),
                 Eigen::Vector3f(3.0, 3.0, 3.0),
                 true);

  std::map<NodeId, NodeId> proposed_merges{{"O2"_id, "O1"_id}};

  auto object_functor = std::make_shared<UpdateObjectsFunctor>();
  auto places_functor = std::make_shared<UpdatePlacesFunctor>(0.5, 0.1);
  MergeHandler handler(object_functor, places_functor, true);
  handler.updateMerges(proposed_merges, graph);

  EXPECT_EQ(handler.mergedNodes(), proposed_merges);
  EXPECT_EQ(graph.numNodes(false), 1u);

  gtsam::Values values;
  UpdateInfo info{&values, nullptr, false, 0, false};
  handler.checkAndUndo(graph, info);

  EXPECT_TRUE(handler.mergedNodes().empty());
  EXPECT_EQ(graph.numNodes(false), 2u);
  EXPECT_TRUE(graph.hasNode("O1"_id));
  EXPECT_TRUE(graph.hasNode("O2"_id));

  // attributes are cached for non-active nodes
  Eigen::Vector3d expected_pos1(0.5, 0.5, 0.5);
  EXPECT_NEAR(0.0, (expected_pos1 - graph.getPosition("O1"_id)).norm(), 1.0e-6);
  Eigen::Vector3d expected_pos2(2.5, 2.5, 2.5);
  EXPECT_NEAR(0.0, (expected_pos2 - graph.getPosition("O2"_id)).norm(), 1.0e-6);
}

// check that updating from unmerged grabs the right siblings and attributes
// from the frontend
TEST(MergeHandlerTests, TestUpdateFromUnmergedWithUndo) {
  DynamicSceneGraph graph;
  graph.emplaceNode(DsgLayers::PLACES, "p1"_id, std::make_unique<PlaceAttrs>(1.0, 2));
  graph.emplaceNode(DsgLayers::PLACES, "p2"_id, std::make_unique<PlaceAttrs>(1.0, 2));
  graph.emplaceNode(DsgLayers::PLACES, "p3"_id, std::make_unique<PlaceAttrs>(1.0, 2));
  graph.emplaceNode(DsgLayers::PLACES, "p4"_id, std::make_unique<PlaceAttrs>(1.0, 2));
  graph.emplaceNode(DsgLayers::PLACES, "p5"_id, std::make_unique<PlaceAttrs>(1.0, 2));
  graph.emplaceNode(DsgLayers::PLACES, "p6"_id, std::make_unique<PlaceAttrs>(1.0, 2));
  graph.emplaceNode(DsgLayers::PLACES, "p7"_id, std::make_unique<PlaceAttrs>(1.0, 2));
  graph.getNode("p1"_id)->get().attributes().position = Eigen::Vector3d::Zero();
  graph.getNode("p2"_id)->get().attributes().position = Eigen::Vector3d::Zero();
  graph.getNode("p2"_id)->get().attributes().is_active = true;
  graph.getNode("p5"_id)->get().attributes().is_active = false;
  graph.getNode("p6"_id)->get().attributes().is_active = false;
  graph.getNode("p7"_id)->get().attributes().is_active = true;
  graph.insertEdge("p1"_id, "p3"_id);
  graph.insertEdge("p2"_id, "p4"_id);
  graph.insertEdge("p6"_id, "p4"_id);
  graph.insertEdge("r1"_id, "p1"_id);
  graph.insertEdge("r2"_id, "p2"_id);
  graph.initMesh();

  // missing p5 for merge
  DynamicSceneGraph fgraph;
  fgraph.emplaceNode(DsgLayers::PLACES, "p1"_id, std::make_unique<PlaceAttrs>(1.0, 2));
  fgraph.emplaceNode(DsgLayers::PLACES, "p2"_id, std::make_unique<PlaceAttrs>(1.0, 2));
  fgraph.emplaceNode(DsgLayers::PLACES, "p3"_id, std::make_unique<PlaceAttrs>(1.0, 2));
  fgraph.emplaceNode(DsgLayers::PLACES, "p4"_id, std::make_unique<PlaceAttrs>(1.0, 2));
  fgraph.emplaceNode(DsgLayers::PLACES, "p6"_id, std::make_unique<PlaceAttrs>(1.0, 2));
  fgraph.getNode("p2"_id)->get().attributes().is_active = true;

  std::map<NodeId, NodeId> proposed_merges{
      {"p2"_id, "p1"_id}, {"p5"_id, "p1"_id}, {"p6"_id, "p1"_id}, {"p7"_id, "p4"_id}};

  auto object_functor = std::make_shared<UpdateObjectsFunctor>();
  auto places_functor = std::make_shared<UpdatePlacesFunctor>(0.5, 0.1);
  MergeHandler handler(object_functor, places_functor, true);

  handler.updateMerges(proposed_merges, graph);
  EXPECT_EQ(handler.mergedNodes(), proposed_merges);

  // p5 is removed, so should not be in merges any more
  handler.updateFromUnmergedGraph(fgraph);
  std::map<NodeId, NodeId> expected_merges{{"p2"_id, "p1"_id}, {"p6"_id, "p1"_id}};
  EXPECT_EQ(handler.mergedNodes(), expected_merges);

  // p7 was removed, so p4 should not be a parent anymore
  EXPECT_FALSE(handler.mergeParentNodes().count("p4"_id));

  gtsam::Values values;
  values.insert("p1"_id, gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(4.0, 5.0, 6.0)));
  values.insert("p2"_id, gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(7.0, 8.0, 9.0)));
  values.insert("p6"_id, gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(9.0, 10.0, 11.0)));

  // we want to make sure the neighbors of p2 disappear, but p6 stays untouched
  UpdateInfo info{&values, nullptr, true, 0, false};
  handler.checkAndUndo(graph, info);

  EXPECT_TRUE(handler.mergedNodes().empty());
  EXPECT_TRUE(graph.hasNode("p1"_id));
  EXPECT_FALSE(graph.hasEdge("p2"_id, "p4"_id));
  EXPECT_TRUE(graph.hasEdge("p6"_id, "p4"_id));
}

// check that when undoing a merge with multiple children, we get a valid result out
TEST(MergeHandlerTests, TestUndoWithNewMerge) {
  DynamicSceneGraph graph;
  graph.emplaceNode(DsgLayers::PLACES, "p1"_id, std::make_unique<PlaceAttrs>(1.0, 2));
  graph.emplaceNode(DsgLayers::PLACES, "p2"_id, std::make_unique<PlaceAttrs>(1.0, 2));
  graph.emplaceNode(DsgLayers::PLACES, "p3"_id, std::make_unique<PlaceAttrs>(1.0, 2));
  graph.initMesh();

  std::map<NodeId, NodeId> proposed_merges{{"p2"_id, "p1"_id}, {"p3"_id, "p1"_id}};

  auto object_functor = std::make_shared<UpdateObjectsFunctor>();
  auto places_functor = std::make_shared<UpdatePlacesFunctor>(0.5, 0.1);
  MergeHandler handler(object_functor, places_functor, true);

  handler.updateMerges(proposed_merges, graph);
  EXPECT_EQ(handler.mergedNodes(), proposed_merges);

  gtsam::Values values;
  values.insert("p1"_id, gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(4.0, 5.0, 6.0)));
  values.insert("p2"_id, gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(7.0, 8.0, 9.0)));
  values.insert("p3"_id, gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(7.0, 8.0, 9.0)));

  // p2 should now be merged with p3
  UpdateInfo info{&values, nullptr, true, 0, false};
  handler.checkAndUndo(graph, info);

  std::map<NodeId, NodeId> expected_merges{{"p3"_id, "p2"_id}};
  EXPECT_EQ(handler.mergedNodes(), expected_merges);
  EXPECT_TRUE(graph.hasNode("p1"_id));
  EXPECT_TRUE(graph.hasNode("p2"_id));
}

// check that we don't undo a merge while it is still consistent (mostly for coverage)
TEST(MergeHandlerTests, TestUndoWithValidMerge) {
  DynamicSceneGraph graph;
  graph.emplaceNode(DsgLayers::PLACES, "p1"_id, std::make_unique<PlaceAttrs>(1.0, 2));
  graph.emplaceNode(DsgLayers::PLACES, "p2"_id, std::make_unique<PlaceAttrs>(1.0, 2));
  graph.initMesh();

  std::map<NodeId, NodeId> proposed_merges{{"p2"_id, "p1"_id}};

  auto object_functor = std::make_shared<UpdateObjectsFunctor>();
  auto places_functor = std::make_shared<UpdatePlacesFunctor>(0.5, 0.1);
  MergeHandler handler(object_functor, places_functor, true);

  handler.updateMerges(proposed_merges, graph);
  EXPECT_EQ(handler.mergedNodes(), proposed_merges);

  gtsam::Values values;
  values.insert("p1"_id, gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0.0, 0.0, 0.0)));
  values.insert("p2"_id, gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0.0, 0.0, 0.0)));

  // no merges should be undone
  UpdateInfo info{&values, nullptr, true, 0, false};
  handler.checkAndUndo(graph, info);

  EXPECT_EQ(handler.mergedNodes(), proposed_merges);
  EXPECT_TRUE(graph.hasNode("p1"_id));
  EXPECT_FALSE(graph.hasNode("p2"_id));
}

// check that reset works
TEST(MergeHandlerTests, TestReset) {
  DynamicSceneGraph graph;
  graph.emplaceNode(DsgLayers::PLACES, "p1"_id, std::make_unique<PlaceAttrs>(1.0, 2));
  graph.emplaceNode(DsgLayers::PLACES, "p2"_id, std::make_unique<PlaceAttrs>(1.0, 2));
  graph.initMesh();

  std::map<NodeId, NodeId> proposed_merges{{"p2"_id, "p1"_id}};

  auto object_functor = std::make_shared<UpdateObjectsFunctor>();
  auto places_functor = std::make_shared<UpdatePlacesFunctor>(0.5, 0.1);
  MergeHandler handler(object_functor, places_functor, true);

  handler.updateMerges(proposed_merges, graph);
  EXPECT_EQ(handler.mergedNodes(), proposed_merges);

  handler.reset();
  EXPECT_TRUE(handler.mergedNodes().empty());
  EXPECT_TRUE(handler.mergeParentNodes().empty());
}

// check that we need valid update functors (for coverage)
TEST(MergeHandlerTests, TestInvalidUpdaters) {
  try {
    MergeHandler handler(nullptr, nullptr, true);
    FAIL();
  } catch (const std::runtime_error&) {
    SUCCEED();
  }
}

}  // namespace hydra
