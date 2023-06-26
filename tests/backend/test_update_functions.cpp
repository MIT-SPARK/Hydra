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
#include <gtsam/geometry/Pose3.h>
#include <hydra/backend/update_functions.h>

namespace hydra {

using MeshVertices = DynamicSceneGraph::MeshVertices;
using MeshFaces = DynamicSceneGraph::MeshFaces;

#define MAKE_POINT(cloud, x_val, y_val, z_val) \
  {                                            \
    pcl::PointXYZRGBA point;                   \
    point.x = x_val;                           \
    point.y = y_val;                           \
    point.z = z_val;                           \
    cloud->push_back(point);                   \
  }                                            \
  static_assert(true, "")

namespace {

inline SharedDsgInfo::Ptr makeSharedDsg() {
  const LayerId mesh_layer_id = 1;
  const std::map<LayerId, char> layer_id_map{{DsgLayers::OBJECTS, 'o'},
                                             {DsgLayers::PLACES, 'p'},
                                             {DsgLayers::ROOMS, 'r'},
                                             {DsgLayers::BUILDINGS, 'b'}};
  return SharedDsgInfo::Ptr(new SharedDsgInfo(layer_id_map, mesh_layer_id));
}

}  // namespace

TEST(DsgInterpolationTests, ObjectUpdate) {
  auto dsg = makeSharedDsg();
  auto& graph = *dsg->graph;
  ObjectNodeAttributes::Ptr attrs(new ObjectNodeAttributes);
  attrs->position << 1.0, 2.0, 3.0;
  attrs->bounding_box.type = BoundingBox::Type::AABB;
  attrs->bounding_box.min << 1.0f, 2.0f, 3.0f;
  attrs->bounding_box.max << 1.0f, 2.0f, 3.0f;
  attrs->is_active = true;
  graph.emplaceNode(DsgLayers::OBJECTS, 0, std::move(attrs));

  const UpdateInfo info{nullptr, nullptr, false, 0, false};
  dsg_updates::UpdateObjectsFunctor functor;
  functor.call(*dsg, info);

  ObjectNodeAttributes& result =
      graph.getNode(0)->get().attributes<ObjectNodeAttributes>();

  {
    // No mesh, so nothing should change
    Eigen::Vector3d expected_pos(1.0, 2.0, 3.0);
    EXPECT_NEAR(0.0, (expected_pos - result.position).norm(), 1.0e-7);

    Eigen::Vector3f expected_min(1.0, 2.0, 3.0);
    EXPECT_NEAR(0.0, (expected_min - result.bounding_box.min).norm(), 1.0e-7);

    Eigen::Vector3f expected_max(1.0, 2.0, 3.0);
    EXPECT_NEAR(0.0, (expected_max - result.bounding_box.max).norm(), 1.0e-7);
  }

  MeshVertices::Ptr cloud(new MeshVertices);
  MAKE_POINT(cloud, -1.0, -2.0, -3.0);
  MAKE_POINT(cloud, 1.0, 2.0, 3.0);

  std::shared_ptr<MeshFaces> faces(new MeshFaces());
  graph.setMesh(cloud, faces);

  result.mesh_connections.push_back(0);
  result.mesh_connections.push_back(1);

  functor.call(*dsg, info);

  {
    // valid mesh: things should change
    Eigen::Vector3d expected_pos(0.0, 0.0, 0.0);
    EXPECT_NEAR(0.0, (expected_pos - result.position).norm(), 1.0e-7);

    Eigen::Vector3f expected_min(-1.0, -2.0, -3.0);
    EXPECT_NEAR(0.0, (expected_min - result.bounding_box.min).norm(), 1.0e-7);

    Eigen::Vector3f expected_max(1.0, 2.0, 3.0);
    EXPECT_NEAR(0.0, (expected_max - result.bounding_box.max).norm(), 1.0e-7);
  }
}

TEST(DsgInterpolationTests, ObjectUpdateMergeLC) {
  auto dsg = makeSharedDsg();
  auto& graph = *dsg->graph;
  ObjectNodeAttributes::Ptr attrs0(new ObjectNodeAttributes);
  attrs0->position << 1.0, 2.0, 3.0;
  attrs0->bounding_box.type = BoundingBox::Type::AABB;
  attrs0->bounding_box.min << 1.0f, 2.0f, 3.0f;
  attrs0->bounding_box.max << 1.0f, 2.0f, 3.0f;
  attrs0->semantic_label = 1u;
  attrs0->is_active = false;
  ObjectNodeAttributes::Ptr attrs1(new ObjectNodeAttributes);
  attrs1->position << 2.0, 3.0, 4.0;
  attrs1->bounding_box.type = BoundingBox::Type::AABB;
  attrs1->bounding_box.min << 2.0f, 3.0f, 4.0f;
  attrs1->bounding_box.max << 2.0f, 3.0f, 4.0f;
  attrs1->semantic_label = 1u;
  attrs1->is_active = true;
  graph.emplaceNode(DsgLayers::OBJECTS, 0, std::move(attrs0));
  graph.emplaceNode(DsgLayers::OBJECTS, 1, std::move(attrs1));

  const UpdateInfo info{nullptr, nullptr, true, 0, true};
  dsg_updates::UpdateObjectsFunctor functor;
  functor.call(*dsg, info);

  ObjectNodeAttributes& result0 =
      graph.getNode(0)->get().attributes<ObjectNodeAttributes>();
  ObjectNodeAttributes& result1 =
      graph.getNode(1)->get().attributes<ObjectNodeAttributes>();

  {
    // No mesh, so nothing should change
    Eigen::Vector3d expected_pos0(1.0, 2.0, 3.0);
    EXPECT_NEAR(0.0, (expected_pos0 - result0.position).norm(), 1.0e-7);

    Eigen::Vector3f expected_min0(1.0, 2.0, 3.0);
    EXPECT_NEAR(0.0, (expected_min0 - result0.bounding_box.min).norm(), 1.0e-7);

    Eigen::Vector3f expected_max0(1.0, 2.0, 3.0);
    EXPECT_NEAR(0.0, (expected_max0 - result0.bounding_box.max).norm(), 1.0e-7);

    Eigen::Vector3d expected_pos1(2.0, 3.0, 4.0);
    EXPECT_NEAR(0.0, (expected_pos1 - result1.position).norm(), 1.0e-7);

    Eigen::Vector3f expected_min1(2.0, 3.0, 4.0);
    EXPECT_NEAR(0.0, (expected_min1 - result1.bounding_box.min).norm(), 1.0e-7);

    Eigen::Vector3f expected_max1(2.0, 3.0, 4.0);
    EXPECT_NEAR(0.0, (expected_max1 - result1.bounding_box.max).norm(), 1.0e-7);
  }

  MeshVertices::Ptr cloud(new MeshVertices);
  MAKE_POINT(cloud, -1.0, -2.0, -3.0);
  MAKE_POINT(cloud, 1.0, 2.0, 3.0);

  result0.mesh_connections = {0, 1};
  result1.mesh_connections = {0, 1};

  std::shared_ptr<MeshFaces> faces(new MeshFaces());
  graph.setMesh(cloud, faces);
  auto merged_nodes = functor.call(*dsg, info);

  {
    // valid mesh: things should change
    Eigen::Vector3d expected_pos(0.0, 0.0, 0.0);
    EXPECT_NEAR(0.0, (expected_pos - result0.position).norm(), 1.0e-7);

    Eigen::Vector3f expected_min(-1.0, -2.0, -3.0);
    EXPECT_NEAR(0.0, (expected_min - result0.bounding_box.min).norm(), 1.0e-7);

    Eigen::Vector3f expected_max(1.0, 2.0, 3.0);
    EXPECT_NEAR(0.0, (expected_max - result0.bounding_box.max).norm(), 1.0e-7);

    std::map<NodeId, NodeId> expected{{1, 0}};
    EXPECT_EQ(merged_nodes, expected);
  }
}

TEST(DsgInterpolationTests, ObjectUpdateMergeNoLC) {
  auto dsg = makeSharedDsg();
  auto& graph = *dsg->graph;
  ObjectNodeAttributes::Ptr attrs0(new ObjectNodeAttributes);
  attrs0->position << 1.0, 2.0, 3.0;
  attrs0->bounding_box.type = BoundingBox::Type::AABB;
  attrs0->bounding_box.min << 1.0f, 2.0f, 3.0f;
  attrs0->bounding_box.max << 1.0f, 2.0f, 3.0f;
  attrs0->semantic_label = 1u;
  attrs0->mesh_connections = {0, 1};
  attrs0->is_active = false;
  ObjectNodeAttributes::Ptr attrs1(new ObjectNodeAttributes);
  attrs1->position << 2.0, 3.0, 4.0;
  attrs1->bounding_box.type = BoundingBox::Type::AABB;
  attrs1->bounding_box.min << 2.0f, 3.0f, 4.0f;
  attrs1->bounding_box.max << 2.0f, 3.0f, 4.0f;
  attrs1->semantic_label = 1u;
  attrs1->mesh_connections = {0, 1};
  attrs1->is_active = true;
  graph.emplaceNode(DsgLayers::OBJECTS, 0, std::move(attrs0));
  graph.emplaceNode(DsgLayers::OBJECTS, 1, std::move(attrs1));

  MeshVertices::Ptr cloud(new MeshVertices);
  MAKE_POINT(cloud, -1.0, -2.0, -3.0);
  MAKE_POINT(cloud, 1.0, 2.0, 3.0);

  std::shared_ptr<MeshFaces> faces(new MeshFaces());
  graph.setMesh(cloud, faces);

  const UpdateInfo info{nullptr, nullptr, false, 0, true};
  dsg_updates::UpdateObjectsFunctor functor;
  auto merged_nodes = functor.call(*dsg, info);

  {
    std::map<NodeId, NodeId> expected{{1, 0}};
    EXPECT_EQ(merged_nodes, expected);
  }
}

TEST(DsgInterpolationTests, BuildingUpdate) {
  auto dsg = makeSharedDsg();
  auto& graph = *dsg->graph;
  graph.emplaceNode(DsgLayers::BUILDINGS,
                    "B0"_id,
                    std::make_unique<NodeAttributes>(Eigen::Vector3d(1.0, 2.0, 3.0)));

  graph.emplaceNode(DsgLayers::ROOMS,
                    3,
                    std::make_unique<NodeAttributes>(Eigen::Vector3d(-1.0, 0.0, 1.0)));
  graph.emplaceNode(DsgLayers::ROOMS,
                    4,
                    std::make_unique<NodeAttributes>(Eigen::Vector3d(-1.0, 0.0, 1.0)));
  graph.emplaceNode(DsgLayers::ROOMS,
                    5,
                    std::make_unique<NodeAttributes>(Eigen::Vector3d(-1.0, 0.0, 1.0)));

  graph.insertEdge("B0"_id, 3);
  graph.insertEdge("B0"_id, 4);
  graph.insertEdge("B0"_id, 5);

  const UpdateInfo info{nullptr, nullptr, false, 0, false};
  dsg_updates::UpdateBuildingsFunctor functor(
      SemanticNodeAttributes::ColorVector::Zero(), 0);
  functor.call(*dsg, info);

  Eigen::Vector3d first_expected(-1.0, 0.0, 1.0);
  Eigen::Vector3d first_result = graph.getPosition("B0"_id);
  EXPECT_NEAR(0.0, (first_expected - first_result).norm(), 1.0e-7);
}

TEST(DsgInterpolationTests, PlaceUpdate) {
  const LayerId place_layer = DsgLayers::PLACES;
  auto dsg = makeSharedDsg();
  auto& graph = *dsg->graph;

  auto attrs1 = std::make_unique<PlaceNodeAttributes>(0.0, 0.0);
  attrs1->position = Eigen::Vector3d(1.0, 2.0, 3.0);
  graph.emplaceNode(place_layer, NodeSymbol('p', 0), std::move(attrs1));

  auto attrs2 = std::make_unique<PlaceNodeAttributes>(0.0, 0.0);
  attrs2->position = Eigen::Vector3d(1.0, 2.0, 3.0);
  graph.emplaceNode(place_layer, NodeSymbol('p', 5), std::move(attrs2));

  auto attrs3 = std::make_unique<PlaceNodeAttributes>(0.0, 0.0);
  attrs3->position = Eigen::Vector3d(1.0, 2.0, 3.0);
  attrs3->is_active = true;  // make sure it doesn't get dropped
  graph.emplaceNode(place_layer, NodeSymbol('p', 6), std::move(attrs3));

  gtsam::Values values;
  values.insert(NodeSymbol('p', 0),
                gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(4.0, 5.0, 6.0)));
  values.insert(NodeSymbol('p', 5),
                gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(7.0, 8.0, 9.0)));

  const UpdateInfo info{&values, nullptr, true, 0, false};
  dsg_updates::UpdatePlacesFunctor functor(0.4, 0.3);
  functor.call(*dsg, info);

  {  // first key exists: new value
    Eigen::Vector3d expected(4.0, 5.0, 6.0);
    Eigen::Vector3d result = graph.getPosition(NodeSymbol('p', 0));
    EXPECT_NEAR(0.0, (result - expected).norm(), 1.0e-7);
  }

  {  // non-zero key exists: new value
    Eigen::Vector3d expected(7.0, 8.0, 9.0);
    Eigen::Vector3d result = graph.getPosition(NodeSymbol('p', 5));
    EXPECT_NEAR(0.0, (result - expected).norm(), 1.0e-7);
  }

  {  // key doesn't exist: original value
    Eigen::Vector3d expected(1.0, 2.0, 3.0);
    Eigen::Vector3d result = graph.getPosition(NodeSymbol('p', 6));
    EXPECT_NEAR(0.0, (result - expected).norm(), 1.0e-7);
  }
}

TEST(DsgInterpolationTests, PlaceUpdateMerge) {
  const LayerId place_layer = DsgLayers::PLACES;
  auto dsg = makeSharedDsg();
  auto& graph = *dsg->graph;

  PlaceNodeAttributes::Ptr attrs0(new PlaceNodeAttributes);
  attrs0->position << 1.0, 2.0, 3.0;
  attrs0->distance = 1.0;
  attrs0->is_active = false;
  PlaceNodeAttributes::Ptr attrs5(new PlaceNodeAttributes);
  attrs5->position << 1.0, 2.0, 3.0;
  attrs5->distance = 1.0;
  attrs5->is_active = false;
  PlaceNodeAttributes::Ptr attrs6(new PlaceNodeAttributes);
  attrs6->position << 1.0, 2.0, 3.0;
  attrs6->distance = 1.0;
  attrs6->is_active = true;

  graph.emplaceNode(place_layer, NodeSymbol('p', 0), std::move(attrs0));
  graph.emplaceNode(place_layer, NodeSymbol('p', 5), std::move(attrs5));
  graph.emplaceNode(place_layer, NodeSymbol('p', 6), std::move(attrs6));

  gtsam::Values values;
  values.insert(NodeSymbol('p', 0),
                gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(4.0, 5.0, 6.0)));
  values.insert(NodeSymbol('p', 5),
                gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(7.0, 8.0, 9.0)));
  values.insert(NodeSymbol('p', 6),
                gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(7.0, 8.0, 9.0)));

  const UpdateInfo info{&values, nullptr, true, 0, true};
  dsg_updates::UpdatePlacesFunctor functor(0.4, 0.3);
  auto merged_nodes = functor.call(*dsg, info);

  {  // first key exists: new value
    Eigen::Vector3d expected(4.0, 5.0, 6.0);
    Eigen::Vector3d result = graph.getPosition(NodeSymbol('p', 0));
    EXPECT_NEAR(0.0, (result - expected).norm(), 1.0e-7);
  }

  {  // merge target key exists: new value
    Eigen::Vector3d expected(7.0, 8.0, 9.0);
    Eigen::Vector3d result = graph.getPosition(NodeSymbol('p', 5));
    EXPECT_NEAR(0.0, (result - expected).norm(), 1.0e-7);
  }

  // node p6 proposed for merge with node p5
  std::map<NodeId, NodeId> expected{{"p6"_id, "p5"_id}};
  EXPECT_EQ(merged_nodes, expected);
}

TEST(DsgInterpolationTests, AgentUpdate) {
  const LayerId agent_layer = DsgLayers::AGENTS;
  auto dsg = makeSharedDsg();
  auto& graph = *dsg->graph;
  {
    NodeAttributes::Ptr attrs =
        std::make_unique<AgentNodeAttributes>(Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0),
                                              Eigen::Vector3d(1.0, 2.0, 3.0),
                                              NodeSymbol('a', 0));
    graph.emplaceNode(agent_layer, 'a', std::chrono::seconds(1), std::move(attrs));
  }
  {
    NodeAttributes::Ptr attrs =
        std::make_unique<AgentNodeAttributes>(Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0),
                                              Eigen::Vector3d(1.0, 2.0, 3.0),
                                              NodeSymbol('a', 5));
    graph.emplaceNode(agent_layer, 'a', std::chrono::seconds(2), std::move(attrs));
  }
  {
    NodeAttributes::Ptr attrs =
        std::make_unique<AgentNodeAttributes>(Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0),
                                              Eigen::Vector3d(1.0, 2.0, 3.0),
                                              NodeSymbol('c', 5));
    graph.emplaceNode(agent_layer, 'b', std::chrono::seconds(2), std::move(attrs));
  }

  gtsam::Values agent_values;
  agent_values.insert(
      NodeSymbol('a', 0),
      gtsam::Pose3(gtsam::Rot3(0.0, 1.0, 0.0, 0.0), gtsam::Point3(4.0, 5.0, 6.0)));
  agent_values.insert(
      NodeSymbol('a', 5),
      gtsam::Pose3(gtsam::Rot3(0.0, 0.0, 1.0, 0.0), gtsam::Point3(7.0, 8.0, 9.0)));
  // unused
  agent_values.insert(
      NodeSymbol('b', 5),
      gtsam::Pose3(gtsam::Rot3(0.0, 0.0, 1.0, 0.0), gtsam::Point3(7.0, 8.0, 9.0)));

  const UpdateInfo info{nullptr, nullptr, false, 0, false, &agent_values};
  dsg_updates::updateAgents(*dsg, info);

  {  // external_key == node_id and in values
    const auto& attrs = graph.getDynamicNode(NodeSymbol('a', 0))
                            ->get()
                            .attributes<AgentNodeAttributes>();
    Eigen::Vector3d expected_pos(4.0, 5.0, 6.0);
    Eigen::Quaterniond expected_rot(0.0, 1.0, 0.0, 0.0);
    EXPECT_NEAR(0.0, (attrs.position - expected_pos).norm(), 1.0e-7);
    EXPECT_NEAR(0.0, attrs.world_R_body.angularDistance(expected_rot), 1.0e-7);
  }

  {  // external_key != node_id, but in values
    const auto& attrs = graph.getDynamicNode(NodeSymbol('a', 1))
                            ->get()
                            .attributes<AgentNodeAttributes>();
    Eigen::Vector3d expected_pos(7.0, 8.0, 9.0);
    Eigen::Quaterniond expected_rot(0.0, 0.0, 1.0, 0.0);
    EXPECT_NEAR(0.0, (attrs.position - expected_pos).norm(), 1.0e-7);
    EXPECT_NEAR(0.0, attrs.world_R_body.angularDistance(expected_rot), 1.0e-7);
  }

  {  // missing key in values, so doesn't get updated
    const auto& attrs = graph.getDynamicNode(NodeSymbol('b', 0))
                            ->get()
                            .attributes<AgentNodeAttributes>();
    Eigen::Vector3d expected_pos(1.0, 2.0, 3.0);
    Eigen::Quaterniond expected_rot(1.0, 0.0, 0.0, 0.0);
    EXPECT_NEAR(0.0, (attrs.position - expected_pos).norm(), 1.0e-7);
    EXPECT_NEAR(0.0, attrs.world_R_body.angularDistance(expected_rot), 1.0e-7);
  }
}

}  // namespace hydra
