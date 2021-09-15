#include <kimera_dsg_builder/dsg_update_functions.h>

#include <gtsam/geometry/Pose3.h>

#include <gtest/gtest.h>

namespace kimera {

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

TEST(DsgInterpolationTests, ObjectUpdate) {
  DynamicSceneGraph graph;
  ObjectNodeAttributes::Ptr attrs(new ObjectNodeAttributes);
  attrs->position << 1.0, 2.0, 3.0;
  attrs->bounding_box.type = BoundingBox::Type::AABB;
  attrs->bounding_box.min << 1.0f, 2.0f, 3.0f;
  attrs->bounding_box.max << 1.0f, 2.0f, 3.0f;
  graph.emplaceNode(
      static_cast<LayerId>(KimeraDsgLayers::OBJECTS), 0, std::move(attrs));

  gtsam::Values values;
  dsg_updates::updateObjects(graph, values, values);

  const ObjectNodeAttributes& result =
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

  graph.insertMeshEdge(0, 0);
  graph.insertMeshEdge(0, 1);

  dsg_updates::updateObjects(graph, values, values);

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

TEST(DsgInterpolationTests, BuildingUpdate) {
  DynamicSceneGraph graph;
  graph.emplaceNode(static_cast<LayerId>(KimeraDsgLayers::BUILDINGS),
                    0,
                    std::make_unique<NodeAttributes>(Eigen::Vector3d(1.0, 2.0, 3.0)));

  graph.emplaceNode(static_cast<LayerId>(KimeraDsgLayers::BUILDINGS),
                    1,
                    std::make_unique<NodeAttributes>(Eigen::Vector3d(4.0, 5.0, 6.0)));

  graph.emplaceNode(static_cast<LayerId>(KimeraDsgLayers::BUILDINGS),
                    2,
                    std::make_unique<NodeAttributes>(Eigen::Vector3d(4.0, 5.0, 6.0)));

  graph.emplaceNode(
      2, 3, std::make_unique<NodeAttributes>(Eigen::Vector3d(-1.0, 0.0, 1.0)));
  graph.emplaceNode(
      2, 4, std::make_unique<NodeAttributes>(Eigen::Vector3d(-1.0, 0.0, 1.0)));
  graph.emplaceNode(
      2, 5, std::make_unique<NodeAttributes>(Eigen::Vector3d(-1.0, 0.0, 1.0)));
  graph.emplaceNode(
      2, 6, std::make_unique<NodeAttributes>(Eigen::Vector3d(-1.0, 0.5, 1.0)));
  graph.emplaceNode(
      2, 7, std::make_unique<NodeAttributes>(Eigen::Vector3d(0.0, 0.0, 0.0)));
  graph.emplaceNode(
      2, 8, std::make_unique<NodeAttributes>(Eigen::Vector3d(1.0, -0.5, -1.0)));

  graph.insertEdge(0, 3);
  graph.insertEdge(0, 4);
  graph.insertEdge(0, 5);
  graph.insertEdge(1, 6);
  graph.insertEdge(1, 7);
  graph.insertEdge(1, 8);

  graph.insertEdge(0, 1);
  graph.insertEdge(0, 2);
  graph.insertEdge(1, 2);

  gtsam::Values values;
  dsg_updates::updateBuildings(graph, values, values);

  Eigen::Vector3d first_expected(-1.0, 0.0, 1.0);
  Eigen::Vector3d first_result = graph.getPosition(0);
  EXPECT_NEAR(0.0, (first_expected - first_result).norm(), 1.0e-7);

  Eigen::Vector3d second_expected(0.0, 0.0, 0.0);
  Eigen::Vector3d second_result = graph.getPosition(1);
  EXPECT_NEAR(0.0, (second_expected - second_result).norm(), 1.0e-7);

  Eigen::Vector3d third_expected(4.0, 5.0, 6.0);
  Eigen::Vector3d third_result = graph.getPosition(2);
  EXPECT_NEAR(0.0, (third_expected - third_result).norm(), 1.0e-7);
}

TEST(DsgInterpolationTests, PlaceUpdate) {
  const LayerId place_layer = static_cast<LayerId>(KimeraDsgLayers::PLACES);
  DynamicSceneGraph graph;
  graph.emplaceNode(place_layer,
                    NodeSymbol('p', 0),
                    std::make_unique<NodeAttributes>(Eigen::Vector3d(1.0, 2.0, 3.0)));
  graph.emplaceNode(place_layer,
                    NodeSymbol('p', 5),
                    std::make_unique<NodeAttributes>(Eigen::Vector3d(1.0, 2.0, 3.0)));
  graph.emplaceNode(place_layer,
                    NodeSymbol('p', 6),
                    std::make_unique<NodeAttributes>(Eigen::Vector3d(1.0, 2.0, 3.0)));

  gtsam::Values values;
  values.insert(NodeSymbol('p', 0),
                gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(4.0, 5.0, 6.0)));
  values.insert(NodeSymbol('p', 5),
                gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(7.0, 8.0, 9.0)));

  gtsam::Values pgmo_values;
  dsg_updates::updatePlaces(graph, values, pgmo_values);

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

TEST(DsgInterpolationTests, AgentUpdate) {
  const LayerId agent_layer = static_cast<LayerId>(KimeraDsgLayers::AGENTS);
  DynamicSceneGraph graph;
  {
    NodeAttributes::Ptr attrs =
        std::make_unique<AgentNodeAttributes>(Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0),
                                              Eigen::Vector3d(1.0, 2.0, 3.0),
                                              NodeSymbol('a', 0));
    graph.emplaceDynamicNode(
        agent_layer, 'a', std::chrono::seconds(1), std::move(attrs));
  }
  {
    NodeAttributes::Ptr attrs =
        std::make_unique<AgentNodeAttributes>(Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0),
                                              Eigen::Vector3d(1.0, 2.0, 3.0),
                                              NodeSymbol('a', 5));
    graph.emplaceDynamicNode(
        agent_layer, 'a', std::chrono::seconds(2), std::move(attrs));
  }
  {
    NodeAttributes::Ptr attrs =
        std::make_unique<AgentNodeAttributes>(Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0),
                                              Eigen::Vector3d(1.0, 2.0, 3.0),
                                              NodeSymbol('c', 5));
    graph.emplaceDynamicNode(
        agent_layer, 'b', std::chrono::seconds(2), std::move(attrs));
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

  gtsam::Values places_values;
  dsg_updates::updateAgents(graph, places_values, agent_values);

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

}  // namespace kimera
