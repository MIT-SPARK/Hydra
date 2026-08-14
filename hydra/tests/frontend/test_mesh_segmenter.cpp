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
#include <glog/logging.h>
#include <gtest/gtest.h>
#include <hydra/frontend/mesh_segmenter.h>
#include <hydra/utils/pgmo_mesh_traits.h>
#include <kimera_pgmo/mesh_delta.h>

namespace hydra {

using kimera_pgmo::MeshDelta;

namespace {

struct NodeResult {
  uint32_t label;
  std::list<size_t> mesh_connections;
  BoundingBox bbox;
  bool is_active = true;
};

void checkBoundingBox(const BoundingBox& expected,
                      const BoundingBox& result,
                      double tolerance = 1.0e-6f) {
  EXPECT_NEAR(expected.dimensions.x(), result.dimensions.x(), tolerance);
  EXPECT_NEAR(expected.dimensions.y(), result.dimensions.y(), tolerance);
  EXPECT_NEAR(expected.dimensions.z(), result.dimensions.z(), tolerance);
  EXPECT_NEAR(expected.world_P_center.x(), result.world_P_center.x(), tolerance);
  EXPECT_NEAR(expected.world_P_center.y(), result.world_P_center.y(), tolerance);
  EXPECT_NEAR(expected.world_P_center.z(), result.world_P_center.z(), tolerance);
}

bool checkNode(const DynamicSceneGraph& graph,
               NodeId node_id,
               const NodeResult& expected) {
  const auto node = graph.findNode(node_id);
  if (!node) {
    return false;
  }

  auto& result = node->attributes<ObjectNodeAttributes>();
  EXPECT_EQ(expected.label, result.semantic_label);
  EXPECT_EQ(expected.mesh_connections, result.mesh_connections);
  checkBoundingBox(expected.bbox, result.bounding_box);
  EXPECT_EQ(expected.is_active, result.is_active);
  return true;
}

void stepSegmenter(const MeshDelta& delta,
                   kimera_pgmo::MeshOffsetInfo& offsets,
                   MeshSegmenter& segmenter,
                   DynamicSceneGraph& graph) {
  delta.updateMesh(*graph.mesh(), offsets);
  const auto clusters = segmenter.detect(0, delta, offsets);
  segmenter.updateGraph(0, offsets, clusters, graph);
}

void addPoints(MeshDelta& delta,
               uint32_t label,
               const Eigen::Vector3f& offset,
               const Eigen::Vector3f& scale,
               bool archive = false) {
  const auto corners = BoundingBox(scale, offset).corners();
  for (const auto& corner : corners) {
    kimera_pgmo::traits::VertexTraits traits;
    traits.properties.has_label = true;
    traits.label = label;
    delta.addVertex(corner, traits, archive);
  }
}

}  // namespace

TEST(MeshSegmenter, TestClustering) {
  MeshSegmenter::Config config;
  config.clustering.min_cluster_size = 4;
  MeshSegmenter segmenter(config, {1, 2});

  MeshDelta delta({0, 0, 0});
  addPoints(delta, 1, {1, 2, 3}, Eigen::Vector3f::Constant(0.1));
  addPoints(delta, 2, {4, 5, 6}, Eigen::Vector3f::Constant(0.1));

  const auto clusters = segmenter.detect(0, delta, {0, 0, 0});
  ASSERT_EQ(clusters.size(), 2u);
}

TEST(MeshSegmenter, TestIndicesRemapping) {
  Eigen::Vector3f dims = Eigen::Vector3f::Constant(0.1);
  const BoundingBox b1(dims, Eigen::Vector3f(1, 2, 3));
  const BoundingBox b2(dims, Eigen::Vector3f(4, 5, 6));

  MeshSegmenter::Config config;
  config.clustering.min_cluster_size = 4;
  MeshSegmenter segmenter(config, {1, 2});

  DynamicSceneGraph graph;
  kimera_pgmo::MeshOffsetInfo offsets;
  graph.setMesh(std::make_shared<spark_dsg::Mesh>());

  {  // original objects
    MeshDelta delta({0, 0, 0});
    addPoints(delta, 1, {1, 2, 3}, dims);
    addPoints(delta, 2, {4, 5, 6}, dims);
    stepSegmenter(delta, offsets, segmenter, graph);

    std::unordered_set<NodeId> active{"O0"_id, "O1"_id};
    EXPECT_EQ(active, segmenter.getActiveNodes());

    const std::map<NodeId, NodeResult> expected_nodes{
        {"O0"_id, {1, {0, 1, 2, 3, 4, 5, 6, 7}, b1}},
        {"O1"_id, {2, {8, 9, 10, 11, 12, 13, 14, 15}, b2}},
    };

    for (const auto& [node_id, expected] : expected_nodes) {
      SCOPED_TRACE("Object " + NodeSymbol(node_id).str());
      EXPECT_TRUE(checkNode(graph, node_id, expected));
    }
  }

  {  // swapped objects
    std::map<size_t, size_t> remap;
    for (size_t i = 0; i < 8; ++i) {
      // two corner sets are swapped in order
      remap[i] = i + 8;
      remap[i + 8] = i;
    }

    const auto tracking = MeshDelta::TrackingInfo::with_remap(0, 16, 0, remap);
    MeshDelta delta(tracking);
    addPoints(delta, 2, {4, 5, 6}, Eigen::Vector3f::Constant(0.1));
    addPoints(delta, 1, {1, 2, 3}, Eigen::Vector3f::Constant(0.1));
    stepSegmenter(delta, offsets, segmenter, graph);

    const std::map<NodeId, NodeResult> expected_nodes{
        {"O0"_id, {1, {8, 9, 10, 11, 12, 13, 14, 15}, b1}},
        {"O1"_id, {2, {0, 1, 2, 3, 4, 5, 6, 7}, b2}},
    };

    for (const auto& [node_id, expected] : expected_nodes) {
      SCOPED_TRACE("Object " + NodeSymbol(node_id).str());
      EXPECT_TRUE(checkNode(graph, node_id, expected));
    }
  }
}

TEST(MeshSegmenter, TestDeletedObject) {
  Eigen::Vector3f dims = Eigen::Vector3f::Constant(0.1);
  const spark_dsg::BoundingBox b2(dims, Eigen::Vector3f(4, 5, 6));

  MeshSegmenter::Config config;
  config.clustering.min_cluster_size = 4;
  MeshSegmenter segmenter(config, {1, 2});

  DynamicSceneGraph graph;
  kimera_pgmo::MeshOffsetInfo offsets;
  graph.setMesh(std::make_shared<spark_dsg::Mesh>());

  {  // setup original objects
    MeshDelta delta({0, 0, 0});
    addPoints(delta, 1, {1, 2, 3}, dims);
    addPoints(delta, 2, {4, 5, 6}, dims);
    stepSegmenter(delta, offsets, segmenter, graph);
  }

  {  // delete object one
    std::map<size_t, size_t> remap;
    for (size_t i = 0; i < 8; ++i) {
      // previous object gets moved up
      remap[i + 8] = i;
    }

    const auto tracking = MeshDelta::TrackingInfo::with_remap(0, 16, 0, remap);
    MeshDelta delta(tracking);
    addPoints(delta, 2, {4, 5, 6}, dims);
    stepSegmenter(delta, offsets, segmenter, graph);

    const std::map<NodeId, NodeResult> expected{
        {"O1"_id, {2, {0, 1, 2, 3, 4, 5, 6, 7}, b2}}};
    for (const auto& [node_id, node] : expected) {
      SCOPED_TRACE("Object " + NodeSymbol(node_id).str());
      EXPECT_TRUE(checkNode(graph, node_id, node));
    }
  }
}

TEST(MeshSegmenter, TestArchivedObject) {
  Eigen::Vector3f dims = Eigen::Vector3f::Constant(0.1);
  const BoundingBox b1(dims, Eigen::Vector3f(1, 2, 3));
  const BoundingBox b2(dims, Eigen::Vector3f(4, 5, 6));

  MeshSegmenter::Config config;
  config.clustering.min_cluster_size = 4;
  MeshSegmenter segmenter(config, {1, 2});

  DynamicSceneGraph graph;
  kimera_pgmo::MeshOffsetInfo offsets;
  graph.setMesh(std::make_shared<spark_dsg::Mesh>());

  {  // setup original objects
    MeshDelta delta({0, 0, 0});
    addPoints(delta, 1, {1, 2, 3}, dims);
    addPoints(delta, 2, {4, 5, 6}, dims);
    stepSegmenter(delta, offsets, segmenter, graph);
  }

  {  // archive object two
    std::map<size_t, size_t> remap;
    for (size_t i = 0; i < 8; ++i) {
      // archived object gets moved down
      remap[i + 8] = i;
      // active object gets moved up
      remap[i] = i + 8;
    }

    const auto tracking = MeshDelta::TrackingInfo::with_remap(0, 16, 0, remap);
    MeshDelta delta(tracking);
    addPoints(delta, 2, {4, 5, 6}, dims, true);
    addPoints(delta, 1, {1, 2, 3}, dims);
    addPoints(delta, 2, {4, 5, 6}, dims);
    stepSegmenter(delta, offsets, segmenter, graph);

    const std::map<NodeId, NodeResult> expected_nodes{
        {"O0"_id, {1, {8, 9, 10, 11, 12, 13, 14, 15}, b1}},
        {"O1"_id, {2, {0, 1, 2, 3, 4, 5, 6, 7}, b2, false}},
        {"O2"_id, {2, {16, 17, 18, 19, 20, 21, 22, 23}, b2}},
    };
    for (const auto& [node_id, expected] : expected_nodes) {
      SCOPED_TRACE("Object " + NodeSymbol(node_id).str());
      EXPECT_TRUE(checkNode(graph, node_id, expected));
    }
  }
}

TEST(MeshSegmenter, TestDeltaWithOffset) {
  Eigen::Vector3f dims = Eigen::Vector3f::Constant(0.1);
  const BoundingBox b1(dims, Eigen::Vector3f(1, 2, 3));

  MeshSegmenter::Config config;
  config.clustering.min_cluster_size = 4;
  MeshSegmenter segmenter(config, {1, 2});

  DynamicSceneGraph graph;
  kimera_pgmo::MeshOffsetInfo offsets;
  graph.setMesh(std::make_shared<spark_dsg::Mesh>());

  {  // add 8 archived vertices to mesh
    MeshDelta delta({0, 0, 0});
    addPoints(delta, 1, {1, 2, 3}, dims, true);
    delta.updateMesh(*graph.mesh(), offsets);
  }

  {  // add actual object
    MeshDelta delta({0, 0, 0});
    addPoints(delta, 1, {1, 2, 3}, dims);
    stepSegmenter(delta, offsets, segmenter, graph);

    const std::map<NodeId, NodeResult> expected_nodes{
        {"O0"_id, {1, {8, 9, 10, 11, 12, 13, 14, 15}, b1}},
    };
    for (const auto& [node_id, expected] : expected_nodes) {
      SCOPED_TRACE("Object " + NodeSymbol(node_id).str());
      EXPECT_TRUE(checkNode(graph, node_id, expected));
    }
  }
}

}  // namespace hydra
