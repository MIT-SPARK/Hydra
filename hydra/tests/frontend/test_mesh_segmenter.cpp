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
#include <hydra/common/global_info.h>
#include <hydra/frontend/graph_builder.h>
#include <hydra/frontend/mesh_delta_clustering.h>
#include <hydra/frontend/mesh_segmenter.h>
#include <hydra/utils/pgmo_mesh_traits.h>

#include <array>
#include <future>
#include <numeric>
#include <random>

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
  ActiveWindowOutput result(std::make_shared<InputData>(nullptr));
  result.timestamp_ns = timestamp;
  result.archived = archived;
  result.setMap(map);
  return result;
}

// Test sequences are individual frontend packets, never composed deltas.
struct Packet {
  uint64_t timestamp_ns;
  BlockIndices archived;
  std::vector<MeshBlock::ConstPtr> blocks;
};
void detect(MeshSegmenter& segmenter, const std::vector<Packet>& packets) {
  for (const auto& packet : packets) {
    segmenter.update(output(packet.timestamp_ns, packet.blocks, packet.archived));
  }
}

struct Pipeline {
  static MeshSegmenter::Config config() {
    MeshSegmenter::Config c;
    c.clustering.min_cluster_size = 4;
    c.association_tolerance = 0.08;
    return c;
  }
  explicit Pipeline(const MeshSegmenter::Config& config = Pipeline::config())
      : segmenter(config, {1, 2}), compression(0.005) {
    graph.setMesh(std::make_shared<Mesh>());
    replay.setMesh(std::make_shared<Mesh>());
  }
  MeshSegmenter segmenter;
  kimera_pgmo::DeltaCompression compression;
  MeshUpdateInfo mesh_update;
  SharedDsgInfo shared{SharedDsgInfo::Config{}};
  SceneGraph& graph = *shared.graph;
  SceneGraph replay;
  const kimera_pgmo::MeshOffsetInfo& offsets = mesh_update.offsets;
  kimera_pgmo::MeshOffsetInfo replay_offsets;

  void step(const std::vector<Packet>& packets) {
    for (const auto& packet : packets) {
      const auto input = output(packet.timestamp_ns, packet.blocks, packet.archived);
      // Exercise the actual independence of clustering and compression.
      const auto previous_objects = graph.getLayer(DsgLayers::OBJECTS).nodes().size();
      FrontendOutput result(packet.timestamp_ns, 1);
      GraphBuilderFunctor& functor = segmenter;
      auto worker = std::async(std::launch::async,
                               [&] { functor.call(input, shared, result, nullptr); });
      auto delta = hydra::updateMesh(compression, input, *graph.mesh(), mesh_update);
      worker.get();
      EXPECT_EQ(graph.getLayer(DsgLayers::OBJECTS).nodes().size(), previous_objects);
      functor.callPostUpdate(shared, result, mesh_update);
      delta->updateMesh(*replay.mesh(), replay_offsets);
      ASSERT_EQ(graph.mesh()->numVertices(), replay.mesh()->numVertices());
      ASSERT_EQ(graph.mesh()->faces, replay.mesh()->faces);
      EXPECT_EQ(offsets.archived_vertices, replay_offsets.archived_vertices);
      EXPECT_EQ(offsets.archived_faces, replay_offsets.archived_faces);
      for (size_t i = 0; i < graph.mesh()->numVertices(); ++i) {
        EXPECT_EQ(graph.mesh()->pos(i), replay.mesh()->pos(i));
        EXPECT_EQ(graph.mesh()->label(i), replay.mesh()->label(i));
      }
      for (const auto& [id, node] : graph.getLayer(DsgLayers::OBJECTS).nodes()) {
        const auto& attrs = node->attributes<ObjectNodeAttributes>();
        for (const auto i : attrs.mesh_connections) {
          EXPECT_LT(i, graph.mesh()->numVertices());
          if (!attrs.is_active) {
            EXPECT_LT(i, offsets.archived_vertices);
          }
        }
      }
    }
  }
  const ObjectNodeAttributes& attrs(NodeId id) const {
    return graph.getNode(id).attributes<ObjectNodeAttributes>();
  }
};
}  // namespace

TEST(GraphBuilder, PostUpdateRunsAllFunctorsAfterCallbacksAndCompression) {
  struct Probe : GraphBuilderFunctor {
    Probe(std::atomic<size_t>& completed,
          const std::string& name,
          std::vector<std::string>& order)
        : name(name), order(order), completed(completed) {}
    const std::string name;
    std::vector<std::string>& order;
    std::atomic<size_t>& completed;
    size_t post_calls = 0;

    void call(const ActiveWindowOutput&,
              SharedDsgInfo&,
              FrontendOutput&,
              const VolumetricWindow*) override {
      ++completed;
    }

    void callPostUpdate(SharedDsgInfo& dsg,
                        FrontendOutput& output,
                        const MeshUpdateInfo& info) override {
      EXPECT_EQ(completed.load(), 6u);
      order.push_back(name);
      ++post_calls;
      if (output.timestamp_ns == 1) {
        ASSERT_EQ(info.blocks.size(), 1u);
        EXPECT_TRUE(info.archived_blocks.empty());
        EXPECT_EQ(info.offsets.archived_vertices, 0u);
        const auto& mapping = info.blocks.begin()->second;
        ASSERT_EQ(mapping.vertices.size(), 8u);
        for (size_t i = 0; i < mapping.vertices.size(); ++i) {
          ASSERT_TRUE(mapping.vertices[i]);
          EXPECT_EQ(dsg.graph->mesh()->pos(*mapping.vertices[i]),
                    mapping.block->pos(i));
        }
      } else {
        EXPECT_TRUE(info.blocks.empty());
        EXPECT_EQ(info.archived_blocks.size(), 1u);
        EXPECT_EQ(info.offsets.archived_vertices, 8u);
      }
    }
  };
  struct Builder : GraphBuilder {
    using GraphBuilder::GraphBuilder;
    std::vector<std::string> order;
    void install(std::atomic<size_t>& completed) {
      for (auto& [name, functor] : functors_) {
        functor = std::make_unique<Probe>(completed, name, order);
      }
    }
    void run(const ActiveWindowOutput& input) {
      order.clear();
      curr_output_ = std::make_shared<FrontendOutput>(input.timestamp_ns, 1);
      updateImpl(std::make_shared<ActiveWindowOutput>(input));
      ASSERT_EQ(order.size(), 6u);
      EXPECT_EQ(order.front(), "objects");
    }
    void checkCalls() const {
      for (const auto& [name, functor] : functors_) {
        EXPECT_EQ(static_cast<const Probe&>(*functor).post_calls, 2u);
      }
    }
  };
  GlobalInfo::init(PipelineConfig{});
  {
    std::atomic<size_t> completed{0};
    auto shared = std::make_shared<SharedDsgInfo>(SharedDsgInfo::Config{});
    Builder builder(
        GraphBuilder::Config{}, shared, std::make_shared<SharedModuleState>());
    builder.install(completed);
    builder.run(output(1, {box({0, 0, 0}, {0, 0, 0})}));
    completed = 0;
    builder.run(output(2, {}, {{0, 0, 0}}));
    builder.checkCalls();
  }
  GlobalInfo::reset();
}

TEST(MeshSegmenter, LabelsUnchangedBlocksAndRemeshing) {
  Pipeline p;
  auto a = box({0, 0, 0}, {0, 0, 0});
  auto b = box({1, 0, 0}, {1, 0, 0}, 2);
  p.step({{1, {}, {a, b}}});
  ASSERT_EQ(p.segmenter.getActiveNodes().size(), 2u);
  auto replacement = box({0, 0, 0}, {0.01f, 0, 0});
  std::reverse(replacement->points.begin(), replacement->points.end());
  p.step({{2, {}, {replacement}}});
  EXPECT_EQ(p.segmenter.getActiveNodes(),
            (std::unordered_set<NodeId>{"O0"_id, "O1"_id}));
  EXPECT_EQ(p.attrs("O0"_id).mesh_connections.size(), 8u);
  EXPECT_NEAR(p.attrs("O0"_id).position.x(), 0.01, 1e-6);
  p.step({{3, {}, {}}});
  EXPECT_EQ(p.attrs("O1"_id).mesh_connections.size(), 8u);
}

TEST(MeshSegmenter, ReplacementDeletesAndRelabelsSupport) {
  Pipeline p;
  p.step({{1, {}, {box({0, 0, 0}, {0, 0, 0})}}});
  p.step({{2, {}, {box({0, 0, 0}, {0, 0, 0}, 2)}}});
  EXPECT_FALSE(p.graph.hasNode("O0"_id));
  EXPECT_EQ(p.attrs("O1"_id).semantic_label, 2u);
  p.step({{3, {}, {block({0, 0, 0}, {})}}});
  EXPECT_FALSE(p.graph.hasNode("O1"_id));
}

TEST(MeshSegmenter, PartialArchivalKeepsSmallActiveFragment) {
  Pipeline p;
  auto a = block({0, 0, 0}, {{0, 0, 0}, {0.05f, 0, 0}, {0.1f, 0, 0}, {0.15f, 0, 0}});
  auto b = block({1, 0, 0}, {{0.2f, 0, 0}});
  p.step({{1, {}, {a, b}}});
  p.step({{2, {{0, 0, 0}}, {}}});
  EXPECT_TRUE(p.attrs("O0"_id).is_active);
  EXPECT_EQ(p.attrs("O0"_id).mesh_connections.size(), 5u);
  p.step({{3, {{1, 0, 0}}, {}}});
  EXPECT_FALSE(p.attrs("O0"_id).is_active);
  EXPECT_EQ(p.attrs("O0"_id).mesh_connections.size(), 5u);
}

TEST(MeshSegmenter, ArchiveThenReentryInSeparatePackets) {
  Pipeline p;
  p.step({{1, {}, {box({0, 0, 0}, {0, 0, 0})}}});
  p.step({{2, {{0, 0, 0}}, {}}});
  p.step({{3, {}, {box({0, 0, 0}, {0, 0, 0})}}});
  EXPECT_FALSE(p.attrs("O0"_id).is_active);
  EXPECT_TRUE(p.attrs("O1"_id).is_active);
  EXPECT_NE(p.attrs("O0"_id).mesh_connections, p.attrs("O1"_id).mesh_connections);
}

TEST(MeshSegmenter, SharedVerticesAndPendingFacesSurviveSequentialUpdates) {
  Pipeline p;
  auto a = block({0, 0, 0}, {{0, 0, 0}, {0.1f, 0, 0}, {0, 0.1f, 0}, {0.1f, 0.1f, 0}});
  a->faces = {{0, 1, 2}, {1, 2, 3}};
  auto b =
      block({1, 0, 0}, {{0.1f, 0, 0}, {0.1f, 0.1f, 0}, {0.2f, 0, 0}, {0.2f, 0.1f, 0}});
  b->faces = {{0, 1, 2}, {1, 2, 3}};
  p.step({{1, {}, {a, b}}});
  p.step({{2, {{0, 0, 0}}, {}}});
  p.step({{3, {{1, 0, 0}}, {}}, {4, {}, {a}}, {5, {{0, 0, 0}}, {}}, {6, {}, {b}}});
  p.step({{7, {{1, 0, 0}}, {}}});
}

TEST(MeshSegmenter, DuplicateVerticesDoNotInflateCreationThreshold) {
  Pipeline p;
  p.step({{1, {}, {block({0, 0, 0}, {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}})}}});
  EXPECT_TRUE(p.segmenter.getActiveNodes().empty());
}

TEST(MeshSegmenter, DeterministicMergeAndSplit) {
  auto config = Pipeline::config();
  config.clustering.min_cluster_size = 2;
  config.clustering.cluster_tolerance = 0.11;
  config.min_overlap_ratio = 0.1;
  config.association_tolerance = 0.04;
  Pipeline p(config);
  p.step(
      {{1,
        {},
        {block({0, 0, 0}, {{0, 0, 0}, {0.05f, 0, 0}, {0.6f, 0, 0}, {0.65f, 0, 0}})}}});
  EXPECT_EQ(p.segmenter.getActiveNodes().size(), 2u);
  p.step({{2,
           {},
           {block({0, 0, 0},
                  {{0, 0, 0},
                   {0.05f, 0, 0},
                   {0.15f, 0, 0},
                   {0.25f, 0, 0},
                   {0.35f, 0, 0},
                   {0.45f, 0, 0},
                   {0.55f, 0, 0},
                   {0.6f, 0, 0},
                   {0.65f, 0, 0}})}}});
  EXPECT_EQ(p.segmenter.getActiveNodes().size(), 1u);
  EXPECT_FALSE(p.graph.hasNode("O1"_id));
  p.step(
      {{3,
        {},
        {block({0, 0, 0}, {{0, 0, 0}, {0.05f, 0, 0}, {0.6f, 0, 0}, {0.65f, 0, 0}})}}});
  EXPECT_EQ(p.segmenter.getActiveNodes(),
            (std::unordered_set<NodeId>{"O0"_id, "O2"_id}));
}

TEST(MeshSegmenter, OversizedComponentIsNotSplitIntoAcceptedFragments) {
  auto config = Pipeline::config();
  config.clustering.min_cluster_size = 2;
  config.clustering.max_cluster_size = 3;
  Pipeline p(config);
  p.step({{1,
           {},
           {block({0, 0, 0},
                  {{0, 0, 0},
                   {0.1f, 0, 0},
                   {0.2f, 0, 0},
                   {0.3f, 0, 0},
                   {2, 0, 0},
                   {2.1f, 0, 0}})}}});
  ASSERT_EQ(p.segmenter.getActiveNodes().size(), 1u);
  EXPECT_EQ(p.attrs("O0"_id).mesh_connections.size(), 2u);
}

TEST(MeshSegmenter, SharedVerticesDelayObjectArchival) {
  Pipeline p;
  p.step({{1, {}, {box({0, 0, 0}, {0, 0, 0}), box({1, 0, 0}, {0, 0, 0}, 2)}}});
  p.step({{2, {{0, 0, 0}}, {}}});
  ASSERT_TRUE(p.attrs("O0"_id).is_active);
  EXPECT_FALSE(p.segmenter.getActiveNodes().count("O0"_id));
  ASSERT_FALSE(p.attrs("O0"_id).mesh_connections.empty());
  EXPECT_GE(p.attrs("O0"_id).mesh_connections.back(), p.offsets.archived_vertices);
  auto shared = p.graph.clone();
  auto backend = p.graph.clone();
  // Insert and archive an unrelated block, moving all shared active vertices.
  p.step({{3, {}, {box({2, 0, 0}, {2, 0, 0})}}, {4, {{2, 0, 0}}, {}}});
  EXPECT_TRUE(p.attrs("O0"_id).is_active);
  EXPECT_FALSE(p.segmenter.objects().count("O0"_id));
  EXPECT_GE(p.attrs("O0"_id).mesh_connections.back(), p.offsets.archived_vertices);
  shared->mergeGraph(p.graph);
  backend->mergeGraph(*shared);
  EXPECT_EQ(
      backend->getNode("O0"_id).attributes<ObjectNodeAttributes>().mesh_connections,
      p.attrs("O0"_id).mesh_connections);
  EXPECT_TRUE(backend->getNode("O0"_id).attributes().is_active);
  for (const auto i : p.attrs("O0"_id).mesh_connections) {
    EXPECT_LT(p.graph.mesh()->pos(i).norm(), 0.2);
  }
  p.step({{5, {{1, 0, 0}}, {}}});
  EXPECT_FALSE(p.attrs("O0"_id).is_active);
  EXPECT_EQ(p.attrs("O0"_id).mesh_connections.size(), 8u);
  shared->mergeGraph(p.graph);
  backend->mergeGraph(*shared);
  EXPECT_FALSE(backend->getNode("O0"_id).attributes().is_active);
  EXPECT_EQ(
      backend->getNode("O0"_id).attributes<ObjectNodeAttributes>().mesh_connections,
      p.attrs("O0"_id).mesh_connections);
}

TEST(MeshSegmenter, SpatialHashMatchesPclEuclideanComponents) {
  auto config = Pipeline::config();
  config.clustering.cluster_tolerance = 0.12;
  MeshSegmenter segmenter(config, {1});
  std::mt19937 rng(42);
  std::uniform_real_distribution<float> jitter(-0.2f, 0.2f);
  std::vector<Eigen::Vector3f> points;
  for (int group = -3; group < 3; ++group) {
    for (int i = 0; i < 80; ++i) {
      points.emplace_back(group + jitter(rng), jitter(rng), jitter(rng));
    }
  }
  auto input = block({0, 0, 0}, points);
  detect(segmenter, {{1, {}, {input}}});
  kimera_pgmo::MeshDelta delta({});
  for (const auto& point : points) {
    kimera_pgmo::traits::VertexTraits traits;
    traits.properties.has_label = true;
    traits.label = 1;
    delta.addVertex(point, traits);
  }
  std::vector<size_t> indices(points.size());
  std::iota(indices.begin(), indices.end(), 0);
  const clustering::ClusteringConfig pcl_config{config.clustering.cluster_tolerance,
                                                config.clustering.min_cluster_size,
                                                indices.size()};
  const auto expected = clustering::findClusters(pcl_config, delta, indices);
  using Component = std::set<std::array<float, 3>>;
  std::set<Component> reference, actual;
  for (const auto& cluster : expected) {
    Component component;
    for (const auto i : cluster) {
      component.insert({points[i].x(), points[i].y(), points[i].z()});
    }
    reference.insert(component);
  }
  for (const auto& [id, object] : segmenter.objects()) {
    Component component;
    for (const auto& point : object.points) {
      component.insert({point.x(), point.y(), point.z()});
    }
    actual.insert(component);
  }
  EXPECT_EQ(reference, actual);
}

TEST(MeshSegmenter, NearbyAndInvalidVertices) {
  auto config = Pipeline::config();
  config.clustering.min_cluster_size = 1;
  config.clustering.cluster_tolerance = 0.25;
  MeshSegmenter segmenter(config, {1});
  auto input = block({0, 0, 0},
                     {{-0.249f, 0, 0},
                      {0, 0, 0},
                      {0.24f, 0.24f, 0},
                      {std::numeric_limits<float>::quiet_NaN(), 0, 0}});
  detect(segmenter, {{1, {}, {input}}});
  ASSERT_EQ(segmenter.objects().size(), 2u);
  EXPECT_EQ(segmenter.objects().at("O0"_id).points.size(), 2u);
  EXPECT_EQ(segmenter.objects().at("O1"_id).points.size(), 1u);
  auto unlabeled = std::make_shared<MeshBlock>(1.0f, BlockIndex(0, 0, 0));
  unlabeled->resizeVertices(1);
  unlabeled->setPos(0, {0, 0, 0});
  detect(segmenter, {{2, {}, {unlabeled}}});
  EXPECT_TRUE(segmenter.getActiveNodes().empty());
}

TEST(MeshSegmenter, MergeRetainsBothParentsArchivedHistory) {
  auto config = Pipeline::config();
  config.clustering.min_cluster_size = 2;
  config.clustering.cluster_tolerance = 0.11;
  config.association_tolerance = 0.04;
  config.min_overlap_ratio = 0.1;
  Pipeline p(config);
  p.step({{1,
           {},
           {block({0, 0, 0}, {{0, 0, 0}, {0.05f, 0, 0}}),
            block({1, 0, 0}, {{0.1f, 0, 0}, {0.15f, 0, 0}}),
            block({2, 0, 0}, {{0.7f, 0, 0}, {0.75f, 0, 0}}),
            block({3, 0, 0}, {{0.8f, 0, 0}, {0.85f, 0, 0}})}}});
  p.step({{2, {{0, 0, 0}, {2, 0, 0}}, {}}});
  ASSERT_EQ(p.segmenter.getActiveNodes().size(), 2u);
  p.step({{3,
           {},
           {block({1, 0, 0},
                  {{0.1f, 0, 0},
                   {0.15f, 0, 0},
                   {0.25f, 0, 0},
                   {0.35f, 0, 0},
                   {0.45f, 0, 0},
                   {0.55f, 0, 0},
                   {0.65f, 0, 0},
                   {0.75f, 0, 0},
                   {0.8f, 0, 0}})}}});
  ASSERT_EQ(p.segmenter.getActiveNodes().size(), 1u);
  const auto id = *p.segmenter.getActiveNodes().begin();
  size_t archived = 0;
  for (const auto index : p.attrs(id).mesh_connections) {
    archived += index < p.offsets.archived_vertices;
  }
  EXPECT_EQ(archived, 4u);
}

TEST(MeshSegmenter, NearDuplicatesAcrossBlocksKeepSourcesAndStableOrdering) {
  MeshSegmenter::Config config;
  config.clustering.min_cluster_size = 2;
  config.vertex_merge_tolerance_m = 1.0e-5;
  const auto a = block({0, 0, 0}, {{-1.0e-6f, 0, 0}, {0.05f, 0, 0}});
  const auto b = block({1, 0, 0}, {{1.0e-6f, 0, 0}, {0.050001f, 0, 0}});
  MeshSegmenter segmenter(config, {1});
  segmenter.update(output(1, {a, b}));
  ASSERT_EQ(segmenter.objects().size(), 1u);
  const auto& object = segmenter.objects().begin()->second;
  EXPECT_EQ(object.points.size(), 2u);
  EXPECT_EQ(object.vertices.size(), 4u);
  const auto expected = object.points;
  MeshSegmenter reversed(config, {1});
  reversed.update(output(1, {b, a}));
  ASSERT_EQ(reversed.objects().size(), 1u);
  EXPECT_EQ(reversed.objects().begin()->second.points, expected);

  segmenter.update(output(2, {}, {a->index}));
  ASSERT_EQ(segmenter.objects().size(), 1u);
  EXPECT_EQ(segmenter.objects().begin()->second.vertices.size(), 2u);
  EXPECT_EQ(segmenter.objects().begin()->second.archived_vertices.size(), 2u);
  EXPECT_EQ(segmenter.objects().begin()->second.points.size(), 2u);
}

TEST(MeshSegmenter, MergeToleranceControlsCreationAndPreservesLabels) {
  const auto a = block({0, 0, 0}, {{-1.0e-6f, 0, 0}});
  const auto b = block({1, 0, 0}, {{1.0e-6f, 0, 0}});
  MeshSegmenter::Config config;
  config.clustering.min_cluster_size = 2;
  MeshSegmenter merged(config, {1});
  merged.update(output(1, {a, b}));
  EXPECT_TRUE(merged.objects().empty());
  config.vertex_merge_tolerance_m = 0;
  MeshSegmenter exact(config, {1});
  exact.update(output(1, {a, b}));
  ASSERT_EQ(exact.objects().size(), 1u);
  EXPECT_EQ(exact.objects().begin()->second.points.size(), 2u);

  config.vertex_merge_tolerance_m = 1.0e-5;
  config.clustering.min_cluster_size = 1;
  MeshSegmenter labels(config, {1, 2});
  const auto c = block({2, 0, 0}, {{1.0e-6f, 0, 0}}, 2);
  labels.update(output(1, {a, c}));
  EXPECT_EQ(labels.objects().size(), 2u);
}

}  // namespace hydra
