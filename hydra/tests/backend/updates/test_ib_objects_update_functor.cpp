#include <gtest/gtest.h>
#include <hydra/backend/updates/ib_objects_update_functor.h>

using namespace spark_dsg;

namespace hydra {
namespace {

Eigen::VectorXf getOneHot(size_t i, size_t dim) {
  Eigen::VectorXf p = Eigen::VectorXf::Zero(dim);
  p(i) = 1.0;
  return p;
}

void addSegment(SceneGraph& graph,
                size_t index,
                double min,
                double max,
                std::optional<size_t> onehot_index = std::nullopt) {
  Eigen::Vector3f x_min(min, -1.0, -1.0);
  Eigen::Vector3f x_max(max, 1.0, 1.0);

  auto attrs = std::make_unique<KhronosObjectAttributes>();
  attrs->position << (min + max) / 2.0, 0.0, 0.0;
  attrs->bounding_box = BoundingBox(x_max - x_min, attrs->position.cast<float>());
  attrs->semantic_feature = getOneHot(onehot_index.value_or(index), 10);
  graph.emplaceNode(DsgLayers::SEGMENTS, NodeSymbol('s', index), std::move(attrs));
}

void callWithUnmerged(const UpdateFunctor& functor, SharedDsgInfo& dsg) {
  const auto unmerged = dsg.graph->clone();
  functor.call(*unmerged, dsg, nullptr);
}

}  // namespace

TEST(IBObjectsUpdateFunctor, AddEdges) {
  SharedDsgInfo dsg(SharedDsgInfo::Config{{
      {DsgLayers::SEGMENTS, 1},
      {DsgLayers::OBJECTS, 2},
      {DsgLayers::PLACES, 3},
  }});

  EmbeddingGroup tasks;
  tasks.embeddings = {getOneHot(0, 10), getOneHot(1, 10)};
  tasks.names = {"task_0", "task_1"};

  IBObjectsUpdateFunctor::Config config;
  config.tasks = tasks;

  auto& graph = *dsg.graph;
  addSegment(graph, 0, -1.0, 1.0, 1);
  addSegment(graph, 1, 0.5, 1.5, 1);
  addSegment(graph, 2, 2.0, 3.0, 0);

  IBObjectsUpdateFunctor functor(config);
  callWithUnmerged(functor, dsg);
  EXPECT_TRUE(graph.hasEdge("s0"_id, "s1"_id));
  EXPECT_FALSE(graph.hasEdge("s0"_id, "s2"_id));
  EXPECT_FALSE(graph.hasEdge("s1"_id, "s2"_id));

  // bridge two components
  addSegment(graph, 3, 1.2, 2.5, 0);
  callWithUnmerged(functor, dsg);
  EXPECT_FALSE(graph.hasEdge("s3"_id, "s0"_id));
  EXPECT_TRUE(graph.hasEdge("s3"_id, "s1"_id));
  EXPECT_TRUE(graph.hasEdge("s3"_id, "s2"_id));
}

}  // namespace hydra
