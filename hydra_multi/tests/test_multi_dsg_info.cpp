#include <gtest/gtest.h>
#include <hydra_multi/common/multi_dsg_info.h>
#include <spark_dsg/labelspace.h>
#include <spark_dsg/scene_graph.h>

namespace hydra_multi {

using spark_dsg::DsgLayers;
using spark_dsg::Labelspace;
using spark_dsg::SceneGraph;

namespace {

SceneGraph::Ptr robotGraph(const std::vector<std::string>& object_names,
                           const std::vector<std::string>& place_names = {}) {
  // layer keys as Hydra writes them: objects {2, 0}, mesh places {3, 1}
  auto graph = SceneGraph::fromNames(
      {{DsgLayers::OBJECTS, {2, 0}}, {DsgLayers::MESH_PLACES, {3, 1}}});
  if (!object_names.empty()) {
    Labelspace(object_names).save(*graph, DsgLayers::OBJECTS);
  }
  if (!place_names.empty()) {
    Labelspace(place_names).save(*graph, DsgLayers::MESH_PLACES);
  }
  return graph;
}

}  // namespace

TEST(MultiDsgInfo, MergedGraphKeepsRobotLabelspaces) {
  MultiDsgInfo info(hydra::SharedDsgInfo::Config{});
  info.addRobotGraph(0, *robotGraph({"car", "tree"}, {"ground"}));

  const auto objects = Labelspace::fromMetadata(*info.graph, 2, 0);
  ASSERT_TRUE(objects);
  EXPECT_EQ(objects.getCategory(1).value_or(""), "tree");
  const auto places = Labelspace::fromMetadata(*info.graph, 3, 1);
  ASSERT_TRUE(places);
  EXPECT_EQ(places.getCategory(0).value_or(""), "ground");
}

TEST(MultiDsgInfo, LaterRobotsFillMissingLabelspacesOnly) {
  MultiDsgInfo info(hydra::SharedDsgInfo::Config{});
  info.addRobotGraph(0, *robotGraph({}));  // no labelspace at all
  info.addRobotGraph(1, *robotGraph({"car", "tree"}));
  EXPECT_EQ(Labelspace::fromMetadata(*info.graph, 2, 0).getCategory(0).value_or(""),
            "car");

  // a conflicting labelspace from a later robot does not replace the first one
  info.addRobotGraph(2, *robotGraph({"bike", "bench"}));
  EXPECT_EQ(Labelspace::fromMetadata(*info.graph, 2, 0).getCategory(0).value_or(""),
            "car");
}

}  // namespace hydra_multi
