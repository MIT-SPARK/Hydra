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
#include <hydra/common/graph_update.h>
#include <spark_dsg/node_attributes.h>
#include <spark_dsg/node_symbol.h>

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <nlohmann/json.hpp>
#include <sstream>

#include "hydra_test/shared_dsg_fixture.h"

using namespace spark_dsg;

namespace hydra {
namespace {

GraphUpdater::Config makeConfig(bool with_matcher) {
  LayerTracker::Config tracker;
  tracker.prefix = 'O';
  if (with_matcher) {
    tracker.matcher =
        config::VirtualConfig<NodeMatcher>(DistanceNodeMatcher::Config{0.5});
  }

  GraphUpdater::Config config;
  config.layer_updates[DsgLayers::OBJECTS] = tracker;
  return config;
}

NodeUpdate addOrUpdate(size_t track_id, double x) {
  auto attrs = std::make_shared<ObjectNodeAttributes>();
  attrs->position << x, 0.0, 0.0;
  return NodeUpdate{attrs, track_id};
}

NodeUpdate remove(size_t track_id) {
  return NodeUpdate{nullptr, track_id, NodeUpdate::UpdateType::Delete};
}

GraphUpdate makeUpdate(const std::list<NodeUpdate>& updates) {
  auto layer = std::make_shared<LayerUpdate>(2);
  for (const auto& update : updates) {
    layer->updates.push_back(update);
  }
  return {{2, layer}};
}

void expectBinding(const TrackBinding& binding,
                   size_t track_id,
                   NodeId node_id,
                   uint64_t first_ns,
                   uint64_t last_ns,
                   const std::optional<std::string>& end) {
  EXPECT_EQ(binding.track_id, track_id);
  EXPECT_EQ(NodeSymbol(binding.node_id).str(), NodeSymbol(node_id).str());
  EXPECT_EQ(binding.first_ns, first_ns);
  EXPECT_EQ(binding.last_ns, last_ns);
  EXPECT_EQ(binding.end, end);
}

}  // namespace

TEST(GraphUpdater, BindingHistory) {
  auto dsg = test::makeSharedDsg();
  auto& graph = *dsg->graph;
  GraphUpdater updater(makeConfig(false));

  updater.update(makeUpdate({addOrUpdate(1, 0.0), addOrUpdate(2, 5.0)}), graph, 10);
  updater.update(makeUpdate({addOrUpdate(1, 0.1), remove(2)}), graph, 20);
  updater.update(makeUpdate({addOrUpdate(2, 6.0), remove(1)}), graph, 30);
  updater.update(makeUpdate({addOrUpdate(2, 6.1)}), graph, 40);

  EXPECT_FALSE(graph.hasNode(NodeSymbol('O', 0)));
  EXPECT_FALSE(graph.hasNode(NodeSymbol('O', 1)));
  EXPECT_TRUE(graph.hasNode(NodeSymbol('O', 2)));

  const auto& bindings = updater.trackers().at(DsgLayers::OBJECTS).bindings;
  ASSERT_EQ(bindings.size(), 3u);
  expectBinding(bindings[0], 1, NodeSymbol('O', 0), 10, 20, "deleted");
  expectBinding(bindings[1], 2, NodeSymbol('O', 1), 10, 10, "deleted");
  expectBinding(bindings[2], 2, NodeSymbol('O', 2), 30, 40, std::nullopt);

  const auto binding = [](int track,
                          NodeId node,
                          uint64_t first,
                          uint64_t last,
                          const nlohmann::json& end) {
    return nlohmann::json{{"track_id", track},
                          {"node_id", node},
                          {"first_ns", first},
                          {"last_ns", last},
                          {"active", end.is_null()},
                          {"end", end}};
  };
  const nlohmann::json expected{
      {"version", 1},
      {"layers",
       {{"OBJECTS",
         {binding(1, NodeSymbol('O', 0), 10, 20, "deleted"),
          binding(2, NodeSymbol('O', 1), 10, 10, "deleted"),
          binding(2, NodeSymbol('O', 2), 30, 40, nullptr)}}}}};
  const auto text = updater.bindingsToJson();
  EXPECT_EQ(nlohmann::json::parse(text), expected);
  EXPECT_EQ(text.rfind("{\n  \"version\": 1,", 0), 0u);  // version first
  EXPECT_EQ(text.back(), '\n');
}

TEST(GraphUpdater, BindingHistoryMatchedTracks) {
  auto dsg = test::makeSharedDsg();
  auto& graph = *dsg->graph;
  GraphUpdater updater(makeConfig(true));

  // track 2 is close enough to be matched to the node of track 1
  updater.update(makeUpdate({addOrUpdate(1, 0.0)}), graph, 10);
  updater.update(makeUpdate({addOrUpdate(2, 0.2)}), graph, 20);
  updater.update(makeUpdate({remove(1)}), graph, 30);
  updater.update(makeUpdate({addOrUpdate(2, 0.3)}), graph, 40);

  EXPECT_TRUE(graph.hasNode(NodeSymbol('O', 0)));
  EXPECT_FALSE(graph.hasNode(NodeSymbol('O', 1)));
  const auto& bindings = updater.trackers().at(DsgLayers::OBJECTS).bindings;
  ASSERT_EQ(bindings.size(), 2u);
  expectBinding(bindings[0], 1, NodeSymbol('O', 0), 10, 10, "track_removed");
  expectBinding(bindings[1], 2, NodeSymbol('O', 0), 20, 40, std::nullopt);
}

TEST(GraphUpdater, SaveBindings) {
  auto dsg = test::makeSharedDsg();
  GraphUpdater updater(makeConfig(false));
  updater.update(makeUpdate({addOrUpdate(1, 0.0)}), *dsg->graph, 10);

  auto dir =
      (std::filesystem::temp_directory_path() / "hydra-bindings-XXXXXX").string();
  ASSERT_NE(mkdtemp(dir.data()), nullptr);
  const auto path = std::filesystem::path(dir) / "track_bindings.json";
  updater.saveBindings(path);
  std::ifstream fin(path);
  std::stringstream contents;
  contents << fin.rdbuf();
  EXPECT_EQ(contents.str(), updater.bindingsToJson());
  EXPECT_FALSE(std::filesystem::exists(path.string() + ".tmp"));
  std::filesystem::remove_all(dir);
}

}  // namespace hydra
