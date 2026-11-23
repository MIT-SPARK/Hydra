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
#include <spark_dsg/node_attributes.h>
#include <spark_dsg/node_symbol.h>
#include <spark_dsg/scene_graph.h>

#include <nlohmann/json.hpp>

#include "hydra/backend/dsg_updater.h"
#include "hydra/backend/graph_reconciler.h"
#include "hydra/backend/merge_tracker.h"
#include "hydra/utils/temp_path.h"
#include "hydra_test/shared_dsg_fixture.h"

using namespace spark_dsg;

namespace hydra {
namespace {

void addObject(SceneGraph& graph, size_t index) {
  auto attrs = std::make_unique<ObjectNodeAttributes>();
  attrs->position << static_cast<double>(index), 0.0, 0.0;
  graph.emplaceNode(DsgLayers::OBJECTS, NodeSymbol('O', index), std::move(attrs));
}

}  // namespace

TEST(MergeTracker, DropsMergeSetsWithMissingParents) {
  for (const auto& [remove_unmerged, remove_merged] :
       {std::pair{true, true}, std::pair{true, false}, std::pair{false, true}}) {
    SCOPED_TRACE(::testing::Message() << remove_unmerged << ", " << remove_merged);
    auto dsg = test::makeSharedDsg();
    auto& merged = *dsg->graph;
    addObject(merged, 0);
    addObject(merged, 1);
    const auto unmerged = merged.clone();

    const auto parent = "O0"_id;
    const auto child = "O1"_id;
    const auto merge_attrs =
        [](const SceneGraph& graph,
           const std::vector<NodeId>& nodes) -> NodeAttributes::Ptr {
      return graph.getNode(nodes.front()).attributes().clone();
    };
    MergeTracker tracker;
    const MergeList proposals{{child, parent}};
    ASSERT_EQ(tracker.applyMerges(*unmerged, proposals, *dsg, merge_attrs), 1u);
    const auto saved_merges = dsg->merges;
    if (remove_unmerged) {
      unmerged->removeNode(parent);
    }

    if (remove_merged) {
      merged.removeNode(parent);
    }

    EXPECT_NO_THROW(tracker.updateAllMergeAttributes(*unmerged, merged, merge_attrs));
    EXPECT_NO_THROW(tracker.applyMerges(*unmerged, proposals, *dsg, merge_attrs));
    EXPECT_EQ(unmerged->hasNode(parent), !remove_unmerged);
    EXPECT_EQ(merged.hasNode(parent), !remove_merged);
    EXPECT_FALSE(merged.hasNode(child));
    EXPECT_EQ(dsg->merges, saved_merges);
  }
}

TEST(MergeTracker, NullAttributesCorrect) {
  auto dsg = test::makeSharedDsg();
  auto& merged = *dsg->graph;
  addObject(merged, 0);
  addObject(merged, 1);
  const auto unmerged = merged.clone();

  MergeTracker tracker;
  const auto null_attrs = [](const SceneGraph&,
                             const std::vector<NodeId>&) -> NodeAttributes::Ptr {
    return nullptr;
  };

  MergeList proposals{{NodeSymbol('O', 1), NodeSymbol('O', 0)}};
  EXPECT_EQ(tracker.applyMerges(*unmerged, proposals, *dsg, null_attrs), 1u);
  EXPECT_TRUE(merged.getNode(NodeSymbol('O', 0)).tryAttributes());
}

TEST(MergeTracker, BackendGraphStoresFinalParentsInMetadata) {
  const auto tempdir = utils::TempDirectory::system_default("hydra-merges-", true);
  auto dsg = test::makeSharedDsg();
  auto& merged = *dsg->graph;
  merged.metadata.add(nlohmann::json{{"source", "test"}});
  for (size_t i = 0; i < 4; ++i) {
    addObject(merged, i);
  }

  const auto unmerged = merged.clone();
  MergeTracker first, second;
  ASSERT_EQ(first.applyMerges(*unmerged, {{"O1"_id, "O0"_id}}, *dsg), 1u);
  ASSERT_EQ(second.applyMerges(*unmerged, {{"O0"_id, "O2"_id}}, *dsg), 1u);

  DsgUpdater updater(DsgUpdater::Config{}, unmerged, dsg);
  const DataDirectory output(tempdir.path);
  updater.save(output, "backend");
  const auto path = tempdir / "backend/dsg.json";
  auto saved = SceneGraph::load(path);
  const NodeId parent = "O2"_id;
  const nlohmann::json expected{{std::to_string("O0"_id), parent},
                                {std::to_string("O1"_id), parent}};
  EXPECT_EQ(saved->metadata.get().at("node_merges"), expected);
  EXPECT_EQ(saved->metadata.get().at("source"), "test");
  EXPECT_FALSE(std::filesystem::exists(tempdir / "backend/merges.json"));

  // Saving again replaces the map, including after all merges are cleared.
  dsg->merges.clear();
  updater.save(output, "backend");
  saved = SceneGraph::load(path);
  EXPECT_EQ(saved->metadata.get().at("node_merges"), nlohmann::json::object());
  EXPECT_EQ(saved->metadata.get().at("source"), "test");
}

TEST(Reconciler, PreservesMergesFromInputGraphMetadata) {
  auto dsg = test::makeSharedDsg();
  addObject(*dsg->graph, 2);
  const NodeId original = "O0"_id;
  const NodeId parent = "O2"_id;
  const nlohmann::json mapping{{std::to_string(original), parent}};
  dsg->graph->metadata.add(nlohmann::json{{"node_merges", mapping}});

  Reconciler reconciler(Reconciler::Config{});
  const auto result = reconciler.reconcile(*dsg->graph);
  ASSERT_TRUE(result);
  EXPECT_EQ(result->metadata.get().at("node_merges"), mapping);
}

}  // namespace hydra
