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
#include <hydra/backend/merge_tracker.h>
#include <spark_dsg/node_attributes.h>
#include <spark_dsg/node_symbol.h>
#include <spark_dsg/scene_graph.h>
#include <unistd.h>

#include <fstream>
#include <nlohmann/json.hpp>

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

    const auto parent = NodeSymbol('O', 0);
    const auto child = NodeSymbol('O', 1);
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

TEST(MergeTracker, SavesMerges) {
  auto dsg = test::makeSharedDsg();
  auto& merged = *dsg->graph;
  for (size_t i = 0; i < 4; ++i) {
    addObject(merged, i);
  }

  const auto unmerged = merged.clone();

  MergeTracker tracker;
  MergeList proposals{{NodeSymbol('O', 1), NodeSymbol('O', 0)},
                      {NodeSymbol('O', 0), NodeSymbol('O', 2)}};
  ASSERT_EQ(tracker.applyMerges(*unmerged, proposals, *dsg), 2u);

  const auto merge = [](NodeId child, NodeId parent) {
    return nlohmann::json{{"child", child}, {"parent", parent}};
  };
  const nlohmann::json expected{{"version", 1},
                                {"merges",
                                 {merge(NodeSymbol('O', 0), NodeSymbol('O', 2)),
                                  merge(NodeSymbol('O', 1), NodeSymbol('O', 2))}}};
  auto pattern =
      (std::filesystem::temp_directory_path() / "hydra-merges-XXXXXX").string();
  ASSERT_NE(mkdtemp(pattern.data()), nullptr);
  const auto path = std::filesystem::path(pattern) / "merges.json";
  const auto read_saved = [&path](const std::map<NodeId, NodeId>& merges) {
    saveMerges(merges, path);
    std::ifstream input(path);
    return nlohmann::json::parse(input);
  };
  EXPECT_EQ(read_saved(dsg->merges), expected);

  // Chains are resolved even if the map is not flattened.
  const std::map<NodeId, NodeId> chain{{NodeSymbol('O', 1), NodeSymbol('O', 0)},
                                       {NodeSymbol('O', 0), NodeSymbol('O', 2)}};
  EXPECT_EQ(read_saved(chain), expected);
  EXPECT_EQ(read_saved({}),
            (nlohmann::json{{"version", 1}, {"merges", nlohmann::json::array()}}));
  std::filesystem::remove_all(pattern);
}

}  // namespace hydra
