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
#include <hydra/backend/backend_utilities.h>
#include <spark_dsg/node_attributes.h>
#include <spark_dsg/node_symbol.h>

#include <chrono>
#include <filesystem>
#include <fstream>

#include "hydra_test/shared_dsg_fixture.h"
#include "hydra_test/temp_directory.h"

namespace hydra {

using spark_dsg::AgentNodeAttributes;
using spark_dsg::DsgLayers;
using spark_dsg::KhronosObjectAttributes;
using spark_dsg::NodeSymbol;
using spark_dsg::SceneGraph;

namespace {

namespace fs = std::filesystem;

void writeFile(const fs::path& path) {
  fs::create_directories(path.parent_path());
  std::ofstream(path) << "{}\n";
}

void addAgent(SceneGraph& graph,
              int64_t stamp_ns,
              size_t index,
              const std::string& image_folder = "") {
  auto attrs = std::make_unique<AgentNodeAttributes>(std::chrono::nanoseconds(stamp_ns),
                                                     Eigen::Quaterniond::Identity(),
                                                     Eigen::Vector3d::Zero(),
                                                     NodeSymbol('a', index));
  attrs->image_folder = image_folder;
  graph.emplaceNode(graph.getLayerKey(DsgLayers::AGENTS)->layer,
                    NodeSymbol('a', index),
                    std::move(attrs),
                    'a');
}

const std::string& agentFolder(const SceneGraph& graph, size_t index) {
  return graph.getNode(NodeSymbol('a', index))
      .attributes<AgentNodeAttributes>()
      .image_folder;
}

const std::string& objectFolder(const SceneGraph& graph, size_t index) {
  return graph.getNode(NodeSymbol('O', index))
      .attributes<KhronosObjectAttributes>()
      .image_folder;
}

}  // namespace

TEST(BackendUtilities, PathUnderDirectory) {
  EXPECT_TRUE(utils::isPathUnder("/a/b/temp/O_1", "/a/b/temp"));
  EXPECT_TRUE(utils::isPathUnder("/a/b/temp/O_1", "/a/b/temp/"));
  EXPECT_TRUE(utils::isPathUnder("/a/b/./temp/x/../O_1", "/a/b/temp"));
  EXPECT_FALSE(utils::isPathUnder("/a/b/temp", "/a/b/temp"));
  EXPECT_FALSE(utils::isPathUnder("/a/b/temporary/O_1", "/a/b/temp"));
  EXPECT_FALSE(utils::isPathUnder("/c/temp/O_1", "/a/b/temp"));
}

// Empty agent image folders are reconstructed from the node timestamp, but only if
// the keyframe metadata exists on disk
TEST(BackendUtilities, ReconcileAgentsFillsEmptyFromTimestamp) {
  test::TempDirectory tmp;
  const auto root = tmp.path / "agents";
  writeFile(root / "agent_1000_meta.json");
  writeFile(root / "agent_2000_meta.json");

  auto dsg = test::makeSharedDsg();
  auto& graph = *dsg->graph;
  addAgent(graph, 1000, 0, "custom");
  addAgent(graph, 2000, 1);
  addAgent(graph, 3000, 2);

  EXPECT_EQ(utils::reconcileAgentImageFolders(graph, root), 1u);
  EXPECT_EQ(agentFolder(graph, 0), "custom");
  // restored folders are relative to the parent of the image root
  EXPECT_EQ(agentFolder(graph, 1), "agents/agent_2000");
  EXPECT_TRUE(agentFolder(graph, 2).empty());
}

// An empty directory disables reconciliation
TEST(BackendUtilities, ReconcileAgentsDisabledWithoutDirectory) {
  test::TempDirectory tmp;
  writeFile(tmp.path / "agent_1000_meta.json");

  auto dsg = test::makeSharedDsg();
  auto& graph = *dsg->graph;
  addAgent(graph, 1000, 0);

  EXPECT_EQ(utils::reconcileAgentImageFolders(graph, ""), 0u);
  EXPECT_TRUE(agentFolder(graph, 0).empty());
}

// Empty object image folders are filled with the final per-node folder if it exists
TEST(BackendUtilities, ReconcileObjectsFillsEmptyFromSymbol) {
  test::TempDirectory tmp;
  const auto root = tmp.path / "images";
  fs::create_directories(root / "O_5");

  auto dsg = test::makeSharedDsg();
  auto& graph = *dsg->graph;
  {
    auto attrs = std::make_unique<KhronosObjectAttributes>();
    attrs->image_folder = "custom";
    graph.emplaceNode(DsgLayers::OBJECTS, NodeSymbol('O', 0), std::move(attrs));
  }
  graph.emplaceNode(DsgLayers::OBJECTS,
                    NodeSymbol('O', 5),
                    std::make_unique<KhronosObjectAttributes>());
  graph.emplaceNode(DsgLayers::OBJECTS,
                    NodeSymbol('O', 7),
                    std::make_unique<KhronosObjectAttributes>());

  EXPECT_EQ(utils::reconcileObjectImageFolders(graph, ""), 0u);
  EXPECT_EQ(utils::reconcileObjectImageFolders(graph, root), 1u);
  EXPECT_EQ(objectFolder(graph, 0), "custom");
  // restored folders are relative to the parent of the image root
  EXPECT_EQ(objectFolder(graph, 5), "images/O_5");
  EXPECT_TRUE(objectFolder(graph, 7).empty());
}

// Merges can be recomputed (e.g., after a loop closure), so a node that was merged
// once can later absorb other nodes and then be merged into a different node. Its
// folder must follow it each time.
TEST(BackendUtilities, ObjectImageFoldersFollowRecomputedMerges) {
  test::TempDirectory tmp;
  const auto root = tmp.path / "images";
  writeFile(root / "O_1" / "frame_1_meta.json");
  writeFile(root / "O_3" / "frame_3_meta.json");

  auto dsg = test::makeSharedDsg();
  auto& merged = *dsg->graph;
  const auto layer = merged.getLayerKey(DsgLayers::OBJECTS)->layer;
  for (const size_t index : {0, 1, 2, 3}) {
    merged.emplaceNode(
        layer, NodeSymbol('O', index), std::make_unique<KhronosObjectAttributes>());
  }
  const auto unmerged = merged.clone();

  const utils::ObjectImageFolders folders(root);
  folders.update(*unmerged,
                 DsgLayers::OBJECTS,
                 {{NodeSymbol('O', 1), NodeSymbol('O', 0)}},
                 merged);
  EXPECT_TRUE(fs::exists(root / "O_0" / "frame_1_meta.json"));

  // the merge is undone and O_1 absorbs O_3 instead
  folders.update(*unmerged,
                 DsgLayers::OBJECTS,
                 {{NodeSymbol('O', 3), NodeSymbol('O', 1)}},
                 merged);
  EXPECT_TRUE(fs::exists(root / "O_1" / "frame_3_meta.json"));

  // O_1 (with O_3) is then merged into O_2
  folders.update(*unmerged,
                 DsgLayers::OBJECTS,
                 {{NodeSymbol('O', 1), NodeSymbol('O', 2)},
                  {NodeSymbol('O', 3), NodeSymbol('O', 2)}},
                 merged);
  EXPECT_TRUE(fs::exists(root / "O_2" / "frame_3_meta.json"));
  EXPECT_FALSE(fs::exists(root / "O_1"));
  EXPECT_EQ(objectFolder(merged, 2), "images/O_2");
}

}  // namespace hydra
