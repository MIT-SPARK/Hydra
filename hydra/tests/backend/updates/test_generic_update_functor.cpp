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
#include <config_utilities/printing.h>
#include <glog/logging.h>
#include <glog/stl_logging.h>
#include <gtest/gtest.h>
#include <hydra/backend/merge_tracker.h>
#include <hydra/backend/updates/generic_update_functor.h>
#include <kimera_pgmo/deformation_graph.h>
#include <spark_dsg/node_attributes.h>
#include <spark_dsg/node_symbol.h>

#include <filesystem>
#include <fstream>
#include <string>

#include "hydra_test/resources.h"
#include "hydra_test/shared_dsg_fixture.h"
#include "hydra_test/temp_directory.h"

using namespace spark_dsg;

namespace hydra {
namespace {

MergeList callWithUnmerged(const UpdateFunctor& functor,
                           SharedDsgInfo& dsg,
                           const UpdateInfo::ConstPtr& info,
                           bool enable_merging) {
  const auto unmerged = dsg.graph->clone();
  functor.call(*unmerged, dsg, info);
  const auto hooks = functor.hooks();
  if (enable_merging && hooks.find_merges) {
    return hooks.find_merges(*unmerged, info);
  } else {
    return {};
  }
}

GenericUpdateFunctor::Config defaultConfig() { return {5, "OBJECTS"}; }

namespace fs = std::filesystem;

void writeFile(const fs::path& path, const std::string& contents = "{}") {
  fs::create_directories(path.parent_path());
  std::ofstream(path) << contents << "\n";
}

std::string readFile(const fs::path& path) {
  std::string contents;
  std::getline(std::ifstream(path), contents);
  return contents;
}

std::unique_ptr<KhronosObjectAttributes> makeObject(const std::string& image_folder,
                                                    bool is_active = false) {
  auto attrs = std::make_unique<KhronosObjectAttributes>();
  attrs->position.setZero();
  attrs->is_active = is_active;
  attrs->last_update_time_ns = 10u;
  attrs->image_folder = image_folder;
  return attrs;
}

const std::string& imageFolder(const SceneGraph& graph, NodeId node) {
  return graph.getNode(node).attributes<KhronosObjectAttributes>().image_folder;
}

GenericUpdateFunctor::Config imageConfig(const fs::path& root) {
  auto config = defaultConfig();
  config.image_root = root;
  return config;
}

}  // namespace

TEST(GenericUpdateFunctor, noUpdate) {
  auto dsg = test::makeSharedDsg();
  auto& graph = *dsg->graph;

  const Eigen::Vector3d expected(1.0, 2.0, 3.0);
  {  // scope limiting moved attrs access
    auto attrs = std::make_unique<NodeAttributes>();
    attrs->position = expected;
    attrs->is_active = true;
    attrs->last_update_time_ns = 10u;
    graph.emplaceNode(DsgLayers::OBJECTS, 0, std::move(attrs));
  }

  UpdateInfo::ConstPtr info(new UpdateInfo{0, nullptr, nullptr, false, {}});
  auto config = defaultConfig();
  config.enable_merging = false;
  GenericUpdateFunctor functor(config);
  callWithUnmerged(functor, *dsg, info, false);

  // No deformation, so nothing should change
  const auto& result = graph.getNode(0).attributes();
  EXPECT_NEAR(0.0, (expected - result.position).norm(), 1.0e-7);
}

TEST(GenericUpdateFunctor, shouldUpdate) {
  auto dsg = test::makeSharedDsg();
  auto& graph = *dsg->graph;

  {  // scope limiting moved attrs access
    auto attrs = std::make_unique<NodeAttributes>();
    attrs->position << 0, 3, 1;
    attrs->is_active = true;
    attrs->last_update_time_ns = 10u;
    graph.emplaceNode(DsgLayers::OBJECTS, 0, std::move(attrs));
  }

  kimera_pgmo::DeformationGraph dgraph;
  dgraph.load(test::get_resource_path() / "graph.dgrf");

  UpdateInfo::ConstPtr info(new UpdateInfo{0, nullptr, nullptr, false, {}, &dgraph});
  auto config = defaultConfig();
  config.enable_merging = false;
  VLOG(1) << "Using config:\n" << config::toString(config);

  GenericUpdateFunctor functor(config);
  callWithUnmerged(functor, *dsg, info, false);

  const auto& result = graph.getNode(0).attributes();
  const Eigen::Vector3d expected(1.0, 2.0, 3.0);
  EXPECT_NEAR(0.0, (expected - result.position).norm(), 1.0e-7);

  // second call shouldn't change position
  graph.getNode(0).attributes().is_active = false;
  callWithUnmerged(functor, *dsg, info, false);
  EXPECT_NEAR(0.0, (expected - result.position).norm(), 1.0e-7);
}

// Without an image root the functor never touches the filesystem
TEST(GenericUpdateFunctor, imageFoldersDisabledWithoutRoot) {
  test::TempDirectory tmp;
  const auto root = tmp.path / "images";
  const auto temp_folder = root / "temp" / "O_7";
  writeFile(temp_folder / "crop.png");

  auto dsg = test::makeSharedDsg();
  dsg->graph->emplaceNode(
      DsgLayers::OBJECTS, NodeSymbol('O', 0), makeObject(temp_folder.string()));
  const auto unmerged = dsg->graph->clone();

  GenericUpdateFunctor functor(defaultConfig());
  UpdateInfo::ConstPtr info(new UpdateInfo{0, nullptr, nullptr, false, {}});
  functor.call(*unmerged, *dsg, info);

  EXPECT_TRUE(fs::exists(temp_folder / "crop.png"));
  EXPECT_FALSE(fs::exists(root / "O_0"));
  EXPECT_EQ(imageFolder(*dsg->graph, NodeSymbol('O', 0)), temp_folder.string());
}

// The frontend's temporary folder lives on the unmerged graph (the merged copy of an
// archived node is never refreshed by mergeGraph), so the rename reads the unmerged
// graph and mirrors the final folder onto the merged graph.
TEST(GenericUpdateFunctor, imageFoldersRenameReadsUnmergedAndWritesMerged) {
  test::TempDirectory tmp;
  const auto root = tmp.path / "images";
  const auto temp_folder = root / "temp" / "O_track7";
  writeFile(temp_folder / "crop_a.png");

  auto dsg = test::makeSharedDsg();
  auto& merged = *dsg->graph;
  merged.emplaceNode(DsgLayers::OBJECTS, NodeSymbol('O', 0), makeObject(""));
  const auto unmerged = merged.clone();
  // stored folders are relative to the parent of the image root
  unmerged->setNodeAttributes(NodeSymbol('O', 0), makeObject("images/temp/O_track7"));

  GenericUpdateFunctor functor(imageConfig(root));
  UpdateInfo::ConstPtr info(new UpdateInfo{0, nullptr, nullptr, false, {}});
  functor.call(*unmerged, *dsg, info);

  EXPECT_TRUE(fs::exists(root / "O_0" / "crop_a.png"));
  EXPECT_FALSE(fs::exists(temp_folder));
  EXPECT_EQ(imageFolder(merged, NodeSymbol('O', 0)), "images/O_0");

  // repeated calls are stable
  functor.call(*unmerged, *dsg, info);
  EXPECT_TRUE(fs::exists(root / "O_0" / "crop_a.png"));
  EXPECT_EQ(imageFolder(merged, NodeSymbol('O', 0)), "images/O_0");
}

// Only folders under <image_root>/temp are moved
TEST(GenericUpdateFunctor, imageFoldersOnlyMoveTemporaryFolders) {
  test::TempDirectory tmp;
  const auto root = tmp.path / "images";
  const auto lookalike = root / "temporary" / "O_1";
  const auto outside = tmp.path / "other" / "temp" / "O_2";
  const auto sibling = tmp.path / "images2" / "temp" / "O_3";
  writeFile(lookalike / "crop.png");
  writeFile(outside / "crop.png");
  writeFile(sibling / "crop.png");

  auto dsg = test::makeSharedDsg();
  auto& merged = *dsg->graph;
  merged.emplaceNode(
      DsgLayers::OBJECTS, NodeSymbol('O', 1), makeObject(lookalike.string()));
  merged.emplaceNode(
      DsgLayers::OBJECTS, NodeSymbol('O', 2), makeObject(outside.string()));
  // relative folders resolve against the parent of the image root
  merged.emplaceNode(
      DsgLayers::OBJECTS, NodeSymbol('O', 3), makeObject("images2/temp/O_3"));
  const auto unmerged = merged.clone();

  GenericUpdateFunctor functor(imageConfig(root));
  UpdateInfo::ConstPtr info(new UpdateInfo{0, nullptr, nullptr, false, {}});
  functor.call(*unmerged, *dsg, info);

  EXPECT_TRUE(fs::exists(lookalike / "crop.png"));
  EXPECT_TRUE(fs::exists(outside / "crop.png"));
  EXPECT_TRUE(fs::exists(sibling / "crop.png"));
  EXPECT_FALSE(fs::exists(root / "O_1"));
  EXPECT_FALSE(fs::exists(root / "O_2"));
  EXPECT_FALSE(fs::exists(root / "O_3"));
  EXPECT_EQ(imageFolder(merged, NodeSymbol('O', 1)), lookalike.string());
  EXPECT_EQ(imageFolder(merged, NodeSymbol('O', 3)), "images2/temp/O_3");
}

// Merging unions the image folders into the surviving node without touching the
// (deformed) geometry of the merged attributes. Each temporary folder is moved once
// (the frontend writes it once); later calls do not touch it again. Absolute input
// folders are accepted; the folders written by the backend are relative.
TEST(GenericUpdateFunctor, imageFoldersUnionOnMerge) {
  test::TempDirectory tmp;
  const auto root = tmp.path / "images";
  writeFile(root / "temp" / "a" / "crop_a.png");
  writeFile(root / "temp" / "b" / "crop_b.png");

  auto dsg = test::makeSharedDsg();
  auto& merged = *dsg->graph;
  merged.emplaceNode(DsgLayers::OBJECTS,
                     NodeSymbol('O', 0),
                     makeObject((root / "temp" / "a").string()));
  merged.emplaceNode(DsgLayers::OBJECTS,
                     NodeSymbol('O', 1),
                     makeObject((root / "temp" / "b").string()));
  const auto unmerged = merged.clone();
  // optimized position of the surviving node differs from the odometric one
  const Eigen::Vector3d optimized(1.0, 2.0, 3.0);
  merged.getNode(NodeSymbol('O', 0)).attributes().position = optimized;

  GenericUpdateFunctor functor(imageConfig(root));
  UpdateInfo::ConstPtr info(new UpdateInfo{0, nullptr, nullptr, false, {}});
  functor.call(*unmerged, *dsg, info);
  ASSERT_TRUE(fs::exists(root / "O_0" / "crop_a.png"));
  ASSERT_TRUE(fs::exists(root / "O_1" / "crop_b.png"));

  const auto hooks = functor.hooks();
  MergeTracker tracker;
  MergeList proposals{{NodeSymbol('O', 1), NodeSymbol('O', 0)}};
  ASSERT_EQ(tracker.applyMerges(*unmerged, proposals, *dsg, hooks.merge), 1u);
  functor.call(*unmerged, *dsg, info);

  EXPECT_TRUE(fs::exists(root / "O_0" / "crop_a.png"));
  EXPECT_TRUE(fs::exists(root / "O_0" / "crop_b.png"));
  EXPECT_FALSE(fs::exists(root / "O_1"));
  EXPECT_EQ(imageFolder(merged, NodeSymbol('O', 0)), "images/O_0");
  const auto& result = merged.getNode(NodeSymbol('O', 0)).attributes();
  EXPECT_NEAR(0.0, (optimized - result.position).norm(), 1.0e-9);

  // the unmerged graph still points to the moved temporary folders, which are not
  // revisited
  writeFile(root / "temp" / "b" / "crop_c.png");
  functor.call(*unmerged, *dsg, info);
  EXPECT_TRUE(fs::exists(root / "temp" / "b" / "crop_c.png"));
  EXPECT_FALSE(fs::exists(root / "O_0" / "crop_c.png"));
  EXPECT_FALSE(fs::exists(root / "O_1"));
  EXPECT_EQ(imageFolder(merged, NodeSymbol('O', 0)), "images/O_0");
}

// Files with the same name in two merged folders are not overwritten; the surviving
// node keeps its own file and the merged node's file stays in its folder
TEST(GenericUpdateFunctor, imageFoldersUnionKeepsExistingFiles) {
  test::TempDirectory tmp;
  const auto root = tmp.path / "images";
  writeFile(root / "temp" / "a" / "frame_10_0.png", "a");
  writeFile(root / "temp" / "b" / "frame_10_0.png", "b");
  writeFile(root / "temp" / "b" / "frame_20_0.png", "b");

  auto dsg = test::makeSharedDsg();
  auto& merged = *dsg->graph;
  merged.emplaceNode(
      DsgLayers::OBJECTS, NodeSymbol('O', 0), makeObject("images/temp/a"));
  merged.emplaceNode(
      DsgLayers::OBJECTS, NodeSymbol('O', 1), makeObject("images/temp/b"));
  const auto unmerged = merged.clone();

  GenericUpdateFunctor functor(imageConfig(root));
  UpdateInfo::ConstPtr info(new UpdateInfo{0, nullptr, nullptr, false, {}});
  functor.call(*unmerged, *dsg, info);

  MergeTracker tracker;
  MergeList proposals{{NodeSymbol('O', 1), NodeSymbol('O', 0)}};
  ASSERT_EQ(tracker.applyMerges(*unmerged, proposals, *dsg, nullptr), 1u);
  functor.call(*unmerged, *dsg, info);

  EXPECT_EQ(readFile(root / "O_0" / "frame_10_0.png"), "a");
  EXPECT_EQ(readFile(root / "O_0" / "frame_20_0.png"), "b");
  EXPECT_EQ(readFile(root / "O_1" / "frame_10_0.png"), "b");
  EXPECT_FALSE(fs::exists(root / "O_1" / "frame_20_0.png"));
  EXPECT_EQ(imageFolder(merged, NodeSymbol('O', 0)), "images/O_0");
}

// A surviving node without crops of its own adopts the union of its children
TEST(GenericUpdateFunctor, imageFoldersUnionChildOnly) {
  test::TempDirectory tmp;
  const auto root = tmp.path / "images";
  writeFile(root / "temp" / "b" / "crop_b.png");

  auto dsg = test::makeSharedDsg();
  auto& merged = *dsg->graph;
  merged.emplaceNode(DsgLayers::OBJECTS, NodeSymbol('O', 0), makeObject(""));
  merged.emplaceNode(DsgLayers::OBJECTS,
                     NodeSymbol('O', 1),
                     makeObject((root / "temp" / "b").string()));
  merged.emplaceNode(DsgLayers::OBJECTS, NodeSymbol('O', 2), makeObject(""));
  const auto unmerged = merged.clone();

  GenericUpdateFunctor functor(imageConfig(root));
  UpdateInfo::ConstPtr info(new UpdateInfo{0, nullptr, nullptr, false, {}});
  functor.call(*unmerged, *dsg, info);

  MergeTracker tracker;
  MergeList proposals{{NodeSymbol('O', 1), NodeSymbol('O', 0)},
                      {NodeSymbol('O', 2), NodeSymbol('O', 0)}};
  ASSERT_EQ(tracker.applyMerges(*unmerged, proposals, *dsg, nullptr), 2u);
  functor.call(*unmerged, *dsg, info);

  EXPECT_TRUE(fs::exists(root / "O_0" / "crop_b.png"));
  EXPECT_FALSE(fs::exists(root / "O_1"));
  EXPECT_FALSE(fs::exists(root / "O_2"));
  EXPECT_EQ(imageFolder(merged, NodeSymbol('O', 0)), "images/O_0");
}

}  // namespace hydra
