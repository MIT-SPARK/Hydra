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
#include <hydra/common/graph_update.h>
#include <hydra/utils/image_store.h>
#include <spark_dsg/node_attributes.h>

#include "hydra_test/image_store_query.h"
#include "hydra_test/shared_dsg_fixture.h"
#include "hydra_test/temp_directory.h"

namespace hydra {

namespace {

GraphUpdater::Config makeConfig() {
  GraphUpdater::Config config;
  config.layer_updates["OBJECTS"].prefix = 'O';
  return config;
}

LayerUpdate::Ptr makeUpdate(spark_dsg::LayerId layer,
                            const std::vector<size_t>& track_ids) {
  auto update = std::make_shared<LayerUpdate>(layer);
  for (const auto track : track_ids) {
    auto attrs = std::make_shared<spark_dsg::KhronosObjectAttributes>();
    attrs->position.setZero();
    update->updates.push_back(NodeUpdate{attrs, track});
  }
  return update;
}

}  // namespace

// Tracks are bound to the node they became in the image store
TEST(GraphUpdater, BindsTracksInImageStore) {
  test::TempDirectory tmp;
  auto dsg = test::makeSharedDsg();
  auto& graph = *dsg->graph;
  const auto layer_id = graph.getLayerKey("OBJECTS")->layer;

  GraphUpdater updater(makeConfig());
  auto store = ImageStore::open(tmp.path);
  ASSERT_TRUE(store);
  updater.setImageStore(store, 2);

  for (size_t i = 0; i < 3; ++i) {
    GraphUpdate round;
    round[layer_id] = makeUpdate(layer_id, {7, 8});
    updater.update(round, graph);
  }

  store->flush();
  const auto rows = test::queryStore(
      tmp.path, "SELECT robot_id, track_id, node_id FROM track_bindings ORDER BY 2");
  ASSERT_EQ(rows.size(), 2u);
  EXPECT_EQ(rows[0][0], "2");
  EXPECT_EQ(rows[0][1], "7");
  EXPECT_EQ(
      rows[0][2],
      std::to_string(static_cast<spark_dsg::NodeId>(spark_dsg::NodeSymbol('O', 0))));
  EXPECT_EQ(rows[1][1], "8");
  EXPECT_EQ(
      rows[1][2],
      std::to_string(static_cast<spark_dsg::NodeId>(spark_dsg::NodeSymbol('O', 1))));
}

// A track whose node is deleted (extraction dropped it) loses its binding
TEST(GraphUpdater, UnbindsDeletedTracksInImageStore) {
  test::TempDirectory tmp;
  auto dsg = test::makeSharedDsg();
  auto& graph = *dsg->graph;
  const auto layer_id = graph.getLayerKey("OBJECTS")->layer;

  GraphUpdater updater(makeConfig());
  auto store = ImageStore::open(tmp.path);
  ASSERT_TRUE(store);
  updater.setImageStore(store, 2);

  GraphUpdate round1;
  round1[layer_id] = makeUpdate(layer_id, {7, 8});
  updater.update(round1, graph);

  GraphUpdate round2;
  auto deletion = std::make_shared<LayerUpdate>(layer_id);
  deletion->updates.push_back(NodeUpdate{nullptr, 8, NodeUpdate::UpdateType::Delete});
  round2[layer_id] = deletion;
  updater.update(round2, graph);
  EXPECT_FALSE(graph.hasNode(spark_dsg::NodeSymbol('O', 1)));

  store->flush();
  const auto rows =
      test::queryStore(tmp.path, "SELECT track_id FROM track_bindings ORDER BY 1");
  ASSERT_EQ(rows.size(), 1u);
  EXPECT_EQ(rows[0][0], "7");
}

// Only object tracks are bound: track ids of other layers would collide with them
TEST(GraphUpdater, BindsOnlyObjectTracks) {
  test::TempDirectory tmp;
  auto dsg = test::makeSharedDsg();
  auto& graph = *dsg->graph;
  const auto objects = graph.getLayerKey("OBJECTS")->layer;
  const auto places = graph.getLayerKey("PLACES")->layer;

  auto config = makeConfig();
  config.layer_updates["PLACES"].prefix = 'p';
  GraphUpdater updater(config);
  auto store = ImageStore::open(tmp.path);
  ASSERT_TRUE(store);
  updater.setImageStore(store, 2);

  GraphUpdate round;
  round[objects] = makeUpdate(objects, {7});
  round[places] = makeUpdate(places, {7, 9});
  updater.update(round, graph);
  ASSERT_TRUE(graph.hasNode(spark_dsg::NodeSymbol('p', 1)));

  store->flush();
  const auto rows =
      test::queryStore(tmp.path, "SELECT track_id, node_id FROM track_bindings");
  ASSERT_EQ(rows.size(), 1u);
  EXPECT_EQ(rows[0][0], "7");
  EXPECT_EQ(
      rows[0][1],
      std::to_string(static_cast<spark_dsg::NodeId>(spark_dsg::NodeSymbol('O', 0))));
}

}  // namespace hydra
