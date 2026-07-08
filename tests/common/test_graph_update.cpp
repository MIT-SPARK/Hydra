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
#include <spark_dsg/node_attributes.h>

#include "hydra_test/shared_dsg_fixture.h"

namespace hydra {

namespace {

GraphUpdater::Config makeConfig(bool archive_missing) {
  GraphUpdater::Config config;
  auto& tracker = config.layer_updates["OBJECTS"];
  tracker.prefix = 'O';
  tracker.archive_missing = archive_missing;
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

// A track present in one round and absent in the next gets archived; a track that
// reappears is re-activated by the normal update path.
TEST(GraphUpdater, ArchiveMissingTracks) {
  auto dsg = test::makeSharedDsg();
  auto& graph = *dsg->graph;
  const auto layer_id = graph.getLayerKey("OBJECTS")->layer;

  GraphUpdater updater(makeConfig(true));

  GraphUpdate round1;
  round1[layer_id] = makeUpdate(layer_id, {7, 8});
  updater.update(round1, graph);

  const spark_dsg::NodeSymbol n0('O', 0);
  const spark_dsg::NodeSymbol n1('O', 1);
  ASSERT_TRUE(graph.hasNode(n0));
  ASSERT_TRUE(graph.hasNode(n1));
  EXPECT_TRUE(graph.getNode(n0).attributes().is_active);
  EXPECT_TRUE(graph.getNode(n1).attributes().is_active);

  // round 2: track 8 vanished (left the active window)
  GraphUpdate round2;
  round2[layer_id] = makeUpdate(layer_id, {7});
  updater.update(round2, graph);
  EXPECT_TRUE(graph.getNode(n0).attributes().is_active);
  EXPECT_FALSE(graph.getNode(n1).attributes().is_active);

  // round 3: track 8 comes back -> normal update path re-activates it
  GraphUpdate round3;
  round3[layer_id] = makeUpdate(layer_id, {7, 8});
  updater.update(round3, graph);
  EXPECT_TRUE(graph.getNode(n1).attributes().is_active);
}

// Default (archive_missing=false) preserves the old always-active behavior.
TEST(GraphUpdater, NoArchivalByDefault) {
  auto dsg = test::makeSharedDsg();
  auto& graph = *dsg->graph;
  const auto layer_id = graph.getLayerKey("OBJECTS")->layer;

  GraphUpdater updater(makeConfig(false));

  GraphUpdate round1;
  round1[layer_id] = makeUpdate(layer_id, {7, 8});
  updater.update(round1, graph);

  GraphUpdate round2;
  round2[layer_id] = makeUpdate(layer_id, {7});
  updater.update(round2, graph);

  EXPECT_TRUE(graph.getNode(spark_dsg::NodeSymbol('O', 1)).attributes().is_active);
}

}  // namespace hydra
