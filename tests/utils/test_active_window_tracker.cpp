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
#include <glog/logging.h>
#include <gtest/gtest.h>
#include <hydra/utils/active_window_tracker.h>

namespace hydra {

namespace {

inline DynamicSceneGraph::Ptr makeGraph() {
  auto graph = std::make_shared<DynamicSceneGraph>();
  for (size_t i = 0; i < 5; ++i) {
    auto attrs = std::make_unique<NodeAttributes>();
    attrs->is_active = false;
    graph->emplaceNode(2, i, std::move(attrs));
  }

  for (size_t i = 5; i < 10; ++i) {
    auto attrs = std::make_unique<NodeAttributes>();
    attrs->is_active = true;
    graph->emplaceNode(2, i, std::move(attrs));
  }

  for (size_t i = 10; i < 15; ++i) {
    auto attrs = std::make_unique<NodeAttributes>();
    attrs->is_active = true;
    graph->emplaceNode(3, i, std::move(attrs));
  }

  return graph;
}

inline std::list<NodeId> getNodes(ActiveWindowTracker& tracker,
                                  const SceneGraphLayer& layer,
                                  bool clear = true) {
  std::list<NodeId> to_return;
  for (const auto& node : tracker.view(layer)) {
    to_return.push_back(node.id);
  }

  if (clear) {
    tracker.clear();
  }

  return to_return;
}

}  // namespace

TEST(ActiveWindowTracker, ViewCorrect) {
  const auto graph = makeGraph();
  {  // layer 2 should just be active
    ActiveWindowTracker tracker;
    const std::list<NodeId> expected{5, 6, 7, 8, 9};
    const auto result = getNodes(tracker, graph->getLayer(2));
    EXPECT_EQ(expected, result);
  }

  {  // layer 3 should just be active
    ActiveWindowTracker tracker;
    const std::list<NodeId> expected{10, 11, 12, 13, 14};
    const auto result = getNodes(tracker, graph->getLayer(3));
    EXPECT_EQ(expected, result);
  }

  {  // empty layer should be empty
    ActiveWindowTracker tracker;
    const std::list<NodeId> expected;
    const auto result = getNodes(tracker, graph->getLayer(4));
    EXPECT_EQ(expected, result);
  }
}

TEST(ActiveWindowTracker, FirstNodeStateCorrect) {
  const auto graph = makeGraph();
  ActiveWindowTracker tracker;

  // toggle active flag on for all nodes
  for (const auto& iter : graph->getLayer(2).nodes()) {
    iter.second->attributes().is_active = true;
  }

  {  // layer 2 should just be active
    const std::list<NodeId> expected{0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
    const auto result = getNodes(tracker, graph->getLayer(2));
    EXPECT_EQ(expected, result);
  }

  // toggle active flag off for all nodes
  for (const auto& iter : graph->getLayer(2).nodes()) {
    iter.second->attributes().is_active = false;
  }

  {  // layer 2 should be all previous active nodes
    const std::list<NodeId> expected{0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
    const auto result = getNodes(tracker, graph->getLayer(2));
    EXPECT_EQ(expected, result);
  }
}

TEST(ActiveWindowTracker, ViewStateCorrect) {
  const auto graph = makeGraph();
  ActiveWindowTracker tracker;

  {  // layer 2 should just be active
    const std::list<NodeId> expected{5, 6, 7, 8, 9};
    const auto result = getNodes(tracker, graph->getLayer(2));
    EXPECT_EQ(expected, result);
  }

  // toggle active flag off for all nodes
  for (const auto& iter : graph->getLayer(2).nodes()) {
    iter.second->attributes().is_active = false;
  }

  {  // layer 2 should be all previous active nodes
    const std::list<NodeId> expected{5, 6, 7, 8, 9};
    const auto result = getNodes(tracker, graph->getLayer(2), false);
    EXPECT_EQ(expected, result);
  }

  {  // repeated iteration without clearing should get the same result
    const std::list<NodeId> expected{5, 6, 7, 8, 9};
    const auto result = getNodes(tracker, graph->getLayer(2), true);
    EXPECT_EQ(expected, result);
  }

  {  // layer 2 should be empty (now that we've cleared previously active nodes)
    const std::list<NodeId> expected;
    const auto result = getNodes(tracker, graph->getLayer(2));
    EXPECT_EQ(expected, result);
  }
}

TEST(ActiveWindowTracker, RemovedNodesCorrect) {
  const auto graph = makeGraph();
  ActiveWindowTracker tracker;

  {  // layer 2 should just be active
    const std::list<NodeId> expected{5, 6, 7, 8, 9};
    const auto result = getNodes(tracker, graph->getLayer(2));
    EXPECT_EQ(expected, result);
  }

  // toggle active flag off for all nodes
  for (const auto& iter : graph->getLayer(2).nodes()) {
    iter.second->attributes().is_active = false;
  }

  // technically this is enough on its own for the tracker to not iterate over the
  // removed node
  graph->removeNode(8);

  {  // layer 2 should be all previous active nodes
    const std::list<NodeId> expected{5, 6, 7, 9};
    const auto result = getNodes(tracker, graph->getLayer(2));
    EXPECT_EQ(expected, result);
  }
}

}  // namespace hydra
