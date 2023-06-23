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
#include <hydra/rooms/graph_filtration.h>

namespace hydra {

bool operator==(const FiltrationInfo& lhs, const FiltrationInfo& rhs) {
  return lhs.distance == rhs.distance && lhs.num_components == rhs.num_components;
}

bool operator==(const ComponentLifetime& lhs, const ComponentLifetime& rhs) {
  // lazy equality check for doubles
  return std::abs(lhs.start - rhs.start) < 1.0e-6 &&
         std::abs(lhs.end - rhs.end) < 1.0e-6;
}

std::ostream& operator<<(std::ostream& out, const ComponentLifetime& c) {
  return out << "[" << c.start << ", " << c.end << "]";
}

Filtration makeFiltration(const std::vector<size_t>& components) {
  double distance = 1.0;
  Filtration filtration;
  for (const size_t value : components) {
    filtration.push_back({distance, value});
    distance += 1.0;
  }
  return filtration;
}

void addNode(IsolatedSceneGraphLayer& layer, NodeId node, double distance) {
  auto attrs = std::make_unique<PlaceNodeAttributes>();
  attrs->distance = distance;
  layer.emplaceNode(node, std::move(attrs));
}

void addEdge(IsolatedSceneGraphLayer& layer,
             NodeId source,
             NodeId target,
             double distance) {
  auto attrs = std::make_unique<EdgeAttributes>();
  attrs->weight = distance;
  layer.insertEdge(source, target, std::move(attrs));
}

std::optional<FiltrationInfo> getLongestSequence(const Filtration& values) {
  return getLongestSequence(values, 0, values.size());
}

TEST(GraphFiltrationTests, TestSimpleSingleComponent) {
  IsolatedSceneGraphLayer layer(1);
  addNode(layer, 0, 1.0);
  addNode(layer, 1, 2.0);
  addNode(layer, 2, 3.0);
  addNode(layer, 3, 4.0);
  addEdge(layer, 0, 1, 0.2);
  addEdge(layer, 1, 2, 0.3);
  addEdge(layer, 2, 3, 0.4);

  const auto result = getGraphFiltration(layer);
  Filtration expected{
      {0.2, 1}, {0.3, 2}, {0.4, 3}, {1.0, 4}, {2.0, 3}, {3.0, 2}, {4.0, 1}};
  EXPECT_EQ(expected, result);
}

TEST(GraphFiltrationTests, TestSimpleSingleComponentMinSize) {
  IsolatedSceneGraphLayer layer(1);
  addNode(layer, 0, 1.0);
  addNode(layer, 1, 2.0);
  addNode(layer, 2, 3.0);
  addNode(layer, 3, 4.0);
  addEdge(layer, 0, 1, 0.2);
  addEdge(layer, 1, 2, 0.3);
  addEdge(layer, 2, 3, 0.4);

  const auto result = getGraphFiltration(layer, 2u, 1.0e-4);
  Filtration expected{
      {0.2, 1}, {0.3, 1}, {0.4, 1}, {1.0, 0}, {2.0, 0}, {3.0, 0}, {4.0, 0}};
  EXPECT_EQ(expected, result);
}

TEST(GraphFiltrationTests, TestDelayedEdgesSingleComponent) {
  IsolatedSceneGraphLayer layer(1);
  addNode(layer, 0, 1.0);
  addNode(layer, 1, 2.0);
  addNode(layer, 2, 3.0);
  addNode(layer, 3, 4.0);
  addEdge(layer, 0, 1, 1.5);
  addEdge(layer, 1, 2, 2.5);
  addEdge(layer, 2, 3, 3.5);

  const auto result = getGraphFiltration(layer);
  Filtration expected{{1.0, 1}, {2.0, 1}, {3.0, 1}, {4.0, 1}};
  EXPECT_EQ(expected, result);
}

TEST(GraphFiltrationTests, TestRepeatedDistanceSingleComponent) {
  IsolatedSceneGraphLayer layer(1);
  addNode(layer, 0, 1.0);
  addNode(layer, 1, 2.0);
  addNode(layer, 2, 3.0);
  addNode(layer, 3, 4.0);
  addEdge(layer, 0, 1, 0.5);
  addEdge(layer, 1, 2, 0.5);
  addEdge(layer, 2, 3, 0.5);

  const auto result = getGraphFiltration(layer);
  Filtration expected{{0.5, 1}, {1.0, 4}, {2.0, 3}, {3.0, 2}, {4.0, 1}};
  EXPECT_EQ(expected, result);
}

TEST(GraphFiltrationTests, TestDelayedEdgeBothNodes) {
  IsolatedSceneGraphLayer layer(1);
  addNode(layer, 0, 1.0);
  addNode(layer, 1, 2.0);
  addNode(layer, 2, 3.0);
  addNode(layer, 3, 4.0);
  addEdge(layer, 1, 3, 5.0);

  const auto result = getGraphFiltration(layer);
  Filtration expected{{1.0, 3}, {2.0, 2}, {3.0, 2}, {4.0, 1}};
  EXPECT_EQ(expected, result);
}

TEST(GraphFiltrationTests, TestEdgesFiltrationOnly) {
  IsolatedSceneGraphLayer layer(1);
  addNode(layer, 0, 1.0);
  addNode(layer, 1, 2.0);
  addNode(layer, 2, 3.0);
  addNode(layer, 3, 4.0);
  addEdge(layer, 0, 1, 0.4);
  addEdge(layer, 1, 2, 0.5);
  addEdge(layer, 2, 3, 0.6);

  BarcodeTracker tracker;
  const auto result = getGraphFiltration(
      layer,
      tracker,
      1.0e-4,
      [](const DisjointSet& components) { return components.sizes.size(); },
      false);
  Filtration expected{{0.4, 1}, {0.5, 2}, {0.6, 3}};
  EXPECT_EQ(expected, result);
}

TEST(GraphFiltrationTests, TestBarcodeFiltration) {
  IsolatedSceneGraphLayer layer(1);
  addNode(layer, 0, 1.0);
  addNode(layer, 1, 2.0);
  addNode(layer, 2, 3.0);
  addNode(layer, 3, 4.0);
  addEdge(layer, 0, 1, 0.4);
  addEdge(layer, 1, 2, 0.5);
  addEdge(layer, 2, 3, 0.6);

  BarcodeTracker tracker;
  const auto result = getGraphFiltration(
      layer,
      tracker,
      1.0e-4,
      [](const DisjointSet& components) { return components.sizes.size(); },
      false);
  Filtration expected{{0.4, 1}, {0.5, 2}, {0.6, 3}};
  EXPECT_EQ(expected, result);

  std::unordered_map<NodeId, ComponentLifetime> expected_barcodes{
      {0, {0.4, 1.0}},
      {1, {0.5, 2.0}},
      {2, {0.6, 3.0}},
      {3, {0.0, 4.0}},
  };
  EXPECT_EQ(expected_barcodes, tracker.barcodes);
}

TEST(GraphFiltrationTests, TestLongestSequence) {
  {  // empty values -> no best index
    Filtration values;
    EXPECT_FALSE(getLongestSequence(values));
  }

  {  // first value is 0 -> no best index
    const auto values = makeFiltration({0});
    EXPECT_FALSE(getLongestSequence(values));
  }

  {  // all values are 0 -> no best index
    const auto values = makeFiltration({0, 0, 0, 0});
    EXPECT_FALSE(getLongestSequence(values));
  }

  {  // single value -> 0
    const auto values = makeFiltration({1});
    auto result = getLongestSequence(values);
    ASSERT_TRUE(result);
    FiltrationInfo expected{1.0, 1};
    EXPECT_EQ(*result, expected);
  }

  {  // all one non-zero sequence
    const auto values = makeFiltration({1, 1, 1, 1});
    auto result = getLongestSequence(values);
    ASSERT_TRUE(result);
    FiltrationInfo expected{1.0, 1};
    EXPECT_EQ(*result, expected);
  }

  {  // two sequences
    const auto values = makeFiltration({1, 1, 2, 2, 2});
    auto result = getLongestSequence(values);
    ASSERT_TRUE(result);
    FiltrationInfo expected{3.0, 2};
    EXPECT_EQ(*result, expected);
  }

  {  // arbitrary set of values
    const auto values = makeFiltration({1, 4, 2, 3, 2, 2, 5, 6, 6, 6, 9, 1, 1, 1, 2});
    auto result = getLongestSequence(values);
    ASSERT_TRUE(result);
    FiltrationInfo expected{8.0, 6};
    EXPECT_EQ(*result, expected);
  }

  {  // no sequence
    const auto values = makeFiltration({1, 2, 3, 4, 3, 2, 1, 0, 0, 0});
    auto result = getLongestSequence(values);
    ASSERT_TRUE(result);
    FiltrationInfo expected{1.0, 1};
    EXPECT_EQ(*result, expected);
  }
}

TEST(GraphFiltrationTests, TestGetTrimmedFiltration) {
  const auto values = makeFiltration({1, 2, 3, 4, 3, 2, 1, 0, 0, 0});

  {  // peak not inside window
    const auto result = getTrimmedFiltration(values, 1.1, 3.1);
    std::pair<size_t, size_t> expected{1, 2};
    EXPECT_EQ(expected, result);
  }

  {  // peak inside window
    const auto result = getTrimmedFiltration(values, 1.1, 10.1);
    std::pair<size_t, size_t> expected{1, 4};
    EXPECT_EQ(expected, result);
  }
}

TEST(GraphFiltrationTests, TestOutputOperators) {
  {  // test that filtration info outputs correctly
    FiltrationInfo info{1.5, 1};
    std::stringstream ss;
    ss << info;
    EXPECT_EQ(ss.str(), "<dist=1.5, size=1>");
  }

  {  // test that a filtration outputs correctly
    Filtration filtration{{1.5, 1}, {2.5, 2}};
    std::stringstream ss;
    ss << filtration;
    EXPECT_EQ(ss.str(), "[{d=1.5, s=1}, {d=2.5, s=2}]");
  }
}

TEST(GraphFiltrationTests, DISABLED_TestLongestLifetimeDilation) {
  {  // empty input -> no output
    Filtration filtration;
    LifetimeMap lifetimes;
    const auto result = getLongestLifetimeDilation(filtration, lifetimes, 0.1, 0, 5);
    EXPECT_FALSE(result);
  }

  {  // non-empty input -> correct output
    Filtration filtration{{0.1, 1}, {0.2, 2}, {0.3, 3}, {0.4, 4}, {0.5, 5}};
    LifetimeMap lifetimes{{0, {-50, 0.15}},
                          {1, {0.1, 0.4}},
                          {2, {0.1, 0.4}},
                          {3, {0.4, 0.9}},
                          {4, {0.4, 0.9}}};
    const auto result = getLongestLifetimeDilation(filtration, lifetimes, 0.1, 1, 5);
    ASSERT_TRUE(result);
    EXPECT_NEAR(result->distance, 0.2, 1.0e-9);
    EXPECT_EQ(result->num_components, 2u);
  }
}

Filtration makePlateauFiltration(const std::vector<size_t>& y_values) {
  Filtration filtration;
  for (size_t i = 0; i < y_values.size(); ++i) {
    filtration.push_back({0.1 * (i + 1), y_values[i]});
  }
  return filtration;
}

TEST(GraphFiltrationTests, TestBestPlateau) {
  {  // empty input -> no output
    Filtration filtration;
    const auto result = getBestPlateau(filtration, 0.1, 0, 5);
    EXPECT_FALSE(result);
  }

  const auto filtration = makePlateauFiltration({1, 1, 1, 1, 2, 2, 3, 2});
  {  // test case 1: low enough ratio that second plateau gets selected
    const auto result = getBestPlateau(filtration, 0.1, 0, filtration.size());
    ASSERT_TRUE(result);
    EXPECT_NEAR(result->distance, 0.5, 1.0e-9);
    EXPECT_EQ(result->num_components, 2u);
  }

  {  // test case 2: longest plateau only
    const auto result = getBestPlateau(filtration, 0.4, 0, filtration.size());
    ASSERT_TRUE(result);
    EXPECT_NEAR(result->distance, 0.1, 1.0e-9);
    EXPECT_EQ(result->num_components, 1u);
  }

  {  // test case 3: max number of components with 0.0
    const auto result = getBestPlateau(filtration, 0.0, 0, filtration.size());
    ASSERT_TRUE(result);
    EXPECT_NEAR(result->distance, 0.7, 1.0e-9);
    EXPECT_EQ(result->num_components, 3u);
  }
}

}  // namespace hydra
