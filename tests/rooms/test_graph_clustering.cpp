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
#include <hydra/rooms/graph_clustering.h>

namespace hydra {

TEST(GraphClusteringTests, ClearResultsCorrect) {
  ClusterResults results{{{0, {1, 2, 3}}}, {{1, 2}, {2, 3}}, 5, true};
  EXPECT_TRUE(results.valid);
  EXPECT_FALSE(results.labels.empty());
  EXPECT_FALSE(results.clusters.empty());

  results.clear();
  EXPECT_FALSE(results.valid);
  EXPECT_TRUE(results.labels.empty());
  EXPECT_TRUE(results.clusters.empty());
}

TEST(GraphClusteringTests, FillFromClusters) {
  ClusterResults results;
  InitialClusters clusters{{1, 2, 3}, {4, 5, 6}};
  results.fillFromInitialClusters(clusters);

  EXPECT_TRUE(results.valid);
  EXPECT_EQ(results.total_iters, 0u);

  std::map<NodeId, size_t> expected_labels{
      {1, 0}, {2, 0}, {3, 0}, {4, 1}, {5, 1}, {6, 1}};
  EXPECT_EQ(results.labels, expected_labels);

  ClusterResults::Clusters expected_clusters{{0, {1, 2, 3}}, {1, {4, 5, 6}}};
  EXPECT_EQ(results.clusters, expected_clusters);
}

TEST(GraphClusteringTests, ModularityClusteringCorrect) {
  IsolatedSceneGraphLayer layer(1);
  for (size_t i = 0; i < 10; ++i) {
    layer.emplaceNode(i, std::make_unique<NodeAttributes>());
  }

  // first clique (3 nodes)
  layer.insertEdge(0, 1, std::make_unique<EdgeAttributes>(1.0));
  layer.insertEdge(0, 2, std::make_unique<EdgeAttributes>(1.0));
  layer.insertEdge(1, 2, std::make_unique<EdgeAttributes>(1.0));
  // bridge between cliques (biased so 3 is in first cluster)
  layer.insertEdge(2, 3, std::make_unique<EdgeAttributes>(0.1));
  layer.insertEdge(3, 4, std::make_unique<EdgeAttributes>(0.01));
  // second clique (5 nodes)
  layer.insertEdge(4, 5, std::make_unique<EdgeAttributes>(1.0));
  layer.insertEdge(4, 6, std::make_unique<EdgeAttributes>(1.0));
  layer.insertEdge(4, 7, std::make_unique<EdgeAttributes>(1.0));
  layer.insertEdge(4, 8, std::make_unique<EdgeAttributes>(1.0));
  layer.insertEdge(5, 6, std::make_unique<EdgeAttributes>(1.0));
  layer.insertEdge(5, 7, std::make_unique<EdgeAttributes>(1.0));
  layer.insertEdge(5, 8, std::make_unique<EdgeAttributes>(1.0));
  layer.insertEdge(6, 7, std::make_unique<EdgeAttributes>(1.0));
  layer.insertEdge(6, 8, std::make_unique<EdgeAttributes>(1.0));
  layer.insertEdge(7, 8, std::make_unique<EdgeAttributes>(1.0));
  // extra connection to second clique
  layer.insertEdge(6, 9, std::make_unique<EdgeAttributes>(0.3));

  InitialClusters initial_clusters{{1},
                                   {4, 6, 8}};  // seed clustering with both cliques
  auto cluster_results = clusterGraphByModularity(layer, initial_clusters, 4);
  const auto& labels = cluster_results.labels;
  for (size_t i = 0; i < 10; ++i) {
    ASSERT_TRUE(labels.count(i));
  }

  // expect that first clique is in one cluster
  EXPECT_EQ(0u, labels.at(0));
  EXPECT_EQ(0u, labels.at(1));
  EXPECT_EQ(0u, labels.at(2));
  // expect that bridge node is in first cluster (because of edge weight difference)
  EXPECT_EQ(0u, labels.at(3));
  // expect that second clique is in once cluster
  EXPECT_EQ(1u, labels.at(4));
  EXPECT_EQ(1u, labels.at(5));
  EXPECT_EQ(1u, labels.at(5));
  EXPECT_EQ(1u, labels.at(7));
  EXPECT_EQ(1u, labels.at(8));
  // expect that extra node is connected to second clique
  EXPECT_EQ(1u, labels.at(9));

  ASSERT_EQ(2u, cluster_results.clusters.size());
  ASSERT_TRUE(cluster_results.clusters.count(0));
  ASSERT_TRUE(cluster_results.clusters.count(1));
  std::unordered_set<NodeId> first_cluster{0, 1, 2, 3};
  std::unordered_set<NodeId> second_cluster{4, 5, 6, 7, 8, 9};
  EXPECT_EQ(first_cluster, cluster_results.clusters.at(0));
  EXPECT_EQ(second_cluster, cluster_results.clusters.at(1));
}

}  // namespace hydra
