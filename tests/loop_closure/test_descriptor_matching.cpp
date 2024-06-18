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
#include <hydra/loop_closure/descriptor_matching.h>

namespace hydra::lcd {

TEST(LoopClosureModuleMatchingTests, TestCosineDistanceFixedSize) {
  Descriptor d1;
  d1.values.resize(5, 1);
  d1.values << 1.0f, 2.0f, 3.0f, 4.0f, 5.0f;
  d1.normalized = false;

  Descriptor d2;
  d2.values.resize(5, 1);
  d2.values << 1.0f, 2.0f, 3.0f, 4.0f, 5.0f;
  d2.normalized = false;

  {  // identical vectors: score of 1
    float result = computeCosineDistance(d1, d2);
    EXPECT_NEAR(1.0f, result, 1.0e-6f);
  }

  {  // one vector is 0: score of 0
    d1.values = Eigen::MatrixXf::Zero(5, 1);

    float result = computeCosineDistance(d1, d2);
    EXPECT_NEAR(0.0f, result, 1.0e-6f);
  }

  {  // one normalized vector
    d1.values << 1.0f, 2.0f, 3.0f, 4.0f, 5.0f;
    d1.values.normalize();
    d1.normalized = true;

    float result = computeCosineDistance(d1, d2);
    EXPECT_NEAR(1.0f, result, 1.0e-6f);
  }

  {  // other normalized vector
    d1.values << 1.0f, 2.0f, 3.0f, 4.0f, 5.0f;
    d2.values.normalize();
    d1.normalized = false;
    d2.normalized = true;

    float result = computeCosineDistance(d1, d2);
    EXPECT_NEAR(1.0f, result, 1.0e-6f);
  }

  {  // both normalized
    d1.values.normalize();
    d2.values.normalize();
    d1.normalized = true;
    d2.normalized = true;

    float result = computeCosineDistance(d1, d2);
    EXPECT_NEAR(1.0f, result, 1.0e-6f);
  }
}

TEST(LoopClosureModuleMatchingTests, TestCosineDistanceVariableSize) {
  Descriptor d1;
  d1.values.resize(5, 1);
  d1.words.resize(5, 1);
  d1.values << 1.0f, 2.0f, 3.0f, 4.0f, 5.0f;
  d1.words << 1, 2, 3, 4, 5;
  d1.normalized = false;

  Descriptor d2;
  d2.values.resize(5, 1);
  d2.words.resize(5, 1);
  d2.values << 1.0f, 2.0f, 3.0f, 4.0f, 5.0f;
  d2.words << 1, 2, 3, 4, 5;
  d2.normalized = false;

  {  // vectors are the same: score of 1
    float result = computeCosineDistance(d1, d2);
    EXPECT_NEAR(1.0f, result, 1.0e-6f);
  }

  {  // vectors are essentially the same: score of 1
    d1.words << 1, 2, 4, 5, 7;
    d1.values << 1.0f, 2.0f, 3.0f, 0.0f, 6.0f;
    d2.words << 1, 2, 3, 4, 7;
    d2.values << 1.0f, 2.0f, 0.0f, 3.0f, 6.0f;

    float result = computeCosineDistance(d1, d2);
    EXPECT_NEAR(1.0f, result, 1.0e-6f);
  }
}

TEST(LoopClosureModuleMatchingTests, TestComputeL1DistanceBow) {
  Descriptor d1;
  d1.values.resize(5, 1);
  d1.words.resize(5, 1);
  d1.values << 1.0f, 2.0f, 3.0f, 4.0f, 5.0f;
  d1.words << 1, 2, 3, 4, 5;
  d1.normalized = false;

  Descriptor d2;
  d2.values.resize(5, 1);
  d2.words.resize(5, 1);
  d2.values << 1.0f, 2.0f, 3.0f, 4.0f, 5.0f;
  d2.words << 1, 2, 3, 4, 5;
  d2.normalized = false;

  {  // vectors are the same: score of 1
    float result = computeL1Distance(d1, d2);
    EXPECT_NEAR(0.0f, result, 1.0e-6f);
  }

  {  // vectors are essentially the same: score of 1
    d1.words << 1, 2, 4, 5, 7;
    d1.values << 1.0f, 2.0f, 3.0f, 0.0f, 6.0f;
    d2.words << 1, 2, 3, 4, 7;
    d2.values << 1.0f, 2.0f, 0.0f, 3.0f, 6.0f;

    float result = computeL1Distance(d1, d2);
    EXPECT_NEAR(0.0f, result, 1.0e-6f);
  }

  {  // vectors have different norms: score is less than 1
    d1.words << 1, 2, 4, 5, 7;
    d1.values << 1.0f, 2.0f, 1.0f, 0.0f, 6.0f;
    d2.words << 1, 2, 3, 4, 7;
    d2.values << 1.0f, 2.0f, 0.0f, 9.0f, 6.0f;

    float result = computeL1Distance(d1, d2);
    EXPECT_LT(0.0f, result);
  }
}

TEST(LoopClosureModuleMatchingTests, TestComputeL1DistanceFixed) {
  Descriptor d1;
  d1.values.resize(5, 1);
  d1.values << 1.0f, 2.0f, 3.0f, 4.0f, 5.0f;
  d1.normalized = false;

  Descriptor d2;
  d2.values.resize(5, 1);
  d2.values << 1.0f, 2.0f, 3.0f, 4.0f, 5.0f;
  d2.normalized = false;

  {  // vectors are the same: score of 1
    float result = computeL1Distance(d1, d2);
    EXPECT_NEAR(0.0f, result, 1.0e-6f);
  }

  {  // vectors are essentially the same: score of 1
    d1.values << 1.0f, 2.0f, 3.0f, 0.0f, 6.0f;
    d2.values << 1.0f, 2.0f, 0.0f, 3.0f, 6.0f;

    float result = computeL1Distance(d1, d2);
    EXPECT_NEAR(0.5f, result, 1.0e-6f);
  }
}

TEST(LoopClosureModuleMatchingTests, TestComputeL1DistanceEmpty) {
  Descriptor d1;
  d1.values.resize(5, 1);
  d1.values.setZero();
  d1.normalized = false;

  Descriptor d2;
  d2.values.resize(5, 1);
  d2.values.setZero();
  d2.normalized = false;

  {
    float dist = computeL1Distance(d1, d2);
    EXPECT_NEAR(0.0f, dist, 1.0e-6f);
    float result = computeDescriptorScore(d1, d2, DescriptorScoreType::L1);
    EXPECT_NEAR(1.0f, result, 1.0e-6f);
  }
}

TEST(LoopClosureModuleMatchingTests, TestComputeCosDistanceEmpty) {
  Descriptor d1;
  d1.values.resize(5, 1);
  d1.values.setZero();
  d1.normalized = false;

  Descriptor d2;
  d2.values.resize(5, 1);
  d2.values.setZero();
  d2.normalized = false;

  {
    float dist = computeCosineDistance(d1, d2);
    EXPECT_NEAR(1.0f, dist, 1.0e-6f);
    float result = computeDescriptorScore(d1, d2, DescriptorScoreType::COSINE);
    EXPECT_NEAR(1.0f, result, 1.0e-6f);
  }
}

template <typename... Ts>
Descriptor::Ptr makeDescriptor(Ts... args) {
  Descriptor::Ptr descriptor(new Descriptor());

  std::vector<float> data{args...};
  descriptor->values = Eigen::VectorXf(data.size(), 1);
  std::memcpy(descriptor->values.data(),
              data.data(),
              descriptor->values.size() * sizeof(float));
  return descriptor;
}

void fillDescriptor(Descriptor& descriptor, NodeId root, std::set<NodeId> nodes) {
  descriptor.root_node = root;
  descriptor.nodes = nodes;
}

TEST(LoopClosureModuleMatchingTests, SearchDescriptorsNoValid) {
  Descriptor::Ptr query = makeDescriptor(1.0f);

  DescriptorMatchConfig config;
  config.min_score = 0.8f;
  config.max_registration_matches = 1;

  std::set<NodeId> valid_matches;

  DescriptorCache descriptors;
  descriptors[1] = makeDescriptor(0.9f);
  descriptors[2] = makeDescriptor(0.9f);
  descriptors[3] = makeDescriptor(0.9f);

  std::map<NodeId, std::set<NodeId>> root_leaf_map;
  root_leaf_map[1] = {};
  root_leaf_map[2] = {};
  root_leaf_map[3] = {};
  LayerSearchResults results =
      searchDescriptors(*query, config, valid_matches, descriptors, root_leaf_map, 5);
  ASSERT_EQ(results.score.size(), 1u);
  EXPECT_EQ(results.score.front(), 0.0);
  EXPECT_TRUE(results.valid_matches.empty());
  EXPECT_TRUE(results.query_nodes.empty());
  EXPECT_TRUE(results.match_nodes.empty());
}

TEST(LoopClosureModuleMatchingTests, SearchDescriptorsValidNoMatches) {
  Descriptor::Ptr query = makeDescriptor(1.0f);

  DescriptorMatchConfig config;
  config.min_score = 1.5f;
  config.max_registration_matches = 1;

  std::set<NodeId> valid_matches{1, 2, 3};

  DescriptorCache descriptors;
  descriptors[1] = makeDescriptor(0.9f);
  descriptors[2] = makeDescriptor(0.9f);
  descriptors[3] = makeDescriptor(0.9f);

  std::map<NodeId, std::set<NodeId>> root_leaf_map;
  root_leaf_map[1] = {};
  root_leaf_map[2] = {};
  root_leaf_map[3] = {};

  LayerSearchResults results =
      searchDescriptors(*query, config, valid_matches, descriptors, root_leaf_map, 5);
  ASSERT_EQ(results.score.size(), 1u);
  EXPECT_NEAR(results.score.front(), 1.0, 1.0e-9);
  EXPECT_TRUE(results.valid_matches.empty());
  EXPECT_TRUE(results.query_nodes.empty());
  EXPECT_TRUE(results.match_nodes.empty());
}

TEST(LoopClosureModuleMatchingTests, SearchDescriptorsValidSomeMatches) {
  Descriptor::Ptr query = makeDescriptor(1.0f, 0.0f);
  fillDescriptor(*query, 0, {13, 14, 15});

  DescriptorMatchConfig config;
  config.min_score = 0.7f;
  config.min_registration_score = 0.9f;
  config.min_time_separation_s = 0.0;
  config.max_registration_matches = 1;

  std::set<NodeId> valid_matches{1, 2, 3};

  DescriptorCache descriptors;
  descriptors[1] = makeDescriptor(0.9f, 0.1f);
  fillDescriptor(*descriptors[1], 1, {4, 5, 6});
  descriptors[2] = makeDescriptor(0.9f, 0.9f);
  fillDescriptor(*descriptors[2], 2, {7, 8, 9});
  descriptors[3] = makeDescriptor(0.9f, 0.05f);
  fillDescriptor(*descriptors[3], 3, {10, 11, 12});

  std::map<NodeId, std::set<NodeId>> root_leaf_map;
  root_leaf_map[1] = {};
  root_leaf_map[2] = {};
  root_leaf_map[3] = {};

  LayerSearchResults results =
      searchDescriptors(*query, config, valid_matches, descriptors, root_leaf_map, 5);
  ASSERT_GT(results.score.size(), 0u);

  EXPECT_GT(results.score[0], config.min_score);
  EXPECT_EQ(3u, results.match_root[0]);

  std::set<NodeId> expected_matches{1, 3};
  EXPECT_EQ(expected_matches, results.valid_matches);

  std::set<NodeId> expected_match_nodes{10, 11, 12};
  EXPECT_EQ(expected_match_nodes, results.match_nodes[0]);
  EXPECT_EQ(3u, results.match_root[0]);

  std::set<NodeId> expected_query_nodes{13, 14, 15};
  EXPECT_EQ(expected_query_nodes, results.query_nodes);
  EXPECT_EQ(0u, results.query_root);
}

TEST(LoopClosureModuleMatchingTests, searchLeafDescriptorsNoValid) {
  Descriptor::Ptr query = makeDescriptor(1.0f);

  DescriptorMatchConfig config;
  config.max_registration_matches = 1;

  std::set<NodeId> valid_matches;

  DescriptorCacheMap descriptors;
  descriptors[1] = DescriptorCache();
  descriptors[2] = DescriptorCache();
  descriptors[1][1] = makeDescriptor(0.9f);
  descriptors[1][2] = makeDescriptor(0.9f);
  descriptors[1][3] = makeDescriptor(0.9f);
  descriptors[2][4] = makeDescriptor(0.9f);

  LayerSearchResults results =
      searchLeafDescriptors(*query, config, valid_matches, descriptors, 10);
  EXPECT_TRUE(results.score.empty());
  EXPECT_TRUE(results.valid_matches.empty());
  EXPECT_TRUE(results.query_nodes.empty());
  EXPECT_TRUE(results.match_nodes.empty());
}

TEST(LoopClosureModuleMatchingTests, SearchLeafDescriptorsAllValid) {
  Descriptor::Ptr query = makeDescriptor(1.0f);

  DescriptorMatchConfig config;
  config.min_time_separation_s = 0.0;
  config.max_registration_matches = 1;
  config.min_score = 0.9;
  config.min_registration_score = 0.9;

  std::set<NodeId> valid_matches{1, 2};

  DescriptorCacheMap descriptors;
  descriptors[1] = DescriptorCache();
  descriptors[2] = DescriptorCache();
  descriptors[1][1] = makeDescriptor(0.9f);
  descriptors[1][2] = makeDescriptor(0.9f);
  descriptors[1][3] = makeDescriptor(0.9f);
  descriptors[2][4] = makeDescriptor(0.9f);

  LayerSearchResults results =
      searchLeafDescriptors(*query, config, valid_matches, descriptors, 10);
  ASSERT_GT(results.score.size(), 0u);
  EXPECT_EQ(1.0f, results.score[0]);
  std::set<NodeId> expected_matches{1, 2, 3, 4};
  EXPECT_EQ(expected_matches, results.valid_matches);

  EXPECT_TRUE(results.query_nodes.empty());
  std::set<NodeId> expected_match_nodes{1};
  EXPECT_EQ(expected_match_nodes, results.match_nodes[0]);
}

TEST(LoopClosureModuleMatchingTests, SearchLeafDescriptorsTimeSeparation) {
  Descriptor::Ptr query = makeDescriptor(1.0f);

  DescriptorMatchConfig config;
  config.min_time_separation_s = 10.0;
  config.max_registration_matches = 1;

  std::set<NodeId> valid_matches{1, 2};

  DescriptorCacheMap descriptors;
  descriptors[1] = DescriptorCache();
  descriptors[2] = DescriptorCache();
  descriptors[1][1] = makeDescriptor(0.9f);
  descriptors[1][2] = makeDescriptor(0.9f);
  descriptors[1][3] = makeDescriptor(0.9f);
  descriptors[2][4] = makeDescriptor(0.9f);

  LayerSearchResults results =
      searchLeafDescriptors(*query, config, valid_matches, descriptors, 10);
  EXPECT_TRUE(results.score.empty());
  EXPECT_TRUE(results.valid_matches.empty());
  EXPECT_TRUE(results.query_nodes.empty());
  EXPECT_TRUE(results.match_nodes.empty());
}

TEST(LoopClosureModuleMatchingTests, SearchLeafDescriptorsMaxRegistrationMatches) {
  Descriptor::Ptr query = makeDescriptor(1.0f, 0.0f);
  fillDescriptor(*query, 0, {13, 14, 15});

  DescriptorMatchConfig config;
  config.min_score = 0.7f;
  config.min_registration_score = 0.7f;
  config.min_score_ratio = 0.3;
  config.min_time_separation_s = 0.0;
  config.max_registration_matches = 3;
  config.min_match_separation_m = 0.0;

  std::set<NodeId> valid_matches{1, 2, 3};

  DescriptorCache descriptors;
  descriptors[1] = makeDescriptor(0.9f, 0.1f);
  fillDescriptor(*descriptors[1], 1, {4, 5, 6});
  descriptors[2] = makeDescriptor(0.9f, 0.9f);
  fillDescriptor(*descriptors[2], 2, {7, 8, 9});
  descriptors[3] = makeDescriptor(0.9f, 0.05f);
  fillDescriptor(*descriptors[3], 3, {10, 11, 12});

  std::map<NodeId, std::set<NodeId>> root_leaf_map;
  root_leaf_map[1] = {};
  root_leaf_map[2] = {};
  root_leaf_map[3] = {};

  LayerSearchResults results =
      searchDescriptors(*query, config, valid_matches, descriptors, root_leaf_map, 5);
  EXPECT_EQ(2u, results.score.size());
  EXPECT_EQ(2u, results.match_root.size());
  EXPECT_EQ(2u, results.match_nodes.size());
}

}  // namespace hydra::lcd
