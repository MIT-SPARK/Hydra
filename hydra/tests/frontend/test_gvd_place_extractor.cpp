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
#include <hydra/active_window/volumetric_window.h>
#include <hydra/frontend/gvd_place_extractor.h>
#include <spark_dsg/node_symbol.h>

#include <limits>

namespace hydra {
namespace {

using spark_dsg::NodeSymbol;

class TestGraphExtractor : public places::GraphExtractor {
 public:
  TestGraphExtractor() : GraphExtractor(Config{}, 1.0f) {}

  using GraphExtractor::graph_;
  using GraphExtractor::gvd_;
  using GraphExtractor::overlap_edges_;
  using GraphExtractor::updateCompressedNodes;
  using GraphExtractor::updateOverlapEdges;
};

class TestGvdPlaceExtractor : public GvdPlaceExtractor {
 public:
  explicit TestGvdPlaceExtractor(size_t min_component_size = 0)
      : GvdPlaceExtractor(makeConfig(min_component_size)) {
    graph_extractor_ = std::make_unique<TestGraphExtractor>();
  }

  using GvdPlaceExtractor::gvd_;
  using GvdPlaceExtractor::tsdf_interpolator_;

  TestGraphExtractor& extractor() {
    return static_cast<TestGraphExtractor&>(*graph_extractor_);
  }

 private:
  static Config makeConfig(size_t min_component_size) {
    Config config;
    config.min_component_size = min_component_size;
    return config;
  }
};

ActiveWindowOutput makeInput(uint64_t timestamp_ns,
                             const VolumetricMap::Config& config = {}) {
  auto data = std::make_shared<InputData>(nullptr);
  data->timestamp_ns = timestamp_ns;
  data->world_T_body = Eigen::Isometry3d::Identity();
  ActiveWindowOutput msg(data);
  msg.setMap(std::make_shared<VolumetricMap>(config));
  return msg;
}

}  // namespace

TEST(GvdPlaceExtractor, FinalizesBoundaryAfterCompressedNodeIsCleared) {
  TestGvdPlaceExtractor frontend;
  auto& extractor = frontend.extractor();
  SharedDsgInfo dsg({});
  const NodeSymbol boundary('p', 0);
  const NodeSymbol active('p', 1);

  extractor.gvd_.add({0, 0, 0}, 3.0, 3);
  extractor.gvd_.add({4, 0, 0}, 3.0, 3);
  extractor.updateCompressedNodes();
  extractor.graph_.add(0, 1);
  extractor.overlap_edges_.insert({0, 1});
  extractor.archiveIndex({0, 0, 0});
  frontend.updateGraph(1, *dsg.graph);

  ASSERT_EQ(extractor.gvd().compressed().count(0), 0u);
  ASSERT_TRUE(dsg.graph->getNode(boundary).attributes().is_active);

  extractor.updateCompressedNodes();
  extractor.updateOverlapEdges();
  ASSERT_TRUE(extractor.graph().has(0));
  extractor.archiveIndex({4, 0, 0});
  frontend.updateGraph(2, *dsg.graph);

  EXPECT_FALSE(dsg.graph->getNode(boundary).attributes().is_active);
  EXPECT_FALSE(dsg.graph->getNode(active).attributes().is_active);
  EXPECT_TRUE(dsg.graph->hasEdge(boundary, active));
}

TEST(GvdPlaceExtractor, PublishesFinalAttributesAndEdgesBeforePruning) {
  TestGvdPlaceExtractor frontend;
  auto& places = frontend.extractor().graph_;
  SharedDsgInfo dsg({});
  const NodeSymbol first('p', 0);
  const NodeSymbol second('p', 1);

  places.add(0).distance = 2.5;
  places.add(1).distance = 3.0;
  places.add(0, 1).weight = 1.5;
  places.archive(0);
  places.archive(1);
  frontend.updateGraph(10, *dsg.graph);

  const auto& attrs =
      dsg.graph->getNode(first).attributes<spark_dsg::PlaceNodeAttributes>();
  EXPECT_FALSE(attrs.is_active);
  EXPECT_EQ(attrs.distance, 2.5);
  EXPECT_EQ(attrs.last_update_time_ns, 10u);
  EXPECT_EQ(dsg.graph->getEdge(first, second).info->weight, 1.5);
  EXPECT_FALSE(dsg.graph->getNode(second).attributes().is_active);
  EXPECT_EQ(places.num_nodes(), 0u);
  EXPECT_TRUE(places.finalized_nodes().empty());
}

TEST(GvdPlaceExtractor, PublishesDeletionWithoutArchivingOtherNodes) {
  TestGvdPlaceExtractor frontend;
  auto& places = frontend.extractor().graph_;
  SharedDsgInfo dsg({});
  const NodeSymbol removed('p', 0);
  const NodeSymbol retained('p', 1);

  places.add(0);
  places.add(1);
  places.add(0, 1);
  frontend.updateGraph(1, *dsg.graph);
  places.remove(0);
  frontend.updateGraph(2, *dsg.graph);

  EXPECT_FALSE(dsg.graph->hasNode(removed));
  EXPECT_FALSE(dsg.graph->hasEdge(removed, retained));
  EXPECT_TRUE(dsg.graph->getNode(retained).attributes().is_active);
  EXPECT_TRUE(places.deleted_nodes().empty());
  EXPECT_TRUE(places.deleted_edges().empty());
}

TEST(GvdPlaceExtractor, FiltersSmallPartialComponentsEvenWithHistoricalNeighbors) {
  TestGvdPlaceExtractor frontend(3);
  auto& places = frontend.extractor().graph_;
  SharedDsgInfo dsg({});
  const NodeSymbol first('p', 0);
  const NodeSymbol boundary('p', 1);
  const NodeSymbol active('p', 2);

  places.add(0);
  places.add(1);
  places.add(2);
  places.add(0, 1);
  places.add(1, 2);
  places.archive(0);
  places.archive(1);
  frontend.updateGraph(1, *dsg.graph);
  ASSERT_TRUE(dsg.graph->hasEdge(first, boundary));

  // Characterize the existing policy: only the two retained nodes are counted.
  frontend.updateGraph(2, *dsg.graph);
  EXPECT_TRUE(dsg.graph->hasNode(first));
  EXPECT_FALSE(dsg.graph->hasNode(boundary));
  EXPECT_FALSE(dsg.graph->hasNode(active));
}

TEST(GvdPlaceExtractor, InvalidAttributesPreserveInactiveRecordAndSkipEdges) {
  TestGvdPlaceExtractor frontend;
  auto& places = frontend.extractor().graph_;
  SharedDsgInfo dsg({});
  const NodeSymbol archived('p', 0);
  const NodeSymbol active('p', 1);

  places.add(0).distance = 2.0;
  places.add(1);
  frontend.updateGraph(1, *dsg.graph);
  dsg.graph->getNode(archived).attributes().is_active = false;
  places.at(0).distance = std::numeric_limits<double>::infinity();
  places.add(0, 1);
  frontend.updateGraph(2, *dsg.graph);

  const auto& attrs =
      dsg.graph->getNode(archived).attributes<spark_dsg::PlaceNodeAttributes>();
  EXPECT_FALSE(attrs.is_active);
  EXPECT_EQ(attrs.distance, 2.0);
  EXPECT_FALSE(dsg.graph->hasEdge(archived, active));
}

TEST(GvdPlaceExtractor, InvalidActiveNodeIsRemoved) {
  TestGvdPlaceExtractor frontend;
  auto& places = frontend.extractor().graph_;
  SharedDsgInfo dsg({});
  const NodeSymbol node('p', 0);
  places.add(0);
  frontend.updateGraph(1, *dsg.graph);

  places.at(0).position.x() = std::numeric_limits<double>::quiet_NaN();
  frontend.updateGraph(2, *dsg.graph);
  EXPECT_FALSE(dsg.graph->hasNode(node));
}

TEST(GvdPlaceExtractor, InvalidFirstInputDoesNotPublish) {
  GvdPlaceExtractor frontend({});
  VolumetricMap::Config config;
  config.truncation_distance = 0.1f;
  const auto msg = makeInput(1, config);
  SharedDsgInfo dsg({});
  FrontendOutput output(1, 0);

  EXPECT_FALSE(frontend.detect(msg));
  EXPECT_NO_THROW(frontend.call(msg, dsg, output, nullptr));
  EXPECT_NO_THROW(frontend.updateGraph(1, *dsg.graph));
  EXPECT_EQ(dsg.graph->numNodes(), 0u);
}

TEST(GvdPlaceExtractor, InvalidLaterInputDoesNotRepublishStalePlaces) {
  TestGvdPlaceExtractor frontend;
  SharedDsgInfo dsg({});
  const NodeSymbol node('p', 0);
  frontend.extractor().graph_.add(0);
  frontend.updateGraph(1, *dsg.graph);

  VolumetricMap::Config config;
  config.truncation_distance = 0.1f;
  const auto msg = makeInput(2, config);
  FrontendOutput output(2, 0);
  frontend.call(msg, dsg, output, nullptr);
  EXPECT_EQ(dsg.graph->getNode(node).attributes().last_update_time_ns, 1u);
}

TEST(GvdPlaceExtractor, InterpolationFailureDoesNotFallBackToOriginalGrid) {
  TestGvdPlaceExtractor frontend;
  DownsampleTsdfInterpolator::Config config;
  config.ratio = 16;  // Leaves only one voxel per side, which the interpolator rejects.
  frontend.tsdf_interpolator_ = std::make_unique<DownsampleTsdfInterpolator>(config);

  EXPECT_FALSE(frontend.detect(makeInput(1)));
  EXPECT_FALSE(frontend.gvd_);
}

TEST(GvdPlaceExtractor, RejectsChangesToInitializedVoxelGrid) {
  TestGvdPlaceExtractor frontend;
  ASSERT_TRUE(frontend.detect(makeInput(1)));
  const auto original = frontend.gvd_;

  VolumetricMap::Config config;
  config.voxel_size *= 2;
  EXPECT_FALSE(frontend.detect(makeInput(2, config)));
  EXPECT_EQ(frontend.gvd_, original);
}

TEST(GvdPlaceExtractor, TemporalWindowUsesLastBlockInputTime) {
  TestGvdPlaceExtractor frontend;
  TemporalWindowChecker window({1.0});
  auto map = std::make_shared<VolumetricMap>(VolumetricMap::Config{});
  map->allocateBlock({0, 0, 0});
  auto msg = makeInput(10'000'000'000);
  msg.setMap(map);

  ASSERT_TRUE(frontend.detect(msg, &window));
  ASSERT_EQ(frontend.gvd_->numBlocks(), 1u);
  ASSERT_TRUE(frontend.detect(makeInput(10'500'000'000), &window));
  EXPECT_EQ(frontend.gvd_->numBlocks(), 1u);
  ASSERT_TRUE(frontend.detect(makeInput(11'500'000'000), &window));
  EXPECT_EQ(frontend.gvd_->numBlocks(), 0u);
}

TEST(GvdPlaceExtractor, SpatialWindowArchivesOutsideBlocks) {
  TestGvdPlaceExtractor frontend;
  SpatialWindowChecker window({2.0});
  auto map = std::make_shared<VolumetricMap>(VolumetricMap::Config{});
  map->allocateBlock({0, 0, 0});
  map->allocateBlock({4, 0, 0});
  auto msg = makeInput(1);
  msg.setMap(map);

  ASSERT_TRUE(frontend.detect(msg, &window));
  EXPECT_TRUE(frontend.gvd_->hasBlock(BlockIndex(0, 0, 0)));
  EXPECT_FALSE(frontend.gvd_->hasBlock(BlockIndex(4, 0, 0)));
}

}  // namespace hydra
