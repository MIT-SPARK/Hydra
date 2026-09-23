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
#include <hydra/active_window/active_window_output.h>
#include <hydra/input/camera.h>
#include <hydra/places/block_traversability_clustering.h>
#include <hydra/places/traversability_projective_integrator.h>

namespace hydra::places {

using Range = BlockTraversabilityClustering::Range;
using spark_dsg::TraversabilityState;

TEST(TraversabilityPlaces, Indexing) {
  // Linear indexing.
  TraversabilityBlock block(1.0f, {1, 2, 3}, 10);
  EXPECT_EQ(block.indexFromLinear(0), Index2D(0, 0));
  EXPECT_EQ(block.indexFromLinear(9), Index2D(9, 0));
  EXPECT_EQ(block.indexFromLinear(10), Index2D(0, 1));
  EXPECT_EQ(block.indexFromLinear(99), Index2D(9, 9));

  // Global indexing.
  EXPECT_EQ(block.globalFromLocalIndex({0, 0}), BlockIndex(10, 20, 3));
  EXPECT_EQ(block.globalFromLocalIndex({3, 4}), BlockIndex(13, 24, 3));
  EXPECT_EQ(block.localFromGlobalIndex(BlockIndex(10, 20, 3)), Index2D(0, 0));
  EXPECT_EQ(block.localFromGlobalIndex(BlockIndex(13, 24, 3)), Index2D(3, 4));
  EXPECT_EQ(block.globalFromLocalIndex({-1, -2}), BlockIndex(9, 18, 3));

  // Layer indexing.
  TraversabilityLayer layer(0.1f, 10);
  EXPECT_EQ(layer.blockIndexFromGlobal(BlockIndex(10, 20, 3)), BlockIndex(1, 2, 3));
  EXPECT_EQ(layer.voxelIndexFromGlobal(BlockIndex(10, 20, 3)), Index2D(0, 0));
  EXPECT_EQ(layer.blockIndexFromGlobal(BlockIndex(13, 24, 3)), BlockIndex(1, 2, 3));
  EXPECT_EQ(layer.voxelIndexFromGlobal(BlockIndex(13, 24, 3)), Index2D(3, 4));
  EXPECT_EQ(layer.blockIndexFromGlobal(BlockIndex(9, 18, 3)), BlockIndex(0, 1, 3));
  EXPECT_EQ(layer.voxelIndexFromGlobal(BlockIndex(9, 18, 3)), Index2D(9, 8));
}

TEST(TraversabilityPlaces, Range) {
  const auto range = Range(0, 0, 9, 9);  // Edges are inclusive.
  EXPECT_EQ(range.x_start, 0);
  EXPECT_EQ(range.y_start, 0);
  EXPECT_EQ(range.x_end, 9);
  EXPECT_EQ(range.y_end, 9);
  EXPECT_EQ(range.width(), 10);
  EXPECT_EQ(range.height(), 10);
  EXPECT_EQ(range.area(), 100);
}

TEST(TraversabilityPlaces, RangeProjection) {
  // invertToSide
  auto range = Range(2, 3, 4, 5);
  EXPECT_EQ(range.invertToSide(0, 10, 3), Range(2, 0, 4, 2));  // bottom
  EXPECT_EQ(range.invertToSide(1, 10, 3), Range(0, 3, 1, 5));  // left
  EXPECT_EQ(range.invertToSide(2, 10, 3), Range(2, 6, 4, 8));  // top
  EXPECT_EQ(range.invertToSide(3, 10, 3), Range(5, 3, 7, 5));  // right

  // projectToNextBlock
  range = Range(2, 3, 4, 5);
  EXPECT_EQ(range.projectToNextBlock(0, 10, 3), Range(2, 7, 4, 9));  // bottom
  EXPECT_EQ(range.projectToNextBlock(1, 10, 3), Range(7, 3, 9, 5));  // left
  EXPECT_EQ(range.projectToNextBlock(2, 10, 3), Range(2, 0, 4, 2));  // top
  EXPECT_EQ(range.projectToNextBlock(3, 10, 3), Range(0, 3, 2, 5));  // right
};

TEST(TraversabilityPlaces, ResetGeometryKeepsSemantics) {
  TraversabilityBlock block(1.0f, {0, 0, 0}, 2);
  for (auto& voxel : block.voxels) {
    voxel.traversability = 0.5f;
    voxel.confidence = 0.7f;
    voxel.height = 1.5f;
    voxel.state = TraversabilityState::TRAVERSABLE;
    voxel.debug_value = 3.0f;
    voxel.semantic.traversable_count = 3;
    voxel.semantic.intraversable_count = 1;
    voxel.semantic.traversability = 0.75f;
    voxel.semantic.confidence = 0.8f;
  }

  resetGeometry(block);
  for (const auto& voxel : block.voxels) {
    EXPECT_EQ(voxel.traversability, 0.0f);
    EXPECT_EQ(voxel.confidence, 0.0f);
    EXPECT_FALSE(voxel.height);
    EXPECT_EQ(voxel.state, TraversabilityState::UNKNOWN);
    EXPECT_EQ(voxel.debug_value, -1.0f);
    EXPECT_EQ(voxel.semantic.traversable_count, 3u);
    EXPECT_EQ(voxel.semantic.intraversable_count, 1u);
    EXPECT_EQ(voxel.semantic.traversability, 0.75f);
    EXPECT_EQ(voxel.semantic.confidence, 0.8f);
  }
}

TEST(TraversabilityPlaces, ProjectiveIntegratorAccumulates) {
  // Camera at the world origin looking along +z (identity body pose and extrinsics).
  Camera::Config camera_config;
  camera_config.min_range = 0.1;
  camera_config.max_range = 10.0;
  camera_config.width = 640;
  camera_config.height = 480;
  camera_config.cx = 320.0f;
  camera_config.cy = 240.0f;
  camera_config.fx = 320.0f;
  camera_config.fy = 320.0f;
  camera_config.extrinsics = ParamSensorExtrinsics::Config();
  auto camera = std::make_shared<Camera>(camera_config, "camera");

  // Voxel (0, 0) is centered at (0.05, 0.05) and sits on a surface at z = 2.
  const float visible_range = Eigen::Vector3f(0.05f, 0.05f, 2.0f).norm();
  auto data = std::make_shared<InputData>(camera);
  data->world_T_body = Eigen::Isometry3d::Identity();
  data->range_image =
      cv::Mat(480, 640, InputData::RangeMatType, cv::Scalar(visible_range));
  data->label_image = cv::Mat(480, 640, InputData::LabelMatType, cv::Scalar(1));
  const ActiveWindowOutput msg(data);

  TraversabilityProjectiveIntegrator::Config config;
  config.interpolation_method =
      config::VirtualConfig<ProjectionInterpolator>(InterpolatorNearest::Config{});
  config.confidence_saturation_count = 4;
  TraversabilityProjectiveIntegrator integrator(config);

  TraversabilityLayer layer(0.1f, 10);
  auto& block = layer.allocateBlock(BlockIndex(0, 0, 0), 10);
  const auto set_heights = [&block]() {
    block.voxel(0, 0).height = 2.0f;  // visible
    block.voxel(1, 0).height = 3.0f;  // behind the observed surface
  };

  set_heights();
  integrator.apply(layer, msg);

  // Emulate the estimator recomputing the block between updates.
  resetGeometry(block);
  set_heights();
  integrator.apply(layer, msg);

  const auto& visible = block.voxel(0, 0).semantic;
  EXPECT_EQ(visible.traversable_count, 2u);
  EXPECT_EQ(visible.intraversable_count, 0u);
  EXPECT_FLOAT_EQ(visible.traversability, 1.0f);
  EXPECT_FLOAT_EQ(visible.confidence, 0.5f);

  // Occluded and height-less cells get no evidence.
  EXPECT_EQ(block.voxel(1, 0).semantic.total(), 0u);
  EXPECT_EQ(block.voxel(0, 1).semantic.total(), 0u);
  EXPECT_LT(block.voxel(0, 1).semantic.traversability, 0.0f);
}

}  // namespace hydra::places
