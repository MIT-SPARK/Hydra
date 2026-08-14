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
#include <glog/stl_logging.h>
#include <gtest/gtest.h>
#include <hydra_visualizer/utils/ear_clipping.h>

namespace hydra {

TEST(EarClipping, TriangleIter) {
  std::vector<Vertex> vertices;
  for (size_t i = 0; i < 5; ++i) {
    vertices.push_back({Eigen::Vector2d::Zero(), i});
  }

  Polygon polygon(vertices);
  std::vector<std::array<size_t, 3>> expected{
      {4, 0, 1}, {0, 1, 2}, {1, 2, 3}, {2, 3, 4}, {3, 4, 0}};
  auto expected_iter = expected.begin();
  for (const auto& view : polygon) {
    std::stringstream ss;
    ss << "View: [" << expected_iter->at(0) << ", " << expected_iter->at(1) << ", "
       << expected_iter->at(2) << "]";
    SCOPED_TRACE(ss.str());
    ASSERT_TRUE(view);
    EXPECT_EQ(view.v0->id, expected_iter->at(0));
    EXPECT_EQ(view.v1->id, expected_iter->at(1));
    EXPECT_EQ(view.v2->id, expected_iter->at(2));
    ++expected_iter;
  }
}

TEST(EarClipping, IsConvex) {
  Vertex v0{Eigen::Vector2d(0.0, 0.0), 0u};
  Vertex v1{Eigen::Vector2d(1.0, 0.0), 1u};
  Vertex v2{Eigen::Vector2d(2.0, 1.0), 2u};
  TriangleView view{&v0, &v1, &v2};
  EXPECT_TRUE(view.isConvex(true));
  EXPECT_FALSE(view.isConvex(false));
}

TEST(EarClipping, IsConvexRotated) {
  Vertex v0{Eigen::Vector2d(0.0, 0.0), 0u};
  Vertex v1{Eigen::Vector2d(0.0, 1.0), 1u};
  Vertex v2{Eigen::Vector2d(-1.0, 2.0), 2u};
  TriangleView view{&v0, &v1, &v2};
  EXPECT_TRUE(view.isConvex(true));
  EXPECT_FALSE(view.isConvex(false));
}

TEST(EarClipping, IsReflex) {
  Vertex v0{Eigen::Vector2d(0.0, 0.0), 0u};
  Vertex v1{Eigen::Vector2d(1.0, 0.0), 1u};
  Vertex v2{Eigen::Vector2d(2.0, -1.0), 2u};
  TriangleView view{&v0, &v1, &v2};
  EXPECT_FALSE(view.isConvex(true));
  EXPECT_TRUE(view.isConvex(false));
}

TEST(EarClipping, IsReflexRotated) {
  Vertex v0{Eigen::Vector2d(0.0, 0.0), 0u};
  Vertex v1{Eigen::Vector2d(0.0, 1.0), 1u};
  Vertex v2{Eigen::Vector2d(1.0, 2.0), 2u};
  TriangleView view{&v0, &v1, &v2};
  EXPECT_FALSE(view.isConvex(true));
  EXPECT_TRUE(view.isConvex(false));
}

TEST(EarClipping, IsInside) {
  Vertex v0{Eigen::Vector2d(-1.0, 0.0), 0u};
  Vertex v1{Eigen::Vector2d(0.0, 1.0), 1u};
  Vertex v2{Eigen::Vector2d(1.0, 0.0), 2u};
  TriangleView view{&v0, &v1, &v2};
  EXPECT_TRUE(view.isInside(Eigen::Vector2d(0.0, 0.5)));
  EXPECT_FALSE(view.isInside(Eigen::Vector2d(0.5, -0.5)));
  EXPECT_FALSE(view.isInside(Eigen::Vector2d(1.5, 0.5)));
  EXPECT_FALSE(view.isInside(Eigen::Vector2d(-1.5, 0.5)));
  EXPECT_FALSE(view.isInside(Eigen::Vector2d(0.0, 1.1)));
}

TEST(EarClipping, TriangulateLine) {
  std::vector<Vertex> vertices{{Eigen::Vector2d(-1.0, 0.0), 0u},
                               {Eigen::Vector2d(0.0, 1.0), 1u}};
  Polygon polygon(vertices);
  auto faces = polygon.triangulate();
  EXPECT_TRUE(faces.empty());
}

TEST(EarClipping, TriangulateFace) {
  std::vector<Vertex> vertices{{Eigen::Vector2d(-1.0, 0.0), 0u},
                               {Eigen::Vector2d(0.0, 1.0), 1u},
                               {Eigen::Vector2d(1.0, 0.0), 2u}};

  Polygon polygon(vertices);
  auto faces = polygon.triangulate();
  std::vector<std::array<size_t, 3>> expected{{0, 1, 2}};
  EXPECT_EQ(faces, expected);
}

TEST(EarClipping, TriangulateSquareCW) {
  std::vector<Vertex> vertices{{Eigen::Vector2d(0.0, 0.0), 0u},
                               {Eigen::Vector2d(0.0, 1.0), 1u},
                               {Eigen::Vector2d(1.0, 1.0), 2u},
                               {Eigen::Vector2d(1.0, 0.0), 3u}};

  Polygon polygon(vertices, false);
  auto faces = polygon.triangulate();
  std::vector<std::array<size_t, 3>> expected{{3, 0, 1}, {3, 1, 2}};
  EXPECT_EQ(faces, expected);
}

TEST(EarClipping, TriangulateSquareCCW) {
  std::vector<Vertex> vertices{{Eigen::Vector2d(0.0, 0.0), 0u},
                               {Eigen::Vector2d(1.0, 0.0), 1u},
                               {Eigen::Vector2d(1.0, 1.0), 2u},
                               {Eigen::Vector2d(0.0, 1.0), 3u}};

  Polygon polygon(vertices);
  auto faces = polygon.triangulate();
  std::vector<std::array<size_t, 3>> expected{{3, 0, 1}, {3, 1, 2}};
  EXPECT_EQ(faces, expected);
}

TEST(EarClipping, TriangulateHexagon) {
  std::vector<Vertex> vertices{{Eigen::Vector2d(0.0, 0.0), 0u},
                               {Eigen::Vector2d(1.0, 1.0), 1u},
                               {Eigen::Vector2d(1.0, 2.0), 2u},
                               {Eigen::Vector2d(0.0, 3.0), 3u},
                               {Eigen::Vector2d(-1.0, 2.0), 4u},
                               {Eigen::Vector2d(-1.0, 1.0), 5u}};

  Polygon polygon(vertices);
  auto faces = polygon.triangulate();
  std::vector<std::array<size_t, 3>> expected{
      {5, 0, 1}, {5, 1, 2}, {2, 3, 4}, {5, 2, 4}};
  EXPECT_EQ(faces, expected);
}

TEST(EarClipping, TriangulateBadCCWDetection) {
  std::vector<Vertex> vertices{{Eigen::Vector2d(-15.45, 22.05), 0u},
                               {Eigen::Vector2d(-16.45, 22.05), 1u},
                               {Eigen::Vector2d(-17.45, 22.45), 2u},
                               {Eigen::Vector2d(-18.55, 23.15), 3u},
                               {Eigen::Vector2d(-19.15, 24.25), 4u},
                               {Eigen::Vector2d(-18.85, 25.15), 5u},
                               {Eigen::Vector2d(-17.65, 26.45), 6u},
                               {Eigen::Vector2d(-16.85, 27.55), 7u},
                               {Eigen::Vector2d(-16.05, 27.25), 8u},
                               {Eigen::Vector2d(-15.45, 26.25), 9u},
                               {Eigen::Vector2d(-15.45, 26.25), 10u},
                               {Eigen::Vector2d(-15.35, 25.65), 11u},
                               {Eigen::Vector2d(-15.35, 24.55), 12u},
                               {Eigen::Vector2d(-15.45, 23.15), 13u},
                               {Eigen::Vector2d(-15.55, 22.45), 14u}};

  Polygon polygon(vertices, false);
  auto faces = polygon.triangulate(true);
  std::vector<std::array<size_t, 3>> expected{{14, 0, 1},
                                              {14, 1, 2},
                                              {14, 2, 3},
                                              {14, 3, 4},
                                              {14, 4, 5},
                                              {14, 5, 6},
                                              {14, 6, 7},
                                              {14, 7, 8},
                                              {14, 8, 10},
                                              {14, 10, 11},
                                              {14, 11, 12},
                                              {14, 12, 13}};
  EXPECT_EQ(faces, expected);
}

TEST(EarClipping, TriangulateMissingFaces) {
  std::vector<Vertex> vertices{{Eigen::Vector2d(-18.15, 26.85), 0u},
                               {Eigen::Vector2d(-18.95, 25.95), 1u},
                               {Eigen::Vector2d(-19.05, 24.15), 2u},
                               {Eigen::Vector2d(-18.65, 23.25), 3u},
                               {Eigen::Vector2d(-17.45, 22.45), 4u},
                               {Eigen::Vector2d(-16.75, 21.95), 5u},
                               {Eigen::Vector2d(-15.45, 22.05), 6u},
                               {Eigen::Vector2d(-15.45, 22.05), 7u},
                               {Eigen::Vector2d(-15.35, 23.25), 8u},
                               {Eigen::Vector2d(-14.95, 24.45), 9u},
                               {Eigen::Vector2d(-15.45, 26.65), 10u},
                               {Eigen::Vector2d(-16.05, 27.25), 11u},
                               {Eigen::Vector2d(-16.05, 27.25), 12u},
                               {Eigen::Vector2d(-17.15, 27.55), 13u},
                               {Eigen::Vector2d(-17.65, 26.75), 14u}};

  Polygon polygon(vertices);
  auto faces = polygon.triangulate(true);
  std::vector<std::array<size_t, 3>> expected{{14, 0, 1},
                                              {14, 1, 2},
                                              {14, 2, 3},
                                              {14, 3, 4},
                                              {14, 4, 5},
                                              {14, 5, 7},
                                              {14, 7, 8},
                                              {14, 8, 9},
                                              {14, 9, 10},
                                              {14, 10, 12},
                                              {14, 12, 13}};
  EXPECT_EQ(faces, expected);
}

}  // namespace hydra
