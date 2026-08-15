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
#pragma once
#include <spark_dsg/dynamic_scene_graph.h>

#include <list>

namespace hydra {

struct Vertex {
  Eigen::Vector2d pos;
  size_t id;
};

struct TriangleView {
  const Vertex* v0 = nullptr;
  const Vertex* v1 = nullptr;
  const Vertex* v2 = nullptr;

  bool valid() const;

  inline operator bool() const { return valid(); }

  double interiorAngle(bool ccw = false) const;

  bool isConvex(bool ccw = false) const;

  bool isInside(const Eigen::Vector2d& p) const;

  bool adjacent(const TriangleView& other) const;

  std::array<size_t, 3> face() const;
};

class Polygon;

class TriangleIter {
 public:
  using iterator_category = std::forward_iterator_tag;
  using difference_type = std::ptrdiff_t;
  using value_type = TriangleView;
  using pointer = const TriangleView*;
  using reference = const TriangleView&;

 public:
  TriangleIter(const Polygon* polygon, std::list<size_t>::const_iterator iter);

  reference operator*() const;
  pointer operator->() const;

  TriangleIter& operator++();
  TriangleIter operator++(int);

  TriangleIter next() const;
  TriangleIter prev() const;

  friend bool operator==(const TriangleIter& lhs, const TriangleIter& rhs);
  friend bool operator!=(const TriangleIter& lhs, const TriangleIter& rhs);
  friend Polygon;

 private:
  void setView();

 private:
  TriangleView view_;
  const Polygon* polygon_;
  std::list<size_t>::const_iterator iter_;
};

class Polygon {
 public:
  explicit Polygon(const std::vector<Vertex>& vertices, bool is_ccw = true);

  static Polygon fromSceneGraph(const spark_dsg::DynamicSceneGraph& graph,
                                const std::vector<spark_dsg::NodeId>& vertex_nodes);

  static Polygon fromPoints(const Eigen::MatrixXd& points);

  TriangleIter begin() const;

  TriangleIter end() const;

  const std::list<size_t>& active() const;

  const Vertex* vertex(size_t idx) const;

  size_t size() const;

  bool isEar(const TriangleView& triangle) const;

  std::vector<std::array<size_t, 3>> triangulate(bool use_first_ear = false);

 private:
  void filter();

  TriangleIter erase(TriangleIter iter);

  TriangleIter minAngleEar(const std::vector<bool>& ears) const;

  TriangleIter getFirstEar(const std::vector<bool>& ears) const;

 private:
  bool is_ccw_;
  std::list<size_t> active_;
  std::vector<Vertex> vertices_;
};

}  // namespace hydra
