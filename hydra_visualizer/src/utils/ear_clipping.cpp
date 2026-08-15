
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
#include "hydra_visualizer/utils/ear_clipping.h"

#include <glog/logging.h>

#include <numeric>

namespace hydra {

using spark_dsg::DynamicSceneGraph;
using spark_dsg::NodeId;

std::ostream& operator<<(std::ostream& out, const TriangleView& v) {
  out << "(";
  if (v.v0) {
    out << v.v0->id;
  } else {
    out << "?";
  }
  out << ", ";

  if (v.v1) {
    out << v.v1->id;
  } else {
    out << "?";
  }
  out << ", ";

  if (v.v2) {
    out << v.v2->id;
  } else {
    out << "?";
  }
  out << ")";

  return out;
}

bool TriangleView::valid() const {
  return v0 != nullptr && v1 != nullptr && v2 != nullptr;
}

double TriangleView::interiorAngle(bool ccw) const {
  if (!valid()) {
    return false;
  }

  const auto n1 = (v1->pos - v0->pos).normalized();
  Eigen::Vector2d n_perp;
  if (ccw) {
    n_perp << -n1.y(), n1.x();
  } else {
    n_perp << n1.y(), -n1.x();
  }

  const auto n2 = (v2->pos - v1->pos).normalized();
  return n_perp.dot(n2);
}

bool TriangleView::isConvex(bool ccw) const { return interiorAngle(ccw) > 0.0; }

double det(const Eigen::Vector2d& u, const Eigen::Vector2d& v) {
  return u.x() * v.y() - u.y() * v.x();
}

bool TriangleView::isInside(const Eigen::Vector2d& p) const {
  if (!valid()) {
    return false;
  }

  // from
  // https://mathworld.wolfram.com/TriangleInterior.html#:~:text=The%20simplest%20way%20to%20determine,it%20lies%20outside%20the%20triangle.
  const auto s0 = v1->pos;
  const auto s1 = v0->pos - v1->pos;
  const auto s2 = v2->pos - v1->pos;
  const auto a = (det(p, s2) - det(s0, s2)) / det(s1, s2);
  const auto b = -(det(p, s1) - det(s0, s1)) / det(s1, s2);
  return a > 0.0 && a < 1.0 && b > 0.0 && b < 1.0;
}

bool TriangleView::adjacent(const TriangleView& other) const {
  if (!valid() || !other) {
    return false;
  }

  return v1->id == other.v0->id || v1->id == other.v1->id || v1->id == other.v2->id;
}

std::array<size_t, 3> TriangleView::face() const {
  if (!valid()) {
    return {0, 0, 0};
  } else {
    return {v0->id, v1->id, v2->id};
  }
}

template <typename T>
typename T::const_iterator getPrevIter(const T& vec, typename T::const_iterator iter) {
  if (vec.empty()) {
    return vec.begin();
  }

  const auto last_valid = std::prev(vec.end());
  return iter == vec.begin() ? last_valid : std::prev(iter);
}

template <typename T>
typename T::const_iterator getNextIter(const T& vec, typename T::const_iterator iter) {
  if (vec.empty()) {
    return vec.begin();
  }

  const auto niter = std::next(iter);
  return niter == vec.end() ? vec.begin() : niter;
}

TriangleIter::TriangleIter(const Polygon* polygon,
                           std::list<size_t>::const_iterator iter)
    : polygon_(polygon), iter_(iter) {
  setView();
}

TriangleIter::reference TriangleIter::operator*() const { return view_; }

TriangleIter::pointer TriangleIter::operator->() const { return &view_; }

TriangleIter& TriangleIter::operator++() {
  ++iter_;
  setView();
  return *this;
}

TriangleIter TriangleIter::operator++(int) {
  auto tmp = *this;
  ++(*this);
  return tmp;
}

TriangleIter TriangleIter::next() const {
  return TriangleIter(polygon_, getNextIter(polygon_->active(), iter_));
}

TriangleIter TriangleIter::prev() const {
  return TriangleIter(polygon_, getPrevIter(polygon_->active(), iter_));
}

bool operator==(const TriangleIter& lhs, const TriangleIter& rhs) {
  return lhs.iter_ == rhs.iter_;
}

bool operator!=(const TriangleIter& lhs, const TriangleIter& rhs) {
  return lhs.iter_ != rhs.iter_;
}

void TriangleIter::setView() {
  const auto& active = polygon_->active();
  if (iter_ == active.end()) {
    view_ = TriangleView{};
    return;
  }

  const auto piter = getPrevIter(active, iter_);
  const auto niter = getNextIter(active, iter_);
  view_.v0 = polygon_->vertex(*piter);
  view_.v1 = polygon_->vertex(*iter_);
  view_.v2 = polygon_->vertex(*niter);
}

Polygon::Polygon(const std::vector<Vertex>& vertices, bool is_ccw)
    : is_ccw_(is_ccw), active_(vertices.size()), vertices_(vertices) {
  std::iota(active_.begin(), active_.end(), 0);
  if (vertices_.size() <= 1) {
    return;
  }

  filter();
  VLOG(10) << "counter clockwise: " << std::boolalpha << is_ccw_;
}

void Polygon::filter() {
  auto iter = active_.begin();
  while (iter != active_.end()) {
    auto next = getNextIter(active_, iter);
    const auto norm = (vertices_.at(*iter).pos - vertices_.at(*next).pos).norm();
    if (norm < 1.0e-6) {
      iter = active_.erase(iter);
    } else {
      ++iter;
    }
  }
}

Polygon Polygon::fromSceneGraph(const DynamicSceneGraph& graph,
                                const std::vector<NodeId>& vertex_nodes) {
  size_t id = 0;
  std::vector<Vertex> vertices;
  for (const auto node : vertex_nodes) {
    const auto pos = graph.getPosition(node);
    vertices.push_back({pos.head<2>(), id});
    ++id;
  }

  return Polygon(vertices);
}

Polygon Polygon::fromPoints(const Eigen::MatrixXd& points) {
  std::vector<Vertex> vertices;
  for (int i = 0; i < points.cols(); ++i) {
    vertices.push_back({points.col(i).head<2>(), static_cast<size_t>(i)});
  }

  return Polygon(vertices);
}

TriangleIter Polygon::erase(TriangleIter iter) {
  auto niter = active_.erase(iter.iter_);
  if (niter == active_.end()) {
    niter = active_.begin();
  }

  return TriangleIter(this, niter);
}

TriangleIter Polygon::begin() const { return TriangleIter(this, active_.begin()); }

TriangleIter Polygon::end() const { return TriangleIter(this, active_.end()); }

const std::list<size_t>& Polygon::active() const { return active_; }

const Vertex* Polygon::vertex(size_t idx) const {
  if (idx >= vertices_.size()) {
    return nullptr;
  }

  return &vertices_[idx];
}

size_t Polygon::size() const { return active_.size(); }

bool Polygon::isEar(const TriangleView& triangle) const {
  if (!triangle.isConvex(is_ccw_)) {
    return false;
  }

  for (const auto& other : *this) {
    if (other.adjacent(triangle)) {
      continue;
    }

    if (other.isConvex(is_ccw_)) {
      continue;
    }

    if (triangle.isInside(other.v1->pos)) {
      return false;
    }
  }

  return true;
}

TriangleIter Polygon::minAngleEar(const std::vector<bool>& ears) const {
  auto iter = begin();
  auto max_ear = end();
  double max_cosine = 0.0;
  VLOG(10) << "Finding minimum ear:";
  while (iter != end()) {
    const auto is_ear = ears[iter->v1->id];
    const auto curr_cosine = iter->interiorAngle(is_ccw_);
    VLOG(10) << std::boolalpha << *iter << ": ear=" << is_ear
             << ", angle=" << curr_cosine;
    if (!is_ear) {
      ++iter;
      continue;
    }

    if (curr_cosine > max_cosine) {
      max_ear = iter;
      max_cosine = curr_cosine;
    }

    ++iter;
  }

  VLOG(10) << "Best ear: " << *max_ear << " (angle: " << max_cosine << ")";
  return max_ear;
}

TriangleIter Polygon::getFirstEar(const std::vector<bool>& ears) const {
  auto iter = begin();
  while (iter != end()) {
    if (ears[iter->v1->id]) {
      break;
    }

    ++iter;
  }

  return iter;
}

std::vector<std::array<size_t, 3>> Polygon::triangulate(bool use_first_ear) {
  if (vertices_.size() <= 2) {
    return {};
  }

  if (vertices_.size() == 3) {
    return {{0, 1, 2}};
  }

  std::vector<bool> ears(vertices_.size(), false);
  for (const auto& triangle : *this) {
    if (isEar(triangle)) {
      ears[triangle.v1->id] = true;
    }
  }

  auto next_ear = use_first_ear ? getFirstEar(ears) : minAngleEar(ears);
  std::vector<std::array<size_t, 3>> faces;
  while (size() >= 3) {
    if (next_ear == end()) {
      return faces;
    }

    faces.push_back(next_ear->face());
    ears[next_ear->v1->id] = false;
    const auto next = erase(next_ear);
    if (size() == 3) {
      faces.push_back(begin()->face());
      break;
    }

    const auto prev = next.prev();
    ears[prev->v1->id] = isEar(*prev);
    ears[next->v1->id] = isEar(*next);
    next_ear = use_first_ear ? getFirstEar(ears) : minAngleEar(ears);
  }

  return faces;
}

}  // namespace hydra
