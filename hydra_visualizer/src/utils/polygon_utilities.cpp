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
#include "hydra_visualizer/utils/polygon_utilities.h"

#include <glog/logging.h>
#include <spark_dsg/bounding_box_extraction.h>
#include <spark_dsg/graph_utilities.h>
#include <spark_dsg/node_attributes.h>

#include <tf2_eigen/tf2_eigen.hpp>

#include "hydra_visualizer/utils/ear_clipping.h"

namespace hydra {

using spark_dsg::DynamicSceneGraph;
using spark_dsg::NodeId;
using spark_dsg::SceneGraphLayer;
using spark_dsg::SceneGraphNode;

double getMeanChildHeight(const DynamicSceneGraph& graph,
                          const SceneGraphNode& parent) {
  double total = 0.0;
  for (const auto& child : parent.children()) {
    total += graph.getNode(child).attributes().position.z();
  }

  return total / static_cast<double>(parent.children().size());
}

double getMeanNeighborHeight(const SceneGraphLayer& graph,
                             const SceneGraphNode& node,
                             double neighborhood_size) {
  double total = 0.0;
  size_t num_found = 0;

  std::deque<NodeId> frontier{node.id};
  std::unordered_set<NodeId> visited{node.id};
  spark_dsg::graph_utilities::breadthFirstSearch(
      graph,
      frontier,
      visited,
      [&](const auto& other) {
        return (node.attributes().position - other.attributes().position).norm() <
               neighborhood_size;
      },
      [](const auto&) { return true; },
      [&](const SceneGraphLayer&, NodeId other) {
        total += graph.getNode(other).attributes().position.z();
        ++num_found;
      });

  return num_found ? total / static_cast<double>(num_found) : 0.0;
}

Eigen::MatrixXd getCirclePolygon(const SceneGraphNode& node,
                                 double radius,
                                 size_t num_samples) {
  const auto& pos = node.attributes().position;
  Eigen::MatrixXd footprint(3, num_samples);
  for (size_t i = 0; i < num_samples; ++i) {
    const auto theta =
        2 * M_PI * (static_cast<double>(i) / static_cast<double>(num_samples));
    Eigen::Vector3d delta(radius * std::cos(theta), radius * std::sin(theta), 0.0);
    footprint.block<3, 1>(0, i) = pos + delta;
  }

  return footprint;
}

struct NodeAdapter : public ::spark_dsg::bounding_box::PointAdaptor {
  NodeAdapter(const DynamicSceneGraph* graph, const std::vector<NodeId>& nodes)
      : graph(graph), nodes(nodes) {}

  size_t size() const override { return nodes.size(); }

  Eigen::Vector3f get(size_t index) const override {
    if (!graph) {
      throw std::runtime_error("invalid graph!");
    }

    return graph->getNode(nodes.at(index)).attributes().position.cast<float>();
  }

  const DynamicSceneGraph* graph;
  const std::vector<NodeId>& nodes;
};

Eigen::MatrixXd getChildrenConvexHull(const DynamicSceneGraph& graph,
                                      const SceneGraphNode& parent) {
  std::vector<NodeId> children(parent.children().begin(), parent.children().end());
  const NodeAdapter adapter(&graph, children);
  std::list<size_t> hull_indices = ::spark_dsg::bounding_box::get2dConvexHull(adapter);

  Eigen::MatrixXd hull_points(3, hull_indices.size());
  size_t i = 0;
  for (const auto idx : hull_indices) {
    hull_points.col(i) = graph.getNode(children.at(idx)).attributes().position;
    ++i;
  }

  return hull_points;
}

void makeFilledPolygon(const Eigen::MatrixXd& points,
                       const std_msgs::msg::ColorRGBA& color,
                       visualization_msgs::msg::Marker& marker,
                       std::optional<double> height) {
  if (points.cols() <= 1 || points.rows() != 3) {
    LOG(ERROR) << "Invalid point dimensions: [" << points.rows() << ", "
               << points.cols() << "]";
    return;
  }

  auto polygon = Polygon::fromPoints(points);
  const auto faces = polygon.triangulate();

  for (const auto& face : faces) {
    for (const auto idx : face) {
      auto& point = marker.points.emplace_back();
      const auto p = points.block<3, 1>(0, idx);
      point.x = p.x();
      point.y = p.y();
      point.z = height.value_or(p.z());
      marker.colors.push_back(color);
    }
  }
}

void makePolygonBoundary(const Eigen::MatrixXd& points,
                         const std_msgs::msg::ColorRGBA& color,
                         visualization_msgs::msg::Marker& edges,
                         std::optional<double> height,
                         visualization_msgs::msg::Marker* corners) {
  if (points.cols() <= 1 || points.rows() != 3) {
    LOG(ERROR) << "Invalid point dimensions: [" << points.rows() << ", "
               << points.cols() << "]";
    return;
  }

  geometry_msgs::msg::Point prev;
  const Eigen::Vector3d prev_pos = points.block<3, 1>(0, points.cols() - 1);
  tf2::convert(prev_pos, prev);
  prev.z = height.value_or(prev.z);

  for (int i = 0; i < points.cols(); ++i) {
    if (corners) {
      corners->points.push_back(prev);
      corners->colors.push_back(color);
    }

    edges.points.push_back(prev);
    edges.colors.push_back(color);

    auto& point = edges.points.emplace_back();

    edges.colors.push_back(color);

    const Eigen::Vector3d pos = points.block<3, 1>(0, i);
    tf2::convert(pos, point);
    point.z = height.value_or(point.z);
    prev = point;
  }
}

}  // namespace hydra
