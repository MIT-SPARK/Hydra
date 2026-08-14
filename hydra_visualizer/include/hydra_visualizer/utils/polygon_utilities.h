
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

#include <optional>

#include <visualization_msgs/msg/marker.hpp>

#include <Eigen/Dense>

namespace hydra {

double getMeanChildHeight(const spark_dsg::DynamicSceneGraph& graph,
                          const spark_dsg::SceneGraphNode& parent);

double getMeanNeighborHeight(const spark_dsg::SceneGraphLayer& graph,
                             const spark_dsg::SceneGraphNode& parent,
                             double neighborhood = 10.0);

Eigen::MatrixXd getCirclePolygon(const spark_dsg::SceneGraphNode& node,
                                 double radius,
                                 size_t num_samples);

Eigen::MatrixXd getChildrenConvexHull(const spark_dsg::DynamicSceneGraph& graph,
                                      const spark_dsg::SceneGraphNode& parent);

void makeFilledPolygon(const Eigen::MatrixXd& points,
                       const std_msgs::msg::ColorRGBA& color,
                       visualization_msgs::msg::Marker& marker,
                       std::optional<double> height = std::nullopt);

void makePolygonBoundary(const Eigen::MatrixXd& points,
                         const std_msgs::msg::ColorRGBA& color,
                         visualization_msgs::msg::Marker& edges,
                         std::optional<double> height = std::nullopt,
                         visualization_msgs::msg::Marker* corners = nullptr);

}  // namespace hydra
