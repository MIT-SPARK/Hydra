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
#include <spark_dsg/bounding_box.h>
#include <spark_dsg/color.h>
#include <spark_dsg/dynamic_scene_graph.h>

#include <kimera_pgmo_msgs/msg/mesh.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "hydra_visualizer/adapters/mesh_color.h"
#include "hydra_visualizer/layer_info.h"

namespace hydra::visualizer {

using spark_dsg::DynamicSceneGraph;
using spark_dsg::EdgeContainer;
using spark_dsg::SceneGraphLayer;
using MarkerMsg = visualization_msgs::msg::Marker;
using MarkerArrayMsg = visualization_msgs::msg::MarkerArray;

void drawBoundingBox(const spark_dsg::BoundingBox& bbox,
                     const std_msgs::msg::ColorRGBA& color,
                     MarkerMsg& marker);

MarkerArrayMsg makeLayerBoundingBoxes(const std_msgs::msg::Header& header,
                                      const LayerInfo& info,
                                      const SceneGraphLayer& layer,
                                      const std::string& ns);

MarkerMsg makeLayerEllipseBoundaries(const std_msgs::msg::Header& header,
                                     const LayerInfo& info,
                                     const SceneGraphLayer& layer,
                                     const std::string& ns);

MarkerMsg makeLayerPolygonEdges(const std_msgs::msg::Header& header,
                                const LayerInfo& info,
                                const SceneGraphLayer& layer,
                                const std::string& ns);

MarkerMsg makeLayerPolygonBoundaries(const std_msgs::msg::Header& header,
                                     const LayerInfo& info,
                                     const SceneGraphLayer& layer,
                                     const std::string& ns);

MarkerArrayMsg makeEllipsoidMarkers(const std_msgs::msg::Header& header,
                                    const LayerInfo& info,
                                    const SceneGraphLayer& layer,
                                    const std::string& ns);

MarkerArrayMsg makeLayerNodeTextMarkers(const std_msgs::msg::Header& header,
                                        const LayerInfo& info,
                                        const SceneGraphLayer& layer,
                                        const std::string& ns);

MarkerMsg makeLayerNodeMarkers(const std_msgs::msg::Header& header,
                               const LayerInfo& info,
                               const SceneGraphLayer& layer,
                               const std::string& ns);

MarkerMsg makeLayerEdgeMarkers(const std_msgs::msg::Header& header,
                               const LayerInfo& config,
                               const SceneGraphLayer& layer,
                               const std::string& ns);

// TODO(nathan) add ability to draw mesh points for given 2D place or object

MarkerMsg makeLayerTextMarker(const std_msgs::msg::Header& header,
                              const LayerInfo& info,
                              const SceneGraphLayer& layer,
                              const std::string& ns);

kimera_pgmo_msgs::msg::Mesh makeMeshMsg(const std_msgs::msg::Header& header,
                                        const spark_dsg::Mesh& mesh,
                                        const std::string& ns,
                                        MeshColoring::Ptr coloring = nullptr);

}  // namespace hydra::visualizer
