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
#include "hydra_utils/visualizer_types.h"

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

namespace hydra {

using ColorFunction = std::function<NodeColor(const SceneGraphNode&)>;
using EdgeColorFunction =
    std::function<NodeColor(const SceneGraphNode&, const SceneGraphNode&, bool)>;

visualization_msgs::Marker makeDeleteMarker(const std_msgs::Header& header,
                                            size_t id,
                                            const std::string& ns);

visualization_msgs::Marker makeBoundingBoxMarker(
    const std_msgs::Header& header,
    const LayerConfig& config,
    const SceneGraphNode& node,
    const VisualizerConfig& visualizer_config,
    const std::string& ns);

visualization_msgs::Marker makeTextMarker(const std_msgs::Header& header,
                                          const LayerConfig& config,
                                          const SceneGraphNode& node,
                                          const VisualizerConfig& visualizer_config,
                                          const std::string& ns);

visualization_msgs::Marker makeCentroidMarkers(
    const std_msgs::Header& header,
    const LayerConfig& config,
    const SceneGraphLayer& layer,
    const VisualizerConfig& visualizer_config,
    const std::string& ns);

visualization_msgs::Marker makeCentroidMarkers(
    const std_msgs::Header& header,
    const LayerConfig& config,
    const SceneGraphLayer& layer,
    const VisualizerConfig& visualizer_config,
    const std::string& ns,
    const ColormapConfig& colors);

visualization_msgs::Marker makeCentroidMarkers(
    const std_msgs::Header& header,
    const LayerConfig& config,
    const SceneGraphLayer& layer,
    const VisualizerConfig& visualizer_config,
    const std::string& ns,
    const ColorFunction& color_func);

visualization_msgs::MarkerArray makeGraphEdgeMarkers(
    const std_msgs::Header& header,
    const DynamicSceneGraph& scene_graph,
    const std::map<LayerId, LayerConfig>& configs,
    const VisualizerConfig& visualizer_config,
    const std::string& ns);

visualization_msgs::Marker makeMeshEdgesMarker(
    const std_msgs::Header& header,
    const LayerConfig& config,
    const VisualizerConfig& visualizer_config,
    const DynamicSceneGraph& graph,
    const SceneGraphLayer& layer,
    const std::string& ns);

visualization_msgs::Marker makeLayerEdgeMarkers(
    const std_msgs::Header& header,
    const LayerConfig& config,
    const SceneGraphLayer& layer,
    const VisualizerConfig& visualizer_config,
    const NodeColor& color,
    const std::string& ns);

visualization_msgs::Marker makeLayerEdgeMarkers(
    const std_msgs::Header& header,
    const LayerConfig& config,
    const SceneGraphLayer& layer,
    const VisualizerConfig& visualizer_config,
    const std::string& ns,
    const EdgeColorFunction& color_func);

visualization_msgs::Marker makeDynamicCentroidMarkers(
    const std_msgs::Header& header,
    const DynamicLayerConfig& config,
    const DynamicSceneGraphLayer& layer,
    const VisualizerConfig& visualizer_config,
    const NodeColor& color,
    const std::string& ns);

visualization_msgs::Marker makeDynamicCentroidMarkers(
    const std_msgs::Header& header,
    const DynamicLayerConfig& config,
    const DynamicSceneGraphLayer& layer,
    double layer_offset_scale,
    const VisualizerConfig& visualizer_config,
    const std::string& ns,
    const ColorFunction& color_func);

visualization_msgs::MarkerArray makeDynamicGraphEdgeMarkers(
    const std_msgs::Header& header,
    const DynamicSceneGraph& graph,
    const std::map<LayerId, LayerConfig>& configs,
    const std::map<LayerId, DynamicLayerConfig>& dynamic_configs,
    const VisualizerConfig& visualizer_config,
    const std::string& ns_prefix);

visualization_msgs::Marker makeDynamicEdgeMarkers(
    const std_msgs::Header& header,
    const DynamicLayerConfig& config,
    const DynamicSceneGraphLayer& layer,
    const VisualizerConfig& visualizer_config,
    const NodeColor& color,
    const std::string& ns);

visualization_msgs::Marker makeDynamicLabelMarker(
    const std_msgs::Header& header,
    const DynamicLayerConfig& config,
    const DynamicSceneGraphLayer& layer,
    const VisualizerConfig& visualizer_config,
    const std::string& ns);

}  // namespace hydra
