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
