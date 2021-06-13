#pragma once
#include "kimera_scene_graph/common.h"

#include <kimera_dsg/node_attributes.h>
#include <kimera_dsg/scene_graph.h>
#include <kimera_scene_graph/LayerVisualizerConfig.h>
#include <kimera_scene_graph/VisualizerConfig.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <voxblox_ros/mesh_vis.h>

namespace kimera {

using NodeColor = SemanticNodeAttributes::ColorVector;
using LayerConfig = kimera_scene_graph::LayerVisualizerConfig;
using VisualizerConfig = kimera_scene_graph::VisualizerConfig;

visualization_msgs::Marker makeBoundingBoxMarker(
    const LayerConfig& config,
    const SceneGraphNode& node,
    const VisualizerConfig& visualizer_config,
    const std::string& marker_namespace = "bounding_box");

visualization_msgs::Marker makeTextMarker(
    const LayerConfig& config,
    const SceneGraphNode& node,
    const VisualizerConfig& visualizer_config,
    const std::string& marker_namespace = "text_label");

visualization_msgs::Marker makeCentroidMarkers(
    const LayerConfig& config,
    const SceneGraphLayer& layer,
    const VisualizerConfig& visualizer_config,
    std::optional<NodeColor> layer_color = std::nullopt,
    const std::string& marker_namespace = "layer_centroids");

visualization_msgs::MarkerArray makeGraphEdgeMarkers(
    const SceneGraph& scene_graph,
    const std::map<LayerId, LayerConfig>& configs,
    const VisualizerConfig& visualizer_config);

// TODO(nathan) this will get merged with other functions probably
visualization_msgs::Marker makeMeshEdgesMarker(
    const LayerConfig& config,
    const SceneGraphLayer& layer,
    const VisualizerConfig& visualizer_config,
    const std::string& marker_namespace = "mesh_layer_edges");

visualization_msgs::Marker makeLayerEdgeMarkers(
    const LayerConfig& config,
    const SceneGraphLayer& layer,
    const VisualizerConfig& visualizer_config,
    const NodeColor& color);

/**
 * @brief construct an rviz marker for the provided mesh
 * @param config visualization config for layer this mesh belongs to
 * @param mesh Actual semantic mesh
 * @param color_mode How to color the mesh
 */
visualization_msgs::Marker makeMeshMarker(
    const LayerConfig& config,
    const VisualizerConfig& visualizer_config,
    const voxblox::Mesh& mesh,
    voxblox::ColorMode color_mode,
    const std::string& marker_namespace = "mesh");

}  // namespace kimera
