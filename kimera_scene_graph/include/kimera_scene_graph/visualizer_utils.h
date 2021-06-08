#pragma once
#include "kimera_scene_graph/common.h"

#include <kimera_dsg/node_attributes.h>
#include <kimera_dsg/scene_graph_layer.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <voxblox_ros/mesh_vis.h>

namespace kimera {

using NodeColor = SemanticNodeAttributes::ColorVector;

struct LayerConfig {
  double z_offset = 0.0;
  double marker_scale = 0.1;
  double marker_alpha = 1.0;
  bool use_sphere_marker = false;
  double edge_scale = 0.03;
  double edge_alpha = 1.0;
  bool use_label = false;
  double text_height = 1.0;
  double text_scale = 0.5;
  size_t intralayer_edge_insertion_skip = 0;
  size_t interlayer_edge_insertion_skip = 0;
  bool interlayer_edge_use_color = false;
  double bounding_box_alpha = 0.5;
  bool use_edge_source = true;
};

std_msgs::ColorRGBA makeColorMsg(const NodeColor& color, double alpha = 1.0);

void fillPoseWithIdentity(geometry_msgs::Pose& pose);

visualization_msgs::Marker makeBoundingBoxMarker(
    const LayerConfig& config,
    const SceneGraphNode& node,
    const std::string& marker_namespace = "bounding_box");

visualization_msgs::Marker makeTextMarker(
    const LayerConfig& config,
    const SceneGraphNode& node,
    const std::string& marker_namespace = "text_label");

visualization_msgs::Marker makeCentroidMarkers(
    const LayerConfig& config,
    const SceneGraphLayer& layer,
    std::optional<NodeColor> layer_color = std::nullopt,
    const std::string& marker_namespace = "layer_centroids");

visualization_msgs::Marker makeMeshEdgesMarker(const LayerConfig& config,
                                               const SceneGraphLayer& layer,
                                               double secondary_offset,
                                               double mesh_offset);

visualization_msgs::Marker makeLayerEdgeMarkers(const LayerConfig& config,
                                                const SceneGraphLayer& layer,
                                                const NodeColor& color);

/**
 * @brief construct an rviz marker for the provided mesh
 * @param config visualization config for layer this mesh belongs to
 * @param mesh Actual semantic mesh
 * @param color_mode How to color the mesh
 */
visualization_msgs::Marker makeMeshMarker(
    const LayerConfig& config,
    const voxblox::Mesh& mesh,
    voxblox::ColorMode color_mode,
    const std::string& marker_namespace = "mesh");

}  // namespace kimera
