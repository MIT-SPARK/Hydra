#include "kimera_scene_graph/visualizer_utils.h"
#include "kimera_scene_graph/voxblox_conversions.h"

#include <kimera_dsg/scene_graph_layer.h>
#include <tf2_eigen/tf2_eigen.h>

namespace kimera {

using visualization_msgs::Marker;
using Node = SceneGraphLayer::Node;

// TODO(nathan) replace everything with tf2 conversions

void fillPoseWithIdentity(geometry_msgs::Pose& pose) {
  pose.position.x = 0.0;
  pose.position.y = 0.0;
  pose.position.z = 0.0;
  pose.orientation.x = 0.0;
  pose.orientation.y = 0.0;
  pose.orientation.z = 0.0;
  pose.orientation.w = 1.0;
}

std_msgs::ColorRGBA makeColorMsg(const NodeColor& color, double alpha) {
  std_msgs::ColorRGBA msg;
  msg.r = static_cast<double>(color(0)) / 255.0;
  msg.g = static_cast<double>(color(1)) / 255.0;
  msg.b = static_cast<double>(color(2)) / 255.0;
  msg.a = alpha;
  return msg;
}

Marker makeBoundingBoxMarker(const LayerConfig& config,
                             const Node& node,
                             const std::string& marker_namespace) {
  // TODO(nathan) think about direct cast to object attributes
  Marker marker;
  marker.type = Marker::CUBE;
  marker.action = Marker::ADD;

  marker.id = node.id;
  marker.ns = marker_namespace;

  BoundingBox bounding_box =
      node.attributes<ObjectNodeAttributes>().bounding_box;

  switch (bounding_box.type) {
    case BoundingBox::Type::OBB:
      marker.pose.position.x = bounding_box.world_P_center(0);
      marker.pose.position.y = bounding_box.world_P_center(1);
      marker.pose.position.z = bounding_box.world_P_center(2) + config.z_offset;
      marker.pose.orientation.x = bounding_box.world_R_center.x();
      marker.pose.orientation.y = bounding_box.world_R_center.y();
      marker.pose.orientation.z = bounding_box.world_R_center.z();
      marker.pose.orientation.w = bounding_box.world_R_center.w();
      break;
    case BoundingBox::Type::AABB:
      marker.pose.position.x = bounding_box.world_P_center(0);
      marker.pose.position.y = bounding_box.world_P_center(1);
      marker.pose.position.z = bounding_box.world_P_center(2) + config.z_offset;
      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 1.0;
      break;
    default:
      ROS_ERROR("Invalid bounding box encountered!");
      break;
  }

  marker.scale.x = bounding_box.max(0) - bounding_box.min(0);
  marker.scale.y = bounding_box.max(1) - bounding_box.min(1);
  marker.scale.z = bounding_box.max(2) - bounding_box.min(2);

  marker.color = makeColorMsg(node.attributes<SemanticNodeAttributes>().color,
                              config.bounding_box_alpha);

  return marker;
}

Marker makeTextMarker(const LayerConfig& config,
                      const Node& node,
                      const std::string& marker_namespace) {
  Marker marker;
  marker.ns = marker_namespace;
  marker.id = node.id;
  marker.type = Marker::TEXT_VIEW_FACING;
  marker.action = Marker::ADD;
  marker.lifetime = ros::Duration(0);

  fillPoseWithIdentity(marker.pose);
  Eigen::Vector3d position = node.attributes().position;
  position(2) += config.z_offset + config.text_height;
  tf2::convert(position, marker.pose.position);

  marker.text = NodeSymbol(node.id).getLabel();
  marker.scale.z = config.text_scale;
  marker.color = makeColorMsg(NodeColor::Zero());

  return marker;
}

Marker makeCentroidMarkers(const LayerConfig& config,
                           const SceneGraphLayer& layer,
                           std::optional<NodeColor> layer_color,
                           const std::string& marker_namespace) {
  Marker marker;
  marker.type =
      config.use_sphere_marker ? Marker::SPHERE_LIST : Marker::CUBE_LIST;
  marker.action = visualization_msgs::Marker::ADD;
  marker.id = layer.id;
  marker.ns = marker_namespace;

  marker.scale.x = config.marker_scale;
  marker.scale.y = config.marker_scale;
  marker.scale.z = config.marker_scale;

  fillPoseWithIdentity(marker.pose);

  bool node_colors_valid = true;
  marker.points.reserve(layer.numNodes());
  marker.colors.reserve(layer.numNodes());
  for (const auto& id_node_pair : layer.nodes) {
    Eigen::Vector3d position = id_node_pair.second->attributes().position;
    position(2) += config.z_offset;

    geometry_msgs::Point node_centroid;
    tf2::convert(position, node_centroid);

    marker.points.push_back(node_centroid);

    // get the color of the node
    NodeColor desired_color;
    if (layer_color) {
      desired_color = *layer_color;
    } else if (!node_colors_valid) {
      // TODO(nathan) not ideal, but maybe not much we can do about it
      desired_color << 1.0, 0.0, 0.0;
    } else {
      try {
        desired_color =
            id_node_pair.second->attributes<SemanticNodeAttributes>().color;
      } catch (const std::bad_cast) {
        node_colors_valid = false;
        // TODO(nathan) not ideal, but maybe not much we can do about it
        desired_color << 1.0, 0.0, 0.0;
      }
    }

    marker.colors.push_back(makeColorMsg(desired_color, config.marker_alpha));
  }

  return marker;
}

Marker makeMeshEdgesMarker(const LayerConfig& config,
                           const SceneGraphLayer& layer,
                           double secondary_offset,
                           double mesh_offset) {
  Marker marker;
  marker.type = Marker::LINE_LIST;
  marker.action = Marker::ADD;
  // TODO(nathan) these probably should be arguments
  marker.id = 0;
  marker.ns = "mesh_layer_edges";

  marker.scale.x = config.edge_scale;
  fillPoseWithIdentity(marker.pose);

  for (const auto& id_node_pair : layer.nodes) {
    const Node& node = *id_node_pair.second;
    ObjectNodeAttributes attrs = node.attributes<ObjectNodeAttributes>();
    if (attrs.points == nullptr || attrs.points->size() == 0) {
      continue;
    }

    geometry_msgs::Point center_point;
    tf2::convert(attrs.position, center_point);
    center_point.z += secondary_offset;

    geometry_msgs::Point centroid_location;
    tf2::convert(attrs.position, centroid_location);
    centroid_location.z += config.z_offset;

    // make first edge
    marker.points.push_back(centroid_location);
    marker.points.push_back(center_point);
    if (config.interlayer_edge_use_color) {
      marker.colors.push_back(makeColorMsg(attrs.color, config.edge_alpha));
      marker.colors.push_back(makeColorMsg(attrs.color, config.edge_alpha));
    } else {
      marker.colors.push_back(
          makeColorMsg(NodeColor::Zero(), config.edge_alpha));
      marker.colors.push_back(
          makeColorMsg(NodeColor::Zero(), config.edge_alpha));
    }

    for (size_t i = 0; i < attrs.points->size();
         i += config.interlayer_edge_insertion_skip + 1) {
      geometry_msgs::Point vertex;
      vertex.x = attrs.points->at(i).x;
      vertex.y = attrs.points->at(i).y;
      vertex.z = attrs.points->at(i).z + mesh_offset;

      marker.points.push_back(center_point);
      marker.points.push_back(vertex);

      if (config.interlayer_edge_use_color) {
        marker.colors.push_back(makeColorMsg(attrs.color, config.edge_alpha));
        marker.colors.push_back(makeColorMsg(attrs.color, config.edge_alpha));
      } else {
        marker.colors.push_back(
            makeColorMsg(NodeColor::Zero(), config.edge_alpha));
        marker.colors.push_back(
            makeColorMsg(NodeColor::Zero(), config.edge_alpha));
      }
    }
  }

  return marker;
}

Marker makeLayerEdgeMarkers(const LayerConfig& config,
                            const SceneGraphLayer& layer,
                            const NodeColor& color) {
  Marker marker;
  marker.type = Marker::LINE_LIST;
  marker.action = Marker::ADD;
  marker.id = 0;
  marker.ns = "layer_" + std::to_string(layer.id) + "_edges";
  marker.scale.x = config.edge_scale;
  marker.color = makeColorMsg(color, config.edge_alpha);
  fillPoseWithIdentity(marker.pose);

  auto edge_iter = layer.edges.begin();
  while (edge_iter != layer.edges.end()) {
    Eigen::Vector3d source_pos = layer.getPosition(edge_iter->second.source);
    source_pos(2) += config.z_offset;
    Eigen::Vector3d target_pos = layer.getPosition(edge_iter->second.target);
    target_pos(2) += config.z_offset;

    geometry_msgs::Point source;
    tf2::convert(source_pos, source);
    geometry_msgs::Point target;
    tf2::convert(target_pos, target);

    marker.points.push_back(source);
    marker.points.push_back(target);

    std::advance(edge_iter, config.intralayer_edge_insertion_skip + 1);
  }

  return marker;
}

Marker makeMeshMarker(const LayerConfig& config,
                      const voxblox::Mesh& mesh,
                      voxblox::ColorMode color_mode,
                      const std::string& marker_namespace) {
  Marker marker;
  marker.ns = marker_namespace;
  marker.type = Marker::TRIANGLE_LIST;
  marker.scale.x = 1.0;
  marker.scale.y = 1.0;
  marker.scale.z = 1.0;
  fillPoseWithIdentity(marker.pose);

  for (size_t i = 0u; i < mesh.vertices.size(); i++) {
    geometry_msgs::Point point;
    Eigen::Vector3d vertex_pos = mesh.vertices.at(i).cast<double>();
    tf2::convert(vertex_pos, point);
    point.z += config.z_offset;
    marker.points.push_back(point);

    std_msgs::ColorRGBA color = voxblox::getVertexColor(mesh, color_mode, i);
    marker.colors.push_back(color);
  }

  return marker;
}

}  // namespace kimera
