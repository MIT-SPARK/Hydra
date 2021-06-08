#pragma once
#include "kimera_scene_graph/common.h"
#include "kimera_scene_graph/semantic_ros_publishers.h"
#include "kimera_scene_graph/visualizer_utils.h"

#include <ros/ros.h>

#include <kimera_dsg/scene_graph.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <string>
#include <vector>

namespace kimera {

// TODO(nathan) reconsider rviz visual tools

class SceneGraphVisualizer {
 public:
  SceneGraphVisualizer(const ros::NodeHandle& nh,
                       const ros::NodeHandle& nh_private);

  virtual ~SceneGraphVisualizer() = default;

  // Visualization
  inline void visualize(const SceneGraph& scene_graph) const {
    visualizeImpl(scene_graph);
  }

  inline void updateEdgeAlpha(const float& alpha) { edge_alpha_ = alpha; }

  void visualizeMesh(voxblox::MeshLayer* mesh,
                     voxblox::ColorMode color_mode,
                     bool is_rgb_mesh = true) const;

  void visualizeWalls(const voxblox::Mesh& mesh) const;

 protected:
  virtual void visualizeImpl(const SceneGraph& scene_graph) const;

  void fillHeader(visualization_msgs::Marker& marker,
                  const ros::Time& current_time) const;

  void displayLayers(const SceneGraph& scene_graph) const;

  void displayEdges(const SceneGraph& scene_graph) const;

  visualization_msgs::Marker makeGraphEdgeMarkers(
      const SceneGraph& scene_graph,
      const std::map<LayerId, LayerConfig>& configs,
      double scale = 0.07) const;

 protected:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  // ROS publishers
  ros::Publisher semantic_instance_centroid_pub_;
  ros::Publisher bounding_box_pub_;
  ros::Publisher edges_centroid_pcl_pub_;
  ros::Publisher edges_node_node_pub_;
  ros::Publisher text_markers_pub_;
  ros::Publisher wall_pub_;

  ros::Publisher semantic_mesh_pub_;
  ros::Publisher rgb_mesh_pub_;

  // Reference frame id
  std::string world_frame_;

  // Visualization params.
  double edge_alpha_ = 0.01;

  // Step in Z axis for the different layers of the scene graph
  double layer_step_z_ = 15;
};

}  // namespace kimera
