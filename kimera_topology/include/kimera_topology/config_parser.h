#pragma once
#include "kimera_topology/gvd_integrator.h"
#include "kimera_topology/gvd_visualization_utilities.h"
#include "kimera_topology/topology_server_visualizer.h"

#include <voxblox_ros/mesh_vis.h>

#include <ros/ros.h>

namespace kimera {
namespace topology {

void fillGraphExtractorConfig(const ros::NodeHandle& nh, GraphExtractorConfig& config);

void fillGvdIntegratorConfig(const ros::NodeHandle& nh, GvdIntegratorConfig& config);

// forward declaration to avoid loop
struct TopologyServerConfig {
  double update_period_s = 1.0;
  bool show_stats = true;
  bool clear_distant_blocks = true;
  double dense_representation_radius_m = 5.0;

  voxblox::ColorMode mesh_color_mode = voxblox::ColorMode::kLambertColor;
  std::string world_frame = "world";
};

void fillTopologyServerConfig(const ros::NodeHandle& nh, TopologyServerConfig& config);

void fillTopologyServerVizConfig(const ros::NodeHandle& nh,
                                 TopologyVisualizerConfig& config);

GvdVisualizerConfig getGvdVisualizerConfig(const std::string& ns);

void showConfig(const GvdIntegratorConfig& config);

}  // namespace topology
}  // namespace kimera
