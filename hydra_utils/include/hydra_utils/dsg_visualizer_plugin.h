#pragma once
#include <kimera_dsg/dynamic_scene_graph.h>
#include <ros/ros.h>

namespace hydra {

using kimera::DynamicSceneGraph;

class DsgVisualizerPlugin {
 public:
  using Ptr = std::shared_ptr<DsgVisualizerPlugin>;

  DsgVisualizerPlugin(const ros::NodeHandle& nh, const std::string& name)
      : nh_(nh, name) {}

  virtual ~DsgVisualizerPlugin() = default;

  virtual void draw(const std_msgs::Header& header, const DynamicSceneGraph& graph) = 0;

  virtual void reset(const std_msgs::Header& header,
                     const DynamicSceneGraph& graph) = 0;

 protected:
  ros::NodeHandle nh_;
};

}  // namespace hydra
