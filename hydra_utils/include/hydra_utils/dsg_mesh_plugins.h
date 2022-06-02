#pragma once
#include "hydra_utils/dsg_visualizer_plugin.h"

#include <voxblox/core/block_hash.h>
#include <voxblox/core/common.h>

namespace hydra {

class RvizMeshPlugin : public DsgVisualizerPlugin {
 public:
  RvizMeshPlugin(const ros::NodeHandle& nh, const std::string& name);

  virtual ~RvizMeshPlugin() = default;

  void draw(const std_msgs::Header& header, const DynamicSceneGraph& graph) override;

  void reset(const std_msgs::Header& header, const DynamicSceneGraph& graph) override;

 private:
  std::string name_;
  ros::Publisher mesh_pub_;
  bool published_mesh_;
};

class PgmoMeshPlugin : public DsgVisualizerPlugin {
 public:
  PgmoMeshPlugin(const ros::NodeHandle& nh, const std::string& name);

  virtual ~PgmoMeshPlugin() = default;

  void draw(const std_msgs::Header& header, const DynamicSceneGraph& graph) override;

  void reset(const std_msgs::Header& header, const DynamicSceneGraph& graph) override;

 protected:
  ros::Publisher mesh_pub_;
};

class VoxbloxMeshPlugin : public DsgVisualizerPlugin {
 public:
  VoxbloxMeshPlugin(const ros::NodeHandle& nh, const std::string& name);

  virtual ~VoxbloxMeshPlugin() = default;

  void draw(const std_msgs::Header& header, const DynamicSceneGraph& graph) override;

  void reset(const std_msgs::Header& header, const DynamicSceneGraph& graph) override;

 protected:
  ros::Publisher mesh_pub_;
  voxblox::BlockIndexList curr_blocks_;
};

}  // namespace hydra
