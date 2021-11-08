#pragma once
#include <kimera_dsg_visualizer/dsg_visualizer_plugin.h>
#include <kimera_dsg_visualizer/visualizer_types.h>
#include <kimera_pgmo/DeformationGraph.h>

#include <voxblox/core/common.h>
#include <voxblox/core/block_hash.h>
#include <voxblox_msgs/Mesh.h>
#include <visualization_msgs/Marker.h>

namespace kimera {

struct PMGraphPluginConfig {
  explicit PMGraphPluginConfig(const ros::NodeHandle& nh);

  double mesh_edge_scale = 0.005;
  double mesh_edge_alpha = 0.8;
  double mesh_marker_scale = 0.1;
  double mesh_marker_alpha = 0.8;
  NodeColor leaf_color;
  NodeColor interior_color;
  LayerConfig layer_config;
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

class MeshPlaceConnectionsPlugin : public DsgVisualizerPlugin {
 public:
  MeshPlaceConnectionsPlugin(const ros::NodeHandle& nh,
                             const std::string& name,
                             char vertex_prefix,
                             const kimera_pgmo::DeformationGraphPtr& graph);

  virtual ~MeshPlaceConnectionsPlugin() = default;

  void draw(const std_msgs::Header& header, const DynamicSceneGraph& graph) override;

  void reset(const std_msgs::Header& header, const DynamicSceneGraph& graph) override;

 protected:
  ros::Publisher marker_pub_;
  char vertex_prefix_;
  kimera_pgmo::DeformationGraphPtr deformation_graph_;
  PMGraphPluginConfig config_;
};

}  // namespace kimera
