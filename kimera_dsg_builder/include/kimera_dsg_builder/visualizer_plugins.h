#pragma once
#include "kimera_dsg_builder/minimum_spanning_tree.h"

#include <kimera_dsg_visualizer/dsg_visualizer_plugin.h>
#include <kimera_dsg_visualizer/visualizer_types.h>
#include <kimera_pgmo/DeformationGraph.h>

#include <visualization_msgs/Marker.h>
#include <voxblox/core/block_hash.h>
#include <voxblox/core/common.h>
#include <voxblox_msgs/Mesh.h>

namespace kimera {

struct PMGraphPluginConfig {
  explicit PMGraphPluginConfig(const ros::NodeHandle& nh);

  double mesh_edge_scale = 0.005;
  double mesh_edge_alpha = 0.8;
  double mesh_marker_scale = 0.1;
  double mesh_marker_alpha = 0.8;
  NodeColor leaf_color;
  NodeColor interior_color;
  NodeColor invalid_color;
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
  MeshPlaceConnectionsPlugin(const ros::NodeHandle& nh, const std::string& name);

  virtual ~MeshPlaceConnectionsPlugin() = default;

  void draw(const std_msgs::Header& header, const DynamicSceneGraph& graph) override;

  void reset(const std_msgs::Header& header, const DynamicSceneGraph& graph) override;

 protected:
  ros::Publisher marker_pub_;
  PMGraphPluginConfig config_;
  bool published_nodes_;
  bool published_edges_;
};

class PlacesFactorGraphViz {
 public:
  using Ptr = std::shared_ptr<PlacesFactorGraphViz>;

  explicit PlacesFactorGraphViz(const ros::NodeHandle& nh);

  virtual ~PlacesFactorGraphViz() = default;

  void draw(char vertex_prefix,
            const SceneGraphLayer& places,
            const MinimumSpanningTreeInfo& mst_info,
            const kimera_pgmo::DeformationGraph& deformations);

 protected:
  ros::NodeHandle nh_;
  ros::Publisher marker_pub_;
  PMGraphPluginConfig config_;
};

class PlaceParentsPlugin : public DsgVisualizerPlugin {
 public:
  PlaceParentsPlugin(const ros::NodeHandle& nh, const std::string& name);

  virtual ~PlaceParentsPlugin() = default;

  void draw(const std_msgs::Header& header, const DynamicSceneGraph& graph) override;

  void reset(const std_msgs::Header& header, const DynamicSceneGraph& graph) override;

 protected:
  ros::Publisher marker_pub_;
  PMGraphPluginConfig config_;
  bool published_nodes_;
  bool published_edges_;
};

}  // namespace kimera
