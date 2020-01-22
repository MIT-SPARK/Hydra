#pragma once

#include <iostream>
#include <map>
#include <memory>
#include <vector>

#include <glog/logging.h>

#include <voxblox/core/layer.h>
#include <voxblox_ros/mesh_vis.h>

#include <dynamic_reconfigure/server.h>
#include <kimera_scene_graph/kimera_scene_graphConfig.h>
#include <std_srvs/Empty.h>
#include <std_srvs/EmptyRequest.h>
#include <std_srvs/EmptyResponse.h>

#include <kimera_semantics_ros/semantic_simulation_server.h>

#include "kimera_scene_graph/common.h"
#include "kimera_scene_graph/object_finder.h"
#include "kimera_scene_graph/room_finder.h"
#include "kimera_scene_graph/scene_node.h"
#include "kimera_scene_graph/semantic_ros_publishers.h"

namespace kimera {

typedef ColoredPointCloud SemanticPointCloud;
typedef std::unordered_map<SemanticLabel, SemanticPointCloud::Ptr>
    SemanticPointCloudMap;
typedef std::unordered_map<SemanticLabel, vxb::Mesh::Ptr> SemanticMeshMap;

class SceneGraphSimulationServer : public SemanticSimulationServer {
 private:
  typedef kimera_scene_graph::kimera_scene_graphConfig RqtSceneGraphConfig;

 public:
  SceneGraphSimulationServer(const ros::NodeHandle& nh,
                             const ros::NodeHandle& nh_private);

  bool sceneGraphReconstructionServiceCall(std_srvs::Empty::Request& request,
                                           std_srvs::Empty::Response& response);

  void sceneGraphReconstruction();

  void getSemanticPointcloudsFromMesh(
      const vxb::MeshLayer::ConstPtr& mesh_layer,
      const vxb::ColorMode& color_mode,
      SemanticPointCloudMap* semantic_pointclouds);

  void getSemanticMeshesFromMesh(
      const vxb::MeshLayer::ConstPtr& mesh_layer,
      const vxb::ColorMode& color_mode,
      SemanticMeshMap* semantic_pointclouds);

  void getPointcloudFromMesh(const vxb::MeshLayer::ConstPtr& mesh_layer,
                             vxb::ColorMode color_mode,
                             ColoredPointCloud::Ptr pointcloud);

 private:
  void rqtReconfigureCallback(RqtSceneGraphConfig& config, uint32_t level);

 private:
  ros::Publisher color_clustered_pcl_pub_;
  ros::Publisher room_centroids_pub_;
  ros::Publisher mesh_pub_;
  ros::Publisher polygon_mesh_pub_;

  // To publish msgs to different topics according to semantic label.
  SemanticRosPublishers<SemanticLabel, ColoredPointCloud> semantic_pcl_pubs_;
  SemanticRosPublishers<SemanticLabel, visualization_msgs::Marker> semantic_mesh_pubs_;

  // Dynamically change params for scene graph reconstruction
  dynamic_reconfigure::Server<RqtSceneGraphConfig> rqt_server_;
  dynamic_reconfigure::Server<RqtSceneGraphConfig>::CallbackType rqt_callback_;

  // Dynamically request a scene graph reconstruction
  ros::ServiceServer reconstruct_scene_graph_srv_;
  ros::ServiceServer load_map_srv_;

  // Object finder
  std::unique_ptr<ObjectFinder<ColorPoint>> object_finder_;

  // Room finder
  std::unique_ptr<RoomFinder> room_finder_;

  // Labels of interesting things
  std::vector<int> stuff_labels_;
  std::vector<int> walls_labels_;
  std::vector<int> floor_labels_;

  // KimeraX
  SceneGraph scene_graph_;
};

}  // namespace kimera
