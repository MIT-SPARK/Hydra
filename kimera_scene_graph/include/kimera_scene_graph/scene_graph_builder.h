#pragma once
// TODO(nathan) condense
#include "kimera_scene_graph/common.h"
#include "kimera_scene_graph/object_finder.h"
#include "kimera_scene_graph/room_finder.h"
#include "kimera_scene_graph/semantic_ros_publishers.h"

#include <kimera_dsg/dynamic_scene_graph.h>

#include <kimera_dsg_visualizer/dynamic_scene_graph_visualizer.h>
#include <kimera_semantics/semantic_integrator_base.h>
#include <voxblox/core/layer.h>

#include <dynamic_reconfigure/server.h>
#include <kimera_scene_graph/kimera_scene_graphConfig.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>

#include <map>
#include <memory>
#include <vector>

namespace kimera {

class SceneGraphBuilder {
 private:
  typedef kimera_scene_graph::kimera_scene_graphConfig RqtSceneGraphConfig;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  SceneGraphBuilder(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);

  void reconstruct();

  void saveSceneGraph(const std::string& filepath) const;

  DynamicSceneGraph::Ptr getSceneGraph() { return scene_graph_; }

  void visualize();

 protected:
  bool reconstructSrvCb(std_srvs::SetBool::Request& request,
                        std_srvs::SetBool::Response& response);

  bool visualizeSrvCb(std_srvs::Trigger::Request& request,
                      std_srvs::Trigger::Response& response);

  void rqtReconfigureCallback(RqtSceneGraphConfig& config, uint32_t level);

 private:
  void loadParams();

 protected:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  std::string world_frame_;
  std::string scene_graph_output_path_;
  int object_finder_type_;
  float room_finder_esdf_slice_level_;
  std::vector<int> default_building_color_;
  std::vector<int> dynamic_labels_;
  std::vector<int> stuff_labels_;
  std::vector<int> walls_labels_;
  std::vector<int> floor_labels_;

  SemanticIntegratorBase::SemanticConfig semantic_config_;

  // To publish msgs to different topics according to semantic label.
  SemanticRosPublishers<SemanticLabel, ColorPointCloud> semantic_pcl_pubs_;

  dynamic_reconfigure::Server<RqtSceneGraphConfig> rqt_server_;
  dynamic_reconfigure::Server<RqtSceneGraphConfig>::CallbackType rqt_callback_;
  ros::ServiceServer reconstruct_scene_graph_srv_;
  ros::ServiceServer visualizer_srv_;

  std::unique_ptr<ObjectFinder> object_finder_;
  std::unique_ptr<RoomFinder> room_finder_;

  DynamicSceneGraph::Ptr scene_graph_;
  std::unique_ptr<DynamicSceneGraphVisualizer> visualizer_;

  vxb::Layer<vxb::EsdfVoxel>::Ptr esdf_layer_;

  ros::Publisher debug_pub_;

  pcl::PolygonMesh::Ptr segmented_walls_mesh_;
  pcl::PolygonMesh::Ptr rgb_mesh_;
};

}  // namespace kimera
