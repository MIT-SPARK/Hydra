#pragma once
// TODO(nathan) condense
#include "kimera_scene_graph/common.h"
#include "kimera_scene_graph/object_finder.h"
#include "kimera_scene_graph/room_finder.h"
#include "kimera_scene_graph/scene_graph_visualizer.h"
#include "kimera_scene_graph/semantic_ros_publishers.h"
#include "kimera_scene_graph/wall_finder.h"

#include <kimera_semantics/semantic_integrator_base.h>
#include <voxblox/core/layer.h>
#include <voxblox/integrator/esdf_integrator.h>
#include <voxblox_ros/mesh_vis.h>
#include <voxblox_skeleton/skeleton.h>

#include <dynamic_reconfigure/server.h>
#include <kimera_scene_graph/kimera_scene_graphConfig.h>
#include <std_srvs/Empty.h>
#include <std_srvs/EmptyRequest.h>
#include <std_srvs/EmptyResponse.h>
#include <std_srvs/SetBool.h>

#include <iostream>
#include <map>
#include <memory>
#include <vector>

namespace kimera {

typedef ColorPointCloud SemanticPointCloud;
typedef std::unordered_map<SemanticLabel, SemanticPointCloud::Ptr>
    SemanticPointCloudMap;
typedef std::unordered_map<SemanticLabel, vxb::Mesh::Ptr> SemanticMeshMap;

class SceneGraphBuilder {
 private:
  typedef kimera_scene_graph::kimera_scene_graphConfig RqtSceneGraphConfig;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  SceneGraphBuilder(const ros::NodeHandle& nh,
                    const ros::NodeHandle& nh_private);

  bool sceneGraphReconstructionServiceCall(
      std_srvs::SetBool::Request& request,
      std_srvs::SetBool::Response& response);

  void sceneGraphReconstruction();

  inline bool loadSceneGraph(const std::string& /*file_path*/) const {
    return false;
  }

  /**
   * @brief getSceneGraph Return the current scene-graph, you should call the
   * sceneGraphReconstruction function or the
   * sceneGraphReconstructionServiceCall
   * before getting the scene_graph via this function
   * @return Pointer to the Scene-Graph
   */
  SceneGraph::Ptr getSceneGraph() { return scene_graph_; }

  void visualize() const;

 protected:
  void rqtReconfigureCallback(RqtSceneGraphConfig& config, uint32_t level);

  void reconstructMeshOutOfTsdf(vxb::Layer<vxb::TsdfVoxel>* tsdf_layer,
                                vxb::MeshLayer::Ptr mesh_layer);

  void reconstructEsdfOutOfTsdf(bool save_to_file);

  void publishSemanticMesh(const SemanticLabel& semantic_label,
                           const vxb::Mesh& semantic_mesh);

  void getSemanticPointcloudsFromMesh(
      const vxb::MeshLayer::ConstPtr& mesh_layer,
      const vxb::ColorMode& color_mode,
      SemanticPointCloudMap* semantic_pointclouds);

  void getSemanticMeshesFromMesh(const vxb::MeshLayer::ConstPtr& mesh_layer,
                                 const vxb::ColorMode& color_mode,
                                 SemanticMeshMap* semantic_pointclouds);

  bool loadTsdfMap(const std::string& file_path,
                   vxb::Layer<vxb::TsdfVoxel>* tsdf_layer);

  bool loadEsdfMap(const std::string& file_path);

 protected:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  ros::Publisher color_clustered_pcl_pub_;
  ros::Publisher walls_clustered_pcl_pub_;
  ros::Publisher room_centroids_pub_;
  ros::Publisher mesh_pub_;
  ros::Publisher rgb_mesh_pub_;
  ros::Publisher polygon_mesh_pub_;
  ros::Publisher sparse_graph_pub_;

  // TODO(Toni): This guy should be in the scene graph visualization itself
  ros::Publisher edges_obj_skeleton_pub_;

  std::string world_frame_;
  std::string scene_graph_output_path_;
  float room_finder_esdf_slice_level_;
  float voxel_size_;
  int voxels_per_side_;

  // Rebuild esdf and save to file
  bool build_esdf_batch_ = false;

  // To publish msgs to different topics according to semantic label.
  SemanticRosPublishers<SemanticLabel, ColorPointCloud> semantic_pcl_pubs_;

  // Dynamically change params for scene graph reconstruction
  dynamic_reconfigure::Server<RqtSceneGraphConfig> rqt_server_;
  dynamic_reconfigure::Server<RqtSceneGraphConfig>::CallbackType rqt_callback_;

  // Dynamically request a scene graph reconstruction
  ros::ServiceServer reconstruct_scene_graph_srv_;

  // Finders
  std::unique_ptr<EnclosingWallFinder> enclosing_wall_finder_;
  std::unique_ptr<ObjectFinder> object_finder_;
  std::unique_ptr<RoomFinder> room_finder_;

  // Labels of interesting things
  std::vector<int> dynamic_labels_;
  std::vector<int> stuff_labels_;
  std::vector<int> walls_labels_;
  std::vector<int> floor_labels_;

  // KimeraX
  SceneGraph::Ptr scene_graph_;
  SceneGraphVisualizer visualizer_;

  // Layers
  std::unique_ptr<vxb::Layer<vxb::TsdfVoxel>> tsdf_layer_;
  std::unique_ptr<vxb::Layer<vxb::TsdfVoxel>> tsdf_layer_rgb_;
  std::unique_ptr<vxb::Layer<vxb::EsdfVoxel>> esdf_layer_;

  // Integrators
  SemanticIntegratorBase::SemanticConfig semantic_config_;
  std::unique_ptr<vxb::EsdfIntegrator> esdf_integrator_;

  // Skeleton graph
  vxb::SparseSkeletonGraph sparse_skeleton_graph_;

  // Structure
  // TODO(nathan) this should get subsumed by the scene graph
  vxb::Mesh::Ptr segmented_walls_mesh_;
  vxb::MeshLayer::Ptr rgb_mesh_;
  vxb::MeshLayer::Ptr semantic_mesh_;

  // KimeraX Dynamic
  // DynamicSceneGraph dynamic_scene_graph_;
};

}  // namespace kimera
