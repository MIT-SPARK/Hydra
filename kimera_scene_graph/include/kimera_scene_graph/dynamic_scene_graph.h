#pragma once

#include <ros/ros.h>
#include <KimeraRPGO/utils/type_utils.h>
#include <KimeraRPGO/RobustSolver.h>
#include <gtsam/geometry/Pose3.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf/transform_broadcaster.h>
#include <std_srvs/Empty.h>
#include <graph_cmr_ros/SMPL.h>
#include <graph_cmr_ros/SMPLList.h>
#include <visualization_msgs/Marker.h>
#include <voxblox_msgs/FilePath.h>

#include <Eigen/Core>
#include <iostream>
#include <string>
#include <vector>

#include "kimera_scene_graph/scene_graph_node.h"
#include "kimera_scene_graph/semantic_ros_publishers.h"
#include "kimera_scene_graph/humans_serialization.h"

namespace kimera {

// Number of joints sent over by the GraphCMR code.
static constexpr int NUM_JOINTS = 19;
static constexpr int NUM_BETAS = 10;
using JointMatrix = Eigen::Matrix<double, NUM_JOINTS, 3, Eigen::RowMajor>;
using BetasArray = std::vector<double>;

using AgentId = long int;
struct DynamicHumanNode : SceneGraphNode {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  gtsam::Pose3 pose_;
  JointMatrix joints_;
  BetasArray betas_;
  AgentId agent_id_;
  graph_cmr_ros::SMPL msg_;
};

using DynamicHumanNodes = std::vector<DynamicHumanNode>;
using DynamicHumanNodeList = std::vector<DynamicHumanNodes>;
using CentroidNodeList =
    std::vector<std::pair<gtsam::Pose3, DynamicHumanNode*>>;

struct DynamicHumanNodesAndPGO {
  DynamicHumanNodesAndPGO(const KimeraRPGO::RobustSolverParams& params)
      : human_nodes_(),
        pgo_(KimeraRPGO::make_unique<KimeraRPGO::RobustSolver>(params)) {}

  DynamicHumanNodes human_nodes_;
  std::unique_ptr<KimeraRPGO::RobustSolver> pgo_;
};

class DynamicSceneGraph {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  DynamicSceneGraph(const ros::NodeHandle& nh,
                    const ros::NodeHandle& nh_private);
  virtual ~DynamicSceneGraph() = default;

 public:
  /**
   * @brief Service wrapper to deserialize DynamicSceneGraph messages from
   * the deserialization path and visualizes them in rviz.
   * @param request: The empty request filled in by the ros service call
   * @param response: The empty response.
   * @return True always.
   */
  bool deserializeServiceCall(
      voxblox_msgs::FilePath::Request& request,
      voxblox_msgs::FilePath::Response& response);

  /**
   * @brief Service wrapper to deserialize DynamicSceneGraph messages from
   * the deserialization path and visualizes them in rviz.
   * @param request: The empty request filled in by the ros service call
   * @param response: The empty response.
   * @return True always.
   */
  bool serializeServiceCall(std_srvs::Empty::Request& request,
                            std_srvs::Empty::Response& response);

  /**
   * @brief humanCallback populates the DynamicSceneGraph with humans from
   * SMPLList messages.
   * @param msg: The SMPLList message containing the detected human SMPLs and
   * their centroids.
   */
  void humanCallback(const graph_cmr_ros::SMPLList::ConstPtr& msg);

  /**
   * @brief visualizePoseGraphs publishes the centroids of the humans and the
   * edges connecting them
   * for each human individually. It additionally publishes all the centroids
   * and all the edges.
   */
  void visualizePoseGraphs(HumansSerializer* serializer = nullptr);

  /**
   * @brief publishOptimizedMeshesAndVis visualizes the skeleton edges for each
   * human, the joint vertices, and an SMPL list with the SMPL of the final
   * human SMPL in each pose graph.
   * @param serializer optional serializer handle
   */
  void publishOptimizedMeshesAndVis(HumansSerializer* serializer = nullptr);

  /**
   * @brief addSceneNode adds a new DynamicHumanNode to the graph.
   * @param scene_node: The scene node to be added.
   */
  inline void addSceneNode(DynamicHumanNode& scene_node) {
    // Expectation that scene_node.id_ is blank, use addHumanNodeToPoseGraphs to
    // get id
    addHumanNodeToPoseGraphs(scene_node);
  }

  /**
   * @brief getAllOptimizedHumanNodes optimizes the poses of the dynamic scene
   * nodes.
   * @param optimized_node_list list of lists of optimized DynamicHumanNodes
   * for each human after pruning small pose-graphs.
   */
  void getAllOptimizedHumanNodes(
      DynamicHumanNodeList* optimized_node_list) const;

  /**
   * getOptimizedPoses collects poses of the human pgo at index idx.
   * @param idx index in human_db_, representing the human for the query
   * @param pose_list output list of poses to be filled for the human
   * @param include_prior appends the identity world-pose to the front of the
   * human's trajectory
   */
  void getOptimizedPoses(size_t idx,
                         std::vector<gtsam::Pose3>* pose_list,
                         bool include_prior = false) const;

  /**
   * getOptimizedAndPrunedPoses collects poses of the human pgo at given
   * index IFF the human's pgo is not to be pruned (has enough detections).
   * @param idx index in human_db_, representing the human for the query
   * @param pose_list output list of poses to be filled for the human
   * @param include_prior appends the identity world-pose to the front of the
   * human's trajectory
   */
  bool getOptimizedAndPrunedPoses(size_t idx,
                                  std::vector<gtsam::Pose3>* pose_list,
                                  bool include_prior = false) const;

  /**
   * getAllOptimizedAndPrunedPoses collects pose list for each human pgo.
   * pgos that are pruned will just have an empty list of poses.
   * @param pose_lists a vector of pose-vectors, indexed by pgo id.
   * @param include_prior appends the identity world-pose to the front of the
   * human's trajectory.
   */
  void getAllOptimizedAndPrunedPoses(
      std::vector<std::vector<gtsam::Pose3>>* pose_lists,
      bool include_prior) const;

  /**
   * @brief DynamicHumanNodeFromSMPL converts an SMPL into a DynamicHumanNode
   * representation.
   * @param mesh: The SMPL message to be converted into a node.
   * @param node: A node object to be filled with the resulting
   * DynamicHumanNode.
   */
  static void DynamicHumanNodeFromSMPL(const graph_cmr_ros::SMPL& mesh,
                                       const std::vector<float>& human_color,
                                       DynamicHumanNode* node);

  /**
   * @brief isPoseDynamicallyFeasible checks whether the distance between
   * centroids is possible given the rate.
   * @param scene_node: The node to be checked as a feasible node in the graph.
   * @param last_node: The last node currently in the graph to be checked
   * against.
   * @return true if the distance is feasible or if filter_centroid is set to
   * false in the launch file.
   */
  bool isPoseDynamicallyFeasible(const DynamicHumanNode& scene_node,
                                 const DynamicHumanNode& last_node) const;

  /**
   * @brief isMeshDynamicallyFeasible checks whether the largest distance between
   * joints is possible given the rate.
   * @param scene_node: The node to be checked as a feasible node in the graph.
   * @param last_node: The last node currently in the graph to be checked
   * against.
   * @return true if the distance is feasible or if filter_centroid is set to
   * false in the launch file.
   */
  bool isMeshDynamicallyFeasible(const DynamicHumanNode& scene_node,
                                 const DynamicHumanNode& last_node) const;

  // TODO(marcus): how to visualize and test?
  /**
   * @brief isShapeFeasible checks whether the beta parameters of two nodes
   * are similar enough to belong to the same human.
   * @param scene_node: The node to be checked as a feasible node in the graph.
   * @param last_node: The last node currently in the graph to be checked
   * against.
   * @return true if the nodes have similar beta parameters.
   */
  bool isShapeFeasible(const DynamicHumanNode& scene_node,
                       const DynamicHumanNode& last_node) const;

 private:
  /////////// Helper Functions to Simplify code. /////////////
  /**
   * @brief deserializeAndPublish deserializes messages from the
   * deserialization_file
   *  and publishes them.
   */
  bool deserializeAndPublish(const std::string& deserialization_file);

  /**
   * @brief serialize the messages to a bag file in the serialization_dir_ named
   * based on the number
   *  of pose graphs.
   */
  bool serialize();

  static void colorPclFromJoints(const std::vector<float>& human_color,
                                 DynamicHumanNode* scene_node);

  int findClosestFeasibleHuman(const DynamicHumanNode& scene_node) const;

  void addHumanNodeToPoseGraphs(DynamicHumanNode& scene_node);

  inline float calcNodeDistance(const DynamicHumanNode& scene_node,
                                const DynamicHumanNode& last_node) const {
    auto sp = scene_node.attributes_.position_;
    auto ep = last_node.attributes_.position_;
    return std::sqrt(std::pow(sp.x - ep.x, 2) + std::pow(sp.y - ep.y, 2) +
                     std::pow(sp.z - ep.z, 2)); // TODO(marcus): use std functs or gtsam.norm
  }

  inline bool isPositionClose(const DynamicHumanNode& scene_node,
                             const DynamicHumanNode& last_node) const {
    return calcNodeDistance(scene_node, last_node) < min_node_dist_;
  }

  inline void setupEdgeMarker(visualization_msgs::Marker& edges,
                              std::vector<float>& color);

  void drawEdgesBetweenSkeletons(const DynamicHumanNode& start,
                                 const DynamicHumanNode& end,
                                 visualization_msgs::Marker& out_edges);

  void drawSkeletonEdges(DynamicHumanNode& node,
                         visualization_msgs::Marker& marker);

 public:
  // ROS Params
  double feasible_dyn_rate_ = 2.0;
  double feasible_mesh_rate_ = 1.5;
  double time_cap_ = 2.0;
  double min_node_dist_ = 0.3;
  double max_pose_rot_ = 3.14/2.0;  // 90deg rotation between detections = bad
  std::vector<float> human_color_ = {0.0, 1.0, 0.0};
  std::vector<float> edge_color_ = {0.0, 0.0, 1.0};
  std::vector<float> centroid_color_ = {0.0, 0.0, 1.0};
  double pgo_trans_threshold_ = 0.05;
  double pgo_rot_threshold_ = 0.005;
  double beta_diff_threshold_ = 0.1;
  size_t prune_threshold_ = 0;
  bool single_sequence_smpl_mode_ = false;
  bool draw_skeleton_edges_ = false;
  std::string human_topic_{"human_meshes"};
  bool check_position_closeness_ = true;
  bool check_pose_dynamic_feasibility_ = true;
  bool check_mesh_dynamic_feasibility_ = true;
  bool check_shape_feasibility_ = true;
  bool visualize_pose_graphs_ = false;
  bool visualize_joints_ = false;

  // Constant joint lookup table
  // TODO(marcus): check consistency with GraphCMR
  const std::vector<int> joint_parents_ =
      {1, 2, 8, 9, 3, 4, 7, 8, 12, 12, 9, 10, 14, -1, 13, -1, -1, 15, 16};

  // Record Keeping for pose graphs.
  std::vector<DynamicHumanNodesAndPGO> human_db_;
  KimeraRPGO::RobustSolverParams rpgo_params_;

  // Noise models + params
  gtsam::SharedNoiseModel detection_noise_model_;
  gtsam::SharedNoiseModel motion_noise_model_;
  double detection_position_variance_ = 0.1;
  double detection_rotation_variance_ = 0.1;
  double motion_position_variance_ = 1.0;
  double motion_rotation_variance_ = 1.0;

 protected:
  ros::NodeHandle nh_private_;
  ros::Subscriber human_sub_;
  ros::Publisher mesh_pub_;
  ros::Publisher edges_pub_;
  ros::Publisher semantic_instance_centroid_pub_;

  SemanticRosPublishers<AgentId, ColorPointCloud> skeleton_points_pubs_;
  SemanticRosPublishers<AgentId, visualization_msgs::Marker>
      skeleton_edge_pubs_;

  SemanticRosPublishers<AgentId, ColorPointCloud> agent_centroids_;
  SemanticRosPublishers<AgentId, visualization_msgs::Marker> agent_graph_edges_;

  tf::TransformBroadcaster br_;

  ros::ServiceServer serialize_graph_srv_;
  ros::ServiceServer deserialize_graph_srv_;
  std::string joint_prefix_ = "joints_";
  std::string joint_edges_prefix_ = "joint_edges_";
  std::string graph_edges_prefix_ = "graph_edges_";
  std::string graph_nodes_prefix_ = "graph_nodes_";

  std::string world_frame_ = "world";

  float edge_thickness_ = 0.02;
};

}  // namespace kimera
