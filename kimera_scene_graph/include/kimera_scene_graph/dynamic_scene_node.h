#pragma once

#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <Eigen/Core>

#include <pcl_conversions/pcl_conversions.h>

#include <boost/filesystem.hpp>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <std_srvs/Empty.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>

#include <graph_cmr_ros/SMPL.h>
#include <graph_cmr_ros/SMPLList.h>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>

#include <voxblox_msgs/FilePath.h>

#include "KimeraRPGO/RobustSolver.h"

#include "kimera_scene_graph/CentroidErrorRequest.h"
#include "kimera_scene_graph/scene_graph_node.h"
#include "kimera_scene_graph/scene_graph.h"
#include "kimera_scene_graph/semantic_ros_publishers.h"

namespace kimera {

typedef gtsam::NonlinearFactorGraph DynamicNodePoseGraph;
typedef gtsam::BetweenFactor<gtsam::Pose3> OdometryFactor;
// Number of joints sent over by the GraphCMR code.
// TODO(marcus): update this if necessary!
static constexpr int NUM_JOINTS = 19;
static constexpr int NUM_BETAS = 10;
using JointMatrix = Eigen::Matrix<double, NUM_JOINTS, 3, Eigen::RowMajor>;
using BetasArray = std::vector<double>;

using AgentId = long int;
struct DynamicSceneNode : SceneGraphNode {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  gtsam::Pose3 pose_;
  JointMatrix joints_;
  BetasArray betas_;
  AgentId agent_id_;
  graph_cmr_ros::SMPL msg_;
};

using DynamicSceneNodes = std::vector<DynamicSceneNode>;
using DynamicNodeList = std::vector<DynamicSceneNodes>;
using CentroidNodeList =
    std::vector<std::pair<gtsam::Pose3, DynamicSceneNode*>>;

class DynamicSceneGraph : public SceneGraph {
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
      voxblox_msgs::FilePath::Response& /*response*/) {  // NOLINT
    LOG(INFO) << "DynamicSceneNode: Deserialization Requested...";
    if (!deserializeAndPublish(request.file_path)) {
      LOG(ERROR) << "Failed to load map!";
      return false;
    } else {
      LOG(INFO) << "Successful deserialization request.";
      return true;
    }
  }

  /**
   * @brief Service wrapper to deserialize DynamicSceneGraph messages from
   * the deserialization path and visualizes them in rviz.
   * @param request: The empty request filled in by the ros service call
   * @param response: The empty response.
   * @return True always.
   */
  bool serializeServiceCall(std_srvs::Empty::Request& request,
                            std_srvs::Empty::Response& response) {
    LOG(INFO) << "DynamicSceneNode: Serialization Requested";
    serialize();
    return true;
  }

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
   * @param serialize: If true, serializes the messages to the
   * serialization_dir_ specified in the launch file.
   */
  void visualizePoseGraphs(bool serialize = false);

  /**
   * @brief visualizeJoints visualizes the skeleton edges for each human, the
   * joint vertices, and an SMPL list with the
   * SMPL of the final human SMPL in each pose graph.
   * @param serialize: If true, serializes the messages to the
   * serialization_dir_ specified in the launch file.
   */
  void visualizeJoints(bool serialize = false);

  /**
   * @brief addSceneNode adds a new dynamicSceneNode to the graph.
   * @param scene_node: The scene node to be added.
   */
  inline void addSceneNode(DynamicSceneNode& scene_node) {
    // Expectation that scene_node.id_ is blank, use addSceneNodetoPoseGraphs to
    // get id
    addSceneNodeToPoseGraphs(scene_node);
    SceneGraph::addSceneNode(scene_node);
  }

  /**
   * @brief getNodes gets the nodes from the graph in lists based on pose graph
   * membership.
   */
  inline DynamicNodeList* getNodes() { return &last_poses_; }

  /**
   * @brief optimizedGraph prune and optimize the pose graph.
   * @param optimized_nodes: An empty vector to be filled with the resulting
   * nodes.
   * TODO(marcus): why can't this be a void that just changes internal member?
   */
  inline void optimizedGraph(DynamicNodeList& optimized_node_list) {
    for (size_t i = 0; i < graph_priors_.size(); i++) {
      if (counts_[i] < prune_threshold_)
        continue;

      gtsam::Values result = pgos_[i]->calculateBestEstimate();  // TODO(marcus): is this useful?
      std::vector<DynamicSceneNode> optimized_nodes;
      for (size_t j = 0; j < result.size(); j++) {
        // Purposefully copy.
        auto dynamic_node = last_poses_[i][j];
        // dynamic_node.pose_ = result.at(j).cast<gtsam::Pose3>();
        optimized_nodes.push_back(dynamic_node);
      }
      optimized_node_list.push_back(optimized_nodes);
      optimized_poses_.push_back(optimized_nodes);
    }
  }

  /**
   * @brief dynamicSceneNodeFromSMPL converts an SMPL into a DynamicSceneNode
   * representation.
   * @param mesh: The SMPL message to be converted into a node.
   * @param node: A node object to be filled with the resulting
   * DynamicSceneNode.
   */
  void dynamicSceneNodeFromSMPL(graph_cmr_ros::SMPL& mesh,
                                DynamicSceneNode& node);

  /**
   * @brief checkDynamicFeasibility checks whether the distance between
   * centroids is possible given the rate.
   * @param scene_node: The node to be checked as a feasible node in the graph.
   * @param last_node: The last node currently in the graph to be checked
   * against.
   * @return true if the distance is feasible or if filter_centroid is set to
   * false in the launch file.
   */
  inline bool checkDynamicFeasibility(const DynamicSceneNode& scene_node,
                                      const DynamicSceneNode& last_node) {
    float distance = calcNodeDistance(scene_node, last_node);
    double time_diff =
        (scene_node.attributes_.timestamp_ - last_node.attributes_.timestamp_) *
        pow(10, -6);
    return !filter_centroid_ ||
           distance <=
               feasible_dyn_rate_ * time_diff;  // Fail if the node is too far
  }

  /**
   * @brief checkMeshFeasibility checks whether the largest distance between
   * joints is possible given the rate.
   * @param scene_node: The node to be checked as a feasible node in the graph.
   * @param last_node: The last node currently in the graph to be checked
   * against.
   * @return true if the distance is feasible or if filter_centroid is set to
   * false in the launch file.
   */
  inline bool checkMeshFeasibility(const DynamicSceneNode& scene_node,
                                   const DynamicSceneNode& last_node) {
    JointMatrix diff = scene_node.joints_ - last_node.joints_;
    double max_dist = diff.rowwise().norm().maxCoeff();
    double time_diff =
        (scene_node.attributes_.timestamp_ - last_node.attributes_.timestamp_) *
        pow(10, -6);
    return !filter_mesh_ || max_dist <= feasible_mesh_rate_ * time_diff;
  }

  /**
   * @brief checkBetaFeasibility checks whether the beta parameters of two nodes
   * are similar enough to belong to the same human.
   * @param scene_node: The node to be checked as a feasible node in the graph.
   * @param last_node: The last node currently in the graph to be checked 
   * against.
   * @return true if the nodes have similar beta parameters.
   */
  inline bool checkBetaFeasibility(const DynamicSceneNode& scene_node,
                                   const DynamicSceneNode& last_node) {
    CHECK_EQ(scene_node.betas_.size(), last_node.betas_.size());

    size_t num_betas = scene_node.betas_.size();
    // double beta_diffs [num_betas];
    // for (size_t i = 0; i < num_betas; i++) {
    //   beta_diffs[i] = abs(scene_node.betas_[i] - last_node.betas_[i]);
    // }

    double avg_beta_diff = 0.0;
    for (size_t i = 0; i < num_betas; i++) {
      avg_beta_diff += (abs(scene_node.betas_[i] - last_node.betas_[i]) / 
                        num_betas);
    }

    return avg_beta_diff <= beta_diff_threshold_;
  }

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
  bool serialize() {
    if (makedirs(serialization_dir_)) {
      // Put in the number of humans as the file name so we know how many
      // topics.
      std::string file_path =
          serialization_dir_ + "/" + std::to_string(graph_priors_.size());
      bag_.open(file_path, rosbag::bagmode::Write);
      visualizeJoints(true);
      visualizePoseGraphs(true);
      bag_.close();
      return true;
    } else {
      LOG(ERROR) << "Serialization for dynamic scene nodes failed!";
      return false;
    }
  }

  /**
   * @brief makedirs makes all directories needed to get to a given file path.
   * @param filepath: The path to the directory or file to check and make.
   * @param create_dirs: If true, makes the directories if they do not already
   * exist.
   * @return true if the directory exists or was successfully made, false
   * otherwise.
   */
  bool makedirs(const std::string& filepath, bool create_dirs = true) {
    // Ensure that the directory exists
    boost::filesystem::path dir(filepath);
    if (!(boost::filesystem::exists(dir))) {
      LOG(WARNING) << "Serialization Directory: " << filepath
                   << " Doesn't Exist...";
      if (create_dirs) {
        LOG(WARNING) << "Creating Directory: " << filepath;
        if (boost::filesystem::create_directory(dir)) {
          LOG(WARNING) << "...Successfully Created !";
          return true;
        } else {
          LOG(ERROR) << "...Failed! Aborting Serialization. Please change in "
                        "launch file!";
          return false;
        }
      } else {
        return false;
      }
    }
    return true;
  }

  inline void colorPclFromJoints(DynamicSceneNode& scene_node) {
    ColorPointCloud node_pcl;
    scene_node.attributes_.pcl_ = node_pcl.makeShared();
    for (size_t idx = 0; idx < NUM_JOINTS; idx++) {
      ColorPoint colorpoint(
          human_color_[0] * 255, human_color_[1] * 255, human_color_[2] * 255);
      colorpoint.x = (float)scene_node.joints_(idx, 0);
      colorpoint.y = (float)scene_node.joints_(idx, 1);
      colorpoint.z = (float)scene_node.joints_(idx, 2);
      scene_node.attributes_.pcl_->points.push_back(colorpoint);
    }
  }

  void addSceneNodeToPoseGraphs(DynamicSceneNode& scene_node);

  inline float calcNodeDistance(const DynamicSceneNode& scene_node,
                                const DynamicSceneNode& last_node) {
    Eigen::Vector3f diff = scene_node.attributes_.position_.getArray3fMap() -
                           last_node.attributes_.position_.getArray3fMap();
    return diff.norm();
  }

  inline bool checkCloseness(const DynamicSceneNode& scene_node,
                             const DynamicSceneNode& last_node) {
    Eigen::Vector3f diff = scene_node.attributes_.position_.getArray3fMap() -
                           last_node.attributes_.position_.getArray3fMap();
    double distance = diff.norm();
    return !merge_close_ ||
           distance > min_node_dist_;  // Fail if the node is too close
  }

  inline void setupEdgeMarker(visualization_msgs::Marker& edges,
                              std::vector<float>& color);

  void drawEdgesBetweenSkeletons(const DynamicSceneNode& start,
                                 const DynamicSceneNode& end,
                                 visualization_msgs::Marker& out_edges);

  void drawSkeletonEdges(DynamicSceneNode& node,
                         visualization_msgs::Marker& marker);

 public:
  // ROS Params
  double feasible_dyn_rate_ = 2.0;
  double feasible_mesh_rate_ = 1.5;
  double time_cap_ = 2.0;
  double min_node_dist_ = 0.3;
  std::vector<float> human_color_ = {0.0, 1.0, 0.0};
  std::vector<float> edge_color_ = {0.0, 0.0, 1.0};
  std::vector<float> centroid_color_ = {0.0, 0.0, 1.0};
  int prune_threshold_ = 3;
  double pgo_trans_threshold_ = 0.05;
  double pgo_rot_threshold_ = 0.005;
  double beta_diff_threshold_;
  bool single_sequence_smpl_mode_ = false;
  bool draw_skeleton_edges_ = false;
  bool merge_close_ = true;
  std::string serialization_dir_ = ".";
  std::string human_topic_;
  bool filter_centroid_ = true;
  bool filter_mesh_ = true;
  int centroid_joint_idx_;

  // Constant joint lookup table
  // TODO(marcus): check consistency with GraphCMR
  const std::vector<int> joint_parents_ =
      {1, 2, 8, 9, 3, 4, 7, 8, 12, 12, 9, 10, 14, -1, 13, -1, -1, 15, 16};

  // Record Keeping for pose graphs.
  std::vector<std::unique_ptr<KimeraRPGO::RobustSolver>> pgos_;
  KimeraRPGO::RobustSolverParams rpgo_params_;
  DynamicNodeList last_poses_;
  DynamicNodeList optimized_poses_;
  std::vector<size_t> counts_;
  std::vector<gtsam::Values> graph_priors_;

  // Noise models + params
  gtsam::SharedNoiseModel detection_noise_model_;
  gtsam::SharedNoiseModel motion_noise_model_;
  double detection_position_variance_ = 0.01;
  double detection_rotation_variance_ = 0.1;
  double motion_position_variance_ = 0.5;
  double motion_rotation_variance_ = 1.0;

 protected:
  ros::Subscriber human_sub_;
  ros::Publisher mesh_pub_;
  ros::Publisher edges_pub_;

  ros::ServiceServer serialize_graph_srv_;
  ros::ServiceServer deserialize_graph_srv_;
  SemanticRosPublishers<AgentId, ColorPointCloud> skeleton_points_pubs_;
  SemanticRosPublishers<AgentId, visualization_msgs::Marker>
      skeleton_edge_pubs_;

  SemanticRosPublishers<AgentId, ColorPointCloud> agent_centroids_;
  SemanticRosPublishers<AgentId, visualization_msgs::Marker> agent_graph_edges_;

  tf::TransformBroadcaster br_;

  // Serializers for the visualization data.
  rosbag::Bag bag_;
  std::string joint_prefix_ = "joints_";
  std::string joint_edges_prefix_ = "joint_edges_";
  std::string graph_edges_prefix_ = "graph_edges_";
  std::string graph_nodes_prefix_ = "graph_nodes_";

  float edge_thickness_ = 0.02;
};

}  // namespace kimera
