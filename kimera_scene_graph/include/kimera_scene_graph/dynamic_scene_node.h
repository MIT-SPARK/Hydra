#pragma once

#include <map>
#include <unordered_map>
#include <vector>
#include <Eigen/Dense>
#include <pcl_conversions/pcl_conversions.h>

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "graph_cmr_ros/SMPLList.h"
#include "graph_cmr_ros/SMPL.h"

#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/Values.h>

#include "kimera_scene_graph/scene_node.h"

namespace kimera {

typedef gtsam::NonlinearFactorGraph DynamicNodePoseGraph;
typedef gtsam::BetweenFactor<gtsam::Pose3> OdometryFactor;

const static int NUM_JOINTS = 38;
typedef Eigen::Matrix<double, NUM_JOINTS, 3, Eigen::RowMajor> JointMatrix;

struct DynamicSceneNode : SceneNode {
  gtsam::Pose3 pose_;
  JointMatrix joints_;
};

typedef std::pair<DynamicSceneNode, size_t> NodeIndex;
 
class DynamicSceneGraph : public SceneGraph {
 public:
  DynamicSceneGraph(const ros::NodeHandle& nh, 
                    const ros::NodeHandle& nh_private) 
    : SceneGraph(nh, nh_private),
      human_sub_(){
    // GTSAM noise model
    noise_model_ = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << P_NOISE_, P_NOISE_, P_NOISE_, O_NOISE_, O_NOISE_, O_NOISE_).finished());

    // Params
    nh_private_.param("world_frame", world_frame_, world_frame_);
    nh_private_.param("human_topic", human_topic_, human_topic_);
    nh_private_.param("filter_by_centroid", filter_centroid_, true);
    nh_private_.param("filter_by_mesh", filter_mesh_, true);
    nh_private_.param("feasible_dyn_rate", feasible_dyn_rate_, feasible_dyn_rate_);
    nh_private_.param("feasible_mesh_rate", feasible_mesh_rate_, feasible_mesh_rate_);
    nh_private_.param("node_association_time_cap", time_cap_, time_cap_);
    nh_private_.param("min_node_association_dist", min_node_dist_, min_node_dist_);

    // Publishers
    semantic_instance_centroid_pub_ = nh_private_.advertise<ColoredPointCloud>(
        "human_instance_centroid_pub", 1, true);
    edge_pub_ = nh_private_.advertise<visualization_msgs::Marker>(
      "human_edges_pub", 1, true);

    LOG(WARNING) << "human topic is : " << human_topic_;
    // Subscribe to human meshes
    human_sub_ = nh_private_.subscribe<graph_cmr_ros::SMPLList>(human_topic_, 10, &DynamicSceneGraph::humanCallback, this);

  };

  virtual ~DynamicSceneGraph() = default;

 public:
  void humanCallback(const graph_cmr_ros::SMPLList::ConstPtr& msg){
    for (auto mesh : msg->human_meshes){
      DynamicSceneNode human_node;
      dynamicSceneNodeFromSMPL(mesh, human_node);
      // TODO (argupta) send over joints in world frame to fill pcl field.
      addSceneNode(human_node);
    }
    visualizePoseGraphs();
  }

  void dynamicSceneNodeFromSMPL(graph_cmr_ros::SMPL& mesh, DynamicSceneNode& node) {
      node.attributes_.position_ = NodePosition(mesh.centroid[0], mesh.centroid[1], mesh.centroid[2]);
      // FIXME (argupta) add in global orientation information for human mesh;
      // human_node.attributes_.orientation_ = NodeOrientation(0, 0, 0, 1);

      // Initialize joints
      node.joints_ = Eigen::Map<JointMatrix>(mesh.joints.data());

      pcl_conversions::toPCL(mesh.header.stamp, node.attributes_.timestamp_);
      // FIXME (argupta) add in a global rotation for the pose.
      node.pose_ = gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(mesh.centroid[0], mesh.centroid[1], mesh.centroid[2]));
  }

  inline void addSceneNode(DynamicSceneNode& scene_node) {
    // Expectation that scene_node.id_ is blank, use addSceneNodetoPoseGraphs to get id 
    addSceneNodeToPoseGraphs(scene_node);
    SceneGraph::addSceneNode(scene_node);
  }

  inline bool addSemanticInstanceSafely(DynamicSceneNode& scene_node) {
    // Nodes added to the database need to have their pose graphs updated as well.
    addSceneNodeToPoseGraphs(scene_node);
    bool added_to_database = SceneGraph::addSemanticInstanceSafely(scene_node);
    if (added_to_database) {
      return true;
    } 
    
    return false;
  }

  void addSceneNodeToPoseGraphs(DynamicSceneNode& scene_node) {
    LOG(WARNING) << "Number of graphs: " << pose_graphs_.size();
    // Find the scene node that passes all time-checks
    bool node_exists = false;
    float closest_dist = 10000.0;
    size_t closest_idx = 0;
    for(int i = 0; i < pose_graphs_.size(); i++){
      auto last_node = last_poses_[i].first;
      float time_diff = (scene_node.attributes_.timestamp_ - last_node.attributes_.timestamp_) * pow(10, -6);
      float c_dist = calcNodeDistance(scene_node, last_node);
      if(time_diff < time_cap_ && checkMeshFeasibility(scene_node, last_node)){
          if (checkDynamicFeasibility(scene_node, last_node)) {
            node_exists = true;
            if(c_dist < closest_dist){
              closest_dist = c_dist;
              closest_idx = i;
            }
          }
        }
    }

    if (node_exists) {
      // Make sure the node is not too close.
      auto last_pose_pair = last_poses_[closest_idx];
      size_t last_node_key = last_pose_pair.second;
      auto last_node = last_pose_pair.first; 

      if (checkCloseness(scene_node, last_node)){
        // Identify the scene node as having the same id as the nearby id.
        scene_node.id_ = last_node.id_;

        gtsam::Pose3 odometry_measurement = last_node.pose_.between(scene_node.pose_);
        pose_graphs_[closest_idx].add(OdometryFactor(last_node_key, last_node_key+1, odometry_measurement, noise_model_));
        graph_priors_[closest_idx].insert(last_node_key+1, scene_node.pose_);

        last_poses_[closest_idx] = NodeIndex(scene_node, last_node_key+1);
      }
      else{
        last_poses_[closest_idx] = NodeIndex(scene_node, last_node_key);
      }
    }
    else {
      pose_graphs_.push_back(DynamicNodePoseGraph());
      size_t last_idx = pose_graphs_.size()-1;
      graph_priors_.push_back(gtsam::Values());
      pose_graphs_[last_idx].add(gtsam::PriorFactor<gtsam::Pose3>(
          0, scene_node.pose_, noise_model_));
      graph_priors_[last_idx].insert(0, scene_node.pose_);
      // ID of the node is the same as the graph index
      scene_node.id_ = last_idx;
      last_poses_.push_back(NodeIndex(scene_node, 0));

    }
  }

  inline float calcNodeDistance(const DynamicSceneNode& scene_node, const DynamicSceneNode& last_node) {
    Eigen::Vector3f diff = scene_node.attributes_.position_.getArray3fMap() - last_node.attributes_.position_.getArray3fMap();
    return diff.norm();
  }

  inline bool checkDynamicFeasibility(const DynamicSceneNode& scene_node, const DynamicSceneNode& last_node) {
    float distance = calcNodeDistance(scene_node, last_node);
    double time_diff = (scene_node.attributes_.timestamp_ - last_node.attributes_.timestamp_) * pow(10, -6);
    LOG(WARNING) << "checkDynamicFeasibility -- Distance: " << distance << "  Threshold: " << feasible_dyn_rate_ * time_diff;
    return !filter_centroid_ || distance <= feasible_dyn_rate_ * time_diff; // Fail if the node is too far
  }

  inline bool checkCloseness(const DynamicSceneNode& scene_node, const DynamicSceneNode& last_node) {
    Eigen::Vector3f diff = scene_node.attributes_.position_.getArray3fMap() - last_node.attributes_.position_.getArray3fMap();
    double distance = diff.norm();
    return distance > min_node_dist_; // Fail if the node is too close
  }

  inline bool checkMeshFeasibility(const DynamicSceneNode& scene_node, const DynamicSceneNode& last_node) {
    JointMatrix diff = scene_node.joints_ - last_node.joints_;
    double max_dist = diff.rowwise().norm().maxCoeff();
    double time_diff = (scene_node.attributes_.timestamp_ - last_node.attributes_.timestamp_) * pow(10, -6);
    LOG(WARNING) << "checkMeshFeasibility -- Distance: " << max_dist << "  Threshold: " << feasible_mesh_rate_ * time_diff;
    return !filter_mesh_ || max_dist <= feasible_mesh_rate_ * time_diff;
  }

  // Standard Visualization will be handled by the base class visualization function.
  void visualizePoseGraphs(){
    visualization_msgs::Marker marker;
    marker.header.frame_id = world_frame_;
    marker.header.stamp = ros::Time();

    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    // static int marker_id = 1u;
    marker.id = 1;
    marker.ns = "node_edges";

    marker.scale.x = .01;
    marker.scale.y = .01;
    marker.scale.z = .01;

    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;

    marker.pose.position.x = 0.0;
    marker.pose.position.y = 0.0;
    marker.pose.position.z = 0.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    PointCloud centroid_pointcloud;
    for (auto values : graph_priors_){
      NodePosition last_centroid;
      int iter_count = 0;
      for(auto key_value_pair : values){
        const gtsam::Point3& centroid = key_value_pair.value.cast<gtsam::Pose3>().translation();
        NodePosition pcl_centroid;
        pcl_centroid.x = float(centroid.x());
        pcl_centroid.y = float(centroid.y());
        pcl_centroid.z = float(centroid.z());
        centroid_pointcloud.push_back(pcl_centroid);

        // Make sure the last_centroid is filled in.
        if (iter_count++ > 0){
          geometry_msgs::Point last_vtx, curr_vtx;
          last_vtx.x = last_centroid.x;
          last_vtx.y = last_centroid.y;
          last_vtx.z = last_centroid.z;

          curr_vtx.x = pcl_centroid.x;
          curr_vtx.y = pcl_centroid.y;
          curr_vtx.z = pcl_centroid.z;
          marker.points.push_back(last_vtx);
          marker.points.push_back(curr_vtx);
        }
        last_centroid = pcl_centroid;
      }
    }
    LOG(WARNING) << "Number of vertices in edge marker: " << marker.points.size();
    centroid_pointcloud.header.frame_id = world_frame_;
    semantic_instance_centroid_pub_.publish(centroid_pointcloud);
    edge_pub_.publish(marker);
  }

 public:
  // Average Walking speed is 1.4 m/s
  double feasible_dyn_rate_ = 2.0;
  // Move body-length in 1 sec.
  double feasible_mesh_rate_ = 1.5;
  double time_cap_ = 2.0;
  double min_node_dist_ = 0.3;

  std::vector<DynamicNodePoseGraph> pose_graphs_;
  std::vector<NodeIndex> last_poses_;
  std::vector<gtsam::Values> graph_priors_;
  gtsam::noiseModel::Diagonal::shared_ptr noise_model_;

 protected:
  // ROS Subscriber
  // TODO (argupta) Talk to Toni to see if he would prefer this be a service?
  ros::Subscriber human_sub_;
  std::string human_topic_;
  bool filter_centroid_ = true;
  bool filter_mesh_ = true;

 private:

  // Position and Orientation noise for pose estimates
  // TODO (argupta) figure out reasonable noise parameters
  const float P_NOISE_ = 0.1;
  const float O_NOISE_ = 0.01;

};

} // namespace kimera