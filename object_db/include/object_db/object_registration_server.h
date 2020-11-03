#pragma once

#include <unordered_map>
#include <vector>

#include <pcl/common/geometry.h>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PoseStamped.h>

#include <tf2_ros/static_transform_broadcaster.h>

#include <teaser/registration.h>

#include "object_db/ObjectRegistrationAction.h"
#include "object_db/common.h"
#include "object_db/matcher.h"
#include "object_db/object_registration_server.h"
#include "object_db/ply_io.h"

namespace object_registration {

/**
 * Server for solving registration with TEASER++
 */
class ObjectRegistrationServer {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  ObjectRegistrationServer(
      const std::string& name,
      const std::string& target_object_label,
      const std::string& db_path = std::string(),
      const std::string& gt_path = std::string(),
      const std::string& label_path = std::string(),
      const teaser::RobustRegistrationSolver::Params& solver_params =
          teaser::RobustRegistrationSolver::Params(),
      const MatcherParams& matcher_params = MatcherParams());

  ~ObjectRegistrationServer() = default;

  /**
   * Call back function for the action server
   * @param goal
   */
  void executeCB(const object_db::ObjectRegistrationGoalConstPtr& goal);

  /**
   * Load a database
   * @param db_path
   */
  int loadObjectDB(const std::string& db_path);

  /**
   * Load GT data
   */
  int loadGTDB(const std::string& gt_file);

  /**
   * Load label db
   */
  int loadLabelDB(const std::string& label_file);

 protected:
  void publishObjectStaticTf(const geometry_msgs::Transform& transform,
                             const std::string& parent_frame_id,
                             const std::string& child_frame_id) {
    static tf2_ros::StaticTransformBroadcaster static_broadcaster;
    geometry_msgs::TransformStamped static_transform_stamped;
    static_transform_stamped.header.stamp = ros::Time::now();
    static_transform_stamped.header.frame_id = parent_frame_id;
    static_transform_stamped.child_frame_id = child_frame_id;
    static_transform_stamped.transform = transform;
    static_broadcaster.sendTransform(static_transform_stamped);
  }

  void dbPublishHelper(
      std::string prefix,
      std::string s_label,
      pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud,
      std::unordered_map<std::string, pcl::PointCloud<pcl::PointXYZ>>& db,
      std::unordered_map<std::string, ros::Publisher>& pubs,
      ros::NodeHandle& nh);

 protected:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  actionlib::SimpleActionServer<object_db::ObjectRegistrationAction> action_server_;
  std::string action_name_;

  object_db::ObjectRegistrationFeedback feedback_;
  object_db::ObjectRegistrationResult result_;

  teaser::RobustRegistrationSolver::Params solver_params_;

  // Frame ids
  std::string world_frame_id_;

  // Keypoints matcher
  KeypointsMatcher matcher_;

  // Object database
  std::unordered_map<int, Object> object_db_;

  // Target object label
  std::string target_object_label_;

  // GT data
  std::vector<pcl::PointXYZ> gt_centroids_;

  // GT db
  std::unordered_map<std::string, int> semantic_label_map_;
  std::unordered_map<int, std::vector<pcl::PointXYZ>> gt_centroids_db_;

  // Error vector
  std::vector<float> known_centroid_errors_;
  std::vector<float> unknown_centroid_errors_;

  // Publishers
  std::unordered_map<std::string, pcl::PointCloud<pcl::PointXYZ>>
      registrated_object_db_;
  std::unordered_map<std::string, ros::Publisher>
      registrated_object_publishers_;

  // Src keypoints
  std::unordered_map<std::string, pcl::PointCloud<pcl::PointXYZ>>
      src_keypoints_db_;
  std::unordered_map<std::string, ros::Publisher> src_keypoints_publishers_;

  // Dst keypoints
  std::unordered_map<std::string, pcl::PointCloud<pcl::PointXYZ>>
      dst_keypoints_db_;
  std::unordered_map<std::string, ros::Publisher> dst_keypoints_publishers_;
};

}  // namespace object_registration
