#pragma once

#include <actionlib/server/simple_action_server.h>
#include <geometry_msgs/Point32.h>
#include <object_db/ObjectRegistrationAction.h>
#include <object_db/common.h>
#include <object_db/matcher.h>
#include <object_db/object_registration_server.h>
#include <object_db/ply_io.h>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/geometry.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <teaser/registration.h>

#include <map>
#include <vector>

namespace object_registration {

/**
 * Server for solving registration with TEASER++
 */
class ObjectRegistrationServer {
 protected:
  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<object_db::ObjectRegistrationAction> as_;
  std::string action_name_;

  object_db::ObjectRegistrationFeedback feedback_;
  object_db::ObjectRegistrationResult result_;

  teaser::RobustRegistrationSolver::Params solver_params_;

  // Keypoints matcher
  KeypointsMatcher matcher_;

  // Object database
  std::unordered_map<int, Object> object_db_;

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

 public:
  explicit ObjectRegistrationServer(const std::string name)
      : as_(nh_,
            name,
            boost::bind(&ObjectRegistrationServer::executeCB, this, _1),
            false),
        action_name_(name) {
    as_.start();
  }

  ObjectRegistrationServer(
      const std::string name,
      teaser::RobustRegistrationSolver::Params solver_params)
      : as_(nh_,
            name,
            boost::bind(&ObjectRegistrationServer::executeCB, this, _1),
            false),
        action_name_(name),
        solver_params_(solver_params) {
    as_.start();
  }

  ObjectRegistrationServer(
      const std::string name,
      const std::string db_path,
      teaser::RobustRegistrationSolver::Params solver_params)
      : as_(nh_,
            name,
            boost::bind(&ObjectRegistrationServer::executeCB, this, _1),
            false),
        action_name_(name),
        solver_params_(solver_params) {
    as_.start();
    ROS_INFO("Loading object database.");
    loadObjectDB(db_path);
  }

  ObjectRegistrationServer(
      const std::string name,
      const std::string db_path,
      teaser::RobustRegistrationSolver::Params solver_params,
      MatcherParams matcher_params)
      : as_(nh_,
            name,
            boost::bind(&ObjectRegistrationServer::executeCB, this, _1),
            false),
        action_name_(name),
        solver_params_(solver_params),
        matcher_(matcher_params) {
    as_.start();
    ROS_INFO("Loading object database.");
    loadObjectDB(db_path);
  }

  ObjectRegistrationServer(
      const std::string name,
      const std::string db_path,
      const std::string gt_path,
      teaser::RobustRegistrationSolver::Params solver_params,
      MatcherParams matcher_params)
      : as_(nh_,
            name,
            boost::bind(&ObjectRegistrationServer::executeCB, this, _1),
            false),
        action_name_(name),
        solver_params_(solver_params),
        matcher_(matcher_params) {
    as_.start();
    ROS_INFO("Loading object database.");
    loadObjectDB(db_path);
    loadGTDB(gt_path);
  }

  ObjectRegistrationServer(
      const std::string& name,
      const std::string& db_path,
      const std::string& gt_path,
      const std::string& label_path,
      teaser::RobustRegistrationSolver::Params solver_params,
      MatcherParams matcher_params)
      : as_(nh_,
            name,
            boost::bind(&ObjectRegistrationServer::executeCB, this, _1),
            false),
        action_name_(name),
        solver_params_(solver_params),
        matcher_(matcher_params) {
    as_.start();
    ROS_INFO("Loading label database.");
    loadLabelDB(label_path);
    ROS_INFO("Loading object database.");
    loadObjectDB(db_path);
    ROS_INFO("Loading GT database.");
    loadGTDB(gt_path);
  }

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
};

}  // namespace object_registration
