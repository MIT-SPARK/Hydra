#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TransformStamped.h>
#include <gtsam/geometry/Pose3.h>
#include <pose_graph_tools/PoseGraph.h>
#include <ros/ros.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_listener.h>

void fillBetweenPose(const geometry_msgs::Pose prev_pose,
                     const geometry_msgs::Pose curr_pose,
                     pose_graph_tools::PoseGraphEdge& edge) {
  Eigen::Vector3d world_P_body_i;
  Eigen::Quaterniond world_R_body_i;
  tf2::convert(prev_pose.position, world_P_body_i);
  tf2::convert(prev_pose.orientation, world_R_body_i);

  Eigen::Vector3d world_P_body_j;
  Eigen::Quaterniond world_R_body_j;
  tf2::convert(curr_pose.position, world_P_body_j);
  tf2::convert(curr_pose.orientation, world_R_body_j);

  gtsam::Pose3 world_T_body_i(gtsam::Rot3(world_R_body_i.toRotationMatrix()),
                              world_P_body_i);
  gtsam::Pose3 world_T_body_j(gtsam::Rot3(world_R_body_j.toRotationMatrix()),
                              world_P_body_j);
  gtsam::Pose3 body_i_T_body_j = world_T_body_i.between(world_T_body_j);

  Eigen::Vector3d body_i_P_body_j = body_i_T_body_j.translation();
  Eigen::Quaterniond body_i_R_body_j =
      Eigen::Quaterniond(body_i_T_body_j.rotation().matrix());
  tf2::convert(body_i_P_body_j, edge.pose.position);
  tf2::convert(body_i_R_body_j, edge.pose.orientation);
}

struct PoseGraphPublisherNode {
  PoseGraphPublisherNode(ros::NodeHandle nh)
      : num_poses(0), world_frame("world"), robot_frame("base_link"), robot_id(0) {
    double keyframe_period_s = 0.2;
    nh.getParam("keyframe_period_s", keyframe_period_s);
    nh.getParam("world_frame", world_frame);
    nh.getParam("robot_frame", robot_frame);
    nh.getParam("robot_id", robot_id);

    pg_pub = nh.advertise<pose_graph_tools::PoseGraph>("pose_graph", 10, true);
    ROS_WARN("PoseGraphPublisher waiting for subscriber...");
    ros::Rate r(10);
    while (ros::ok() && !pg_pub.getNumSubscribers()) {
      r.sleep();
    }

    tf_listener.reset(new tf2_ros::TransformListener(buffer));
    timer = nh.createTimer(
        ros::Duration(keyframe_period_s), &PoseGraphPublisherNode::timerCallback, this);
  }

  inline void spin() { ros::spin(); }

  void timerCallback(const ros::TimerEvent&) {
    geometry_msgs::TransformStamped transform;
    try {
      transform = buffer.lookupTransform(world_frame, robot_frame, ros::Time(0));
    } catch (const tf2::TransformException& ex) {
      ROS_WARN_STREAM(ex.what());
      return;
    }

    ros::Time curr_time = transform.header.stamp;

    geometry_msgs::Pose curr_pose;
    curr_pose.position.x = transform.transform.translation.x;
    curr_pose.position.y = transform.transform.translation.y;
    curr_pose.position.z = transform.transform.translation.z;
    curr_pose.orientation = transform.transform.rotation;

    if (num_poses > 0) {
      ROS_INFO_STREAM(" Publishing edge " << num_poses - 1 << " -> " << num_poses);
      pose_graph_tools::PoseGraph pose_graph;
      pose_graph.header.stamp = curr_time;
      pose_graph.header.frame_id = world_frame;

      pose_graph_tools::PoseGraphNode prev_node;
      prev_node.header.stamp = prev_time;
      prev_node.header.frame_id = world_frame;
      prev_node.robot_id = robot_id;
      prev_node.key = num_poses - 1;
      prev_node.pose = prev_pose;

      pose_graph.nodes.push_back(prev_node);

      pose_graph_tools::PoseGraphNode curr_node;
      curr_node.header.stamp = curr_time;
      curr_node.header.frame_id = world_frame;
      curr_node.robot_id = robot_id;
      curr_node.key = num_poses;
      curr_node.pose = curr_pose;

      pose_graph.nodes.push_back(curr_node);

      pose_graph_tools::PoseGraphEdge edge;
      edge.header.stamp = curr_time;
      edge.header.frame_id = world_frame;
      edge.key_from = num_poses - 1;
      edge.key_to = num_poses;
      edge.robot_from = robot_id;
      edge.robot_to = robot_id;
      edge.type = pose_graph_tools::PoseGraphEdge::ODOM;
      fillBetweenPose(prev_pose, curr_pose, edge);
      pose_graph.edges.push_back(edge);

      pg_pub.publish(pose_graph);
    }

    prev_pose = curr_pose;
    prev_time = curr_time;
    num_poses++;
  }

  size_t num_poses;
  std::string world_frame;
  std::string robot_frame;
  int robot_id;

  ros::Time prev_time;
  geometry_msgs::Pose prev_pose;

  ros::Timer timer;
  ros::Publisher pg_pub;
  tf2_ros::Buffer buffer;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener;
};

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "pose_graph_publisher");

  ros::NodeHandle nh("~");
  PoseGraphPublisherNode node(nh);
  node.spin();

  return 0;
}
