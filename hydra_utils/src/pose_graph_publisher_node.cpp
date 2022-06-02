/* -----------------------------------------------------------------------------
 * Copyright 2022 Massachusetts Institute of Technology.
 * All Rights Reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  1. Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *
 *  2. Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Research was sponsored by the United States Air Force Research Laboratory and
 * the United States Air Force Artificial Intelligence Accelerator and was
 * accomplished under Cooperative Agreement Number FA8750-19-2-1000. The views
 * and conclusions contained in this document are those of the authors and should
 * not be interpreted as representing the official policies, either expressed or
 * implied, of the United States Air Force or the U.S. Government. The U.S.
 * Government is authorized to reproduce and distribute reprints for Government
 * purposes notwithstanding any copyright notation herein.
 * -------------------------------------------------------------------------- */
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TransformStamped.h>
#include <gtsam/geometry/Pose3.h>
#include <pose_graph_tools/PoseGraph.h>
#include <ros/ros.h>
#include <std_msgs/Time.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_listener.h>

gtsam::Pose3 getPose(const geometry_msgs::Pose& pose) {
  Eigen::Vector3d world_P_body;
  Eigen::Quaterniond world_R_body;
  tf2::convert(pose.position, world_P_body);
  tf2::convert(pose.orientation, world_R_body);
  return gtsam::Pose3(gtsam::Rot3(world_R_body), world_P_body);
}

void fillPose(const gtsam::Pose3& pose, geometry_msgs::Pose& msg) {
  Eigen::Vector3d position = pose.translation();
  Eigen::Quaterniond rotation = Eigen::Quaterniond(pose.rotation().matrix());
  tf2::convert(position, msg.position);
  tf2::convert(rotation, msg.orientation);
}

void fillBetweenPose(const geometry_msgs::Pose prev_pose,
                     const geometry_msgs::Pose curr_pose,
                     pose_graph_tools::PoseGraphEdge& edge) {
  gtsam::Pose3 world_T_body_i = getPose(prev_pose);
  gtsam::Pose3 world_T_body_j = getPose(curr_pose);
  gtsam::Pose3 body_i_T_body_j = world_T_body_i.between(world_T_body_j);

  fillPose(body_i_T_body_j, edge.pose);
}

struct PoseGraphPublisherNode {
  PoseGraphPublisherNode(ros::NodeHandle nh)
      : num_poses(0),
        world_frame("world"),
        robot_frame("base_link"),
        robot_id(0),
        use_pointcloud_time(false) {
    double keyframe_period_s = 0.2;
    nh.getParam("keyframe_period_s", keyframe_period_s);
    nh.getParam("world_frame", world_frame);
    nh.getParam("robot_frame", robot_frame);
    nh.getParam("robot_id", robot_id);
    nh.getParam("use_pointcloud_time", use_pointcloud_time);

    pg_pub = nh.advertise<pose_graph_tools::PoseGraph>("pose_graph", 10, true);
    ROS_WARN("PoseGraphPublisher waiting for subscriber...");
    ros::WallRate r(10);
    while (ros::ok() && !pg_pub.getNumSubscribers()) {
      r.sleep();
    }

    tf_listener.reset(new tf2_ros::TransformListener(buffer));

    ROS_WARN_STREAM("PoseGraphPublisher waiting for transform: "
                    << robot_frame << " -> " << world_frame);
    while (ros::ok()) {
      try {
        buffer.lookupTransform(world_frame, robot_frame, ros::Time(0));
        break;
      } catch (const tf2::TransformException& ex) {
        r.sleep();
        continue;
      }
    }

    if (!use_pointcloud_time) {
      ROS_WARN_STREAM("using timer!");
      ros::Duration loop_dur(keyframe_period_s);
      timer = nh.createTimer(loop_dur, &PoseGraphPublisherNode::timerCallback, this);
    } else {
      ROS_WARN_STREAM("using pointcloud time!");
      time_sub =
          nh.subscribe("time_point", 10, &PoseGraphPublisherNode::timeCallback, this);
    }
  }

  inline void spin() { ros::spin(); }

  void timerCallback(const ros::TimerEvent&) { publishNewPose(ros::Time::now(), true); }

  void timeCallback(const std_msgs::Time& msg) { publishNewPose(msg.data, false); }

  void publishNewPose(const ros::Time& time_to_use, bool use_latest) {
    geometry_msgs::TransformStamped transform;
    try {
      transform = buffer.lookupTransform(
          world_frame, robot_frame, use_latest ? ros::Time(0) : time_to_use);
    } catch (const tf2::TransformException& ex) {
      ROS_WARN_STREAM(ex.what());
      return;
    }

    geometry_msgs::Pose curr_pose;
    curr_pose.position.x = transform.transform.translation.x;
    curr_pose.position.y = transform.transform.translation.y;
    curr_pose.position.z = transform.transform.translation.z;
    curr_pose.orientation = transform.transform.rotation;

    if (num_poses > 0) {
      ROS_INFO_STREAM(" Publishing edge " << num_poses - 1 << " -> " << num_poses);
      pose_graph_tools::PoseGraph pose_graph;
      pose_graph.header.stamp = time_to_use;
      pose_graph.header.frame_id = world_frame;

      pose_graph_tools::PoseGraphNode prev_node;
      prev_node.header.stamp = prev_time;
      prev_node.header.frame_id = world_frame;
      prev_node.robot_id = robot_id;
      prev_node.key = num_poses - 1;
      prev_node.pose = prev_pose;

      pose_graph.nodes.push_back(prev_node);

      pose_graph_tools::PoseGraphNode curr_node;
      curr_node.header.stamp = time_to_use;
      curr_node.header.frame_id = world_frame;
      curr_node.robot_id = robot_id;
      curr_node.key = num_poses;
      curr_node.pose = curr_pose;

      pose_graph.nodes.push_back(curr_node);

      pose_graph_tools::PoseGraphEdge edge;
      edge.header.stamp = time_to_use;
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
    ++num_poses;
  }

  size_t num_poses;
  std::string world_frame;
  std::string robot_frame;
  int robot_id;
  bool use_pointcloud_time;

  ros::Time prev_time;
  geometry_msgs::Pose prev_pose;

  ros::Timer timer;
  ros::Publisher pg_pub;
  ros::Subscriber time_sub;
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
