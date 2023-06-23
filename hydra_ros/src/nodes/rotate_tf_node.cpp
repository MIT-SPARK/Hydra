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
#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_broadcaster.h>

void send_tf(tf2_ros::TransformBroadcaster& br,
             const std::string& parent,
             const std::string& child,
             double radius,
             const std::vector<double>& centroid,
             double ratio) {
  const double theta = ratio * 2 * M_PI;
  const double phi = theta + M_PI;
  Eigen::Quaterniond q(std::cos(phi / 2.0), 0.0, 0.0, std::sin(phi / 2.0));

  geometry_msgs::TransformStamped msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = parent;
  msg.child_frame_id = child;
  msg.transform.translation.x = centroid[0] + radius * std::cos(theta);
  msg.transform.translation.y = centroid[1] + radius * std::sin(theta);
  msg.transform.translation.z = centroid[2];
  tf2::convert(q, msg.transform.rotation);

  br.sendTransform(msg);
}

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "rotate_tf_node");
  ros::NodeHandle nh;

  std::string parent_frame = "world";
  ros::param::get("~parent_frame", parent_frame);

  std::string child_frame = "rotated_view";
  ros::param::get("~child_frame", child_frame);

  double period_s = 5.0;
  ros::param::get("~period_s", period_s);

  bool use_wall_time = true;
  ros::param::get("~use_wall_time", use_wall_time);

  double radius = 50.0;
  ros::param::get("~radius", radius);

  std::vector<double> centroid{0.0, 0.0, 0.0};
  ros::param::get("~centroid", centroid);
  if (centroid.size() != 3) {
    ROS_FATAL_STREAM("Invalid centroid size: " << centroid.size() << " != 3");
    return 1;
  }

  tf2_ros::TransformBroadcaster br;

  const double timer_period_s = 0.01;

  if (use_wall_time) {
    ros::WallTime start_time = ros::WallTime::now();
    ros::WallTimer timer = nh.createWallTimer(
        ros::WallDuration(timer_period_s), [&](const ros::WallTimerEvent& event) {
          const double elapsed_s = (event.current_real - start_time).toSec();
          const double ratio = elapsed_s / period_s;
          send_tf(br, parent_frame, child_frame, radius, centroid, ratio);
        });
    ros::spin();
  } else {
    ros::Time start_time = ros::Time::now();
    ros::Timer timer = nh.createTimer(
        ros::Duration(timer_period_s), [&](const ros::TimerEvent& event) {
          const double elapsed_s = (event.current_real - start_time).toSec();
          const double ratio = elapsed_s / period_s;
          send_tf(br, parent_frame, child_frame, radius, centroid, ratio);
        });

    ros::spin();
  }

  return 0;
}
