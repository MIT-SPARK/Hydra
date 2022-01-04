#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_listener.h>

#include <fstream>
#include <iostream>

using Cloud = pcl::PointCloud<pcl::PointXYZRGBA>;

struct CloudSaverNode {
  explicit CloudSaverNode(const ros::NodeHandle& node_handle)
      : nh(node_handle),
        num_output(0),
        max_output(100),
        prefix("pointcloud_"),
        world_frame("world"),
        sensor_frame("left_cam"),
        min_separation_s(0.2),
        last_time(0) {
    tf_listener.reset(new tf2_ros::TransformListener(tf_buffer));

    if (!nh.getParam("output_path", output_path)) {
      throw std::runtime_error("missing output_path");
    }

    nh.getParam("prefix", prefix);
    nh.getParam("max_output", max_output);
    nh.getParam("min_separation_s", min_separation_s);
    nh.getParam("world_frame", world_frame);
    nh.getParam("sensor_frame", sensor_frame);

    pcl_sub = nh.subscribe("pointcloud", 1000, &CloudSaverNode::handlePointcloud, this);
  }

  geometry_msgs::Transform getTf() {
    return tf_buffer
        .lookupTransform(world_frame, sensor_frame, last_time, ros::Duration(1.0))
        .transform;
  }

  void handlePointcloud(const Cloud::ConstPtr& cloud) {
    ros::Time latest_time;
    pcl_conversions::fromPCL(cloud->header.stamp, latest_time);

    const auto diff_s = (latest_time - last_time).toSec();
    if (diff_s < min_separation_s) {
      return;
    }

    last_time = latest_time;
    auto world_T_body = getTf();
    Eigen::Quaterniond world_q_body;
    tf2::convert(world_T_body.rotation, world_q_body);
    Eigen::Matrix3d world_R_body = world_q_body.toRotationMatrix();
    Eigen::Vector3d world_t_body(world_T_body.translation.x,
                                 world_T_body.translation.y,
                                 world_T_body.translation.z);

    std::string num = std::to_string(num_output);
    if (num.size() < 3) {
      num.insert(num.begin(), 3 - num.size(), '0');
    }

    std::string filepath = output_path + "/" + prefix + num + ".csv";
    ROS_INFO_STREAM("Writing pointcloud to " << filepath);
    std::ofstream fout(filepath);
    fout << "x,y,z,r,g,b" << std::endl;

    for (const auto& point : *cloud) {
      const Eigen::Vector3d p_body(point.x, point.y, point.z);
      const Eigen::Vector3d p_world = world_R_body * p_body + world_t_body;

      fout << p_world.x() << "," << p_world.y() << "," << p_world.z() << ","
           << static_cast<int>(point.r) << "," << static_cast<int>(point.g) << ","
           << static_cast<int>(point.b) << std::endl;
    }

    num_output++;
    if (num_output >= max_output) {
      ros::shutdown();
      return;
    }
  }

  ros::NodeHandle nh;
  ros::Subscriber pcl_sub;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener;
  tf2_ros::Buffer tf_buffer;

  int num_output;
  int max_output;
  std::string output_path;
  std::string prefix;

  std::string world_frame;
  std::string sensor_frame;

  double min_separation_s;
  ros::Time last_time;
};

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "pointcloud_saver");
  ros::NodeHandle nh("~");

  CloudSaverNode node(nh);

  ros::spin();

  return 0;
}
