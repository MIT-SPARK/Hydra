#include "hydra_multi_ros/interface/ros_unit_interface.h"

#include <config_utilities/config.h>
#include <glog/logging.h>
#include <hydra_multi/common/multi_global_info.h>
#include <pose_graph_tools_ros/conversions.h>

#include "hydra_multi_ros/common.h"

namespace hydra_multi {

void declare_config(RosUnitInterface::Config& config) {
  using namespace config;
  name("RosUnitInterface::Config");
  base<UnitInterface::Config>(config);
  field(config.rate, "rate");
  field(config.robot_frame, "robot_frame");
}

RosUnitInterface::RosUnitInterface(const Config& config)
    : UnitInterface(config),
      config(config),
      nh_(getHydraMultiNodeHandle(config.robot_name)),
      tf_broadcaster_(nh_.node()) {}

RosUnitInterface::~RosUnitInterface() {}

void RosUnitInterface::init() { UnitInterface::init(); }

void RosUnitInterface::start() {
  UnitInterface::start();
  spin_thread_.reset(new std::thread(&RosUnitInterface::spin, this));
}

void RosUnitInterface::spin() {
  rclcpp::WallRate rate(config.rate);

  while (rclcpp::ok() && !should_shutdown_) {
    spinOnce();
    publishTf();
    rate.sleep();
  }
}

void RosUnitInterface::stop() {
  UnitInterface::stop();

  if (spin_thread_) {
    spin_thread_->join();
    spin_thread_.reset();
  }
}

void RosUnitInterface::publishTf() {
  const auto& T = state_->world_T_robot;

  geometry_msgs::msg::TransformStamped tf_msg;
  tf_msg.header.stamp = nh_.now();
  tf_msg.header.frame_id = MultiGlobalInfo::instance().getWorldFrame();
  tf_msg.child_frame_id = config.robot_frame;

  tf_msg.transform.translation.x = T.translation().x();
  tf_msg.transform.translation.y = T.translation().y();
  tf_msg.transform.translation.z = T.translation().z();

  const auto q = T.rotation().toQuaternion();
  tf_msg.transform.rotation.x = q.x();
  tf_msg.transform.rotation.y = q.y();
  tf_msg.transform.rotation.z = q.z();
  tf_msg.transform.rotation.w = q.w();

  tf_broadcaster_.sendTransform(tf_msg);
}
}  // namespace hydra_multi
