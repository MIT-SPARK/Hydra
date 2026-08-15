#include "hydra_ros/input/pointcloud_receiver.h"

#include <config_utilities/config.h>
#include <config_utilities/factory.h>
#include <glog/logging.h>
#include <hydra/common/global_info.h>
#include <ianvs/node_handle.h>

#include "hydra_ros/input/pointcloud_adaptor.h"

namespace hydra {
namespace {

static const auto registration_ =
    config::RegistrationWithConfig<DataReceiver,
                                   PointcloudReceiver,
                                   PointcloudReceiver::Config,
                                   std::string>("PointcloudReceiver");

}

using sensor_msgs::msg::PointCloud2;

void declare_config(PointcloudReceiver::Config& config) {
  using namespace config;
  name("PointcloudReceiver::Config");
  base<RosDataReceiver::Config>(config);
  field(config.in_world_frame, "in_world_frame");
  field(config.instance_ids, "instance_ids");
  field(config.discard_transparent_color, "discard_transparent_color");
}

PointcloudReceiver::PointcloudReceiver(const Config& config,
                                       const std::string& sensor_name)
    : RosDataReceiver(config, sensor_name), config(config) {}

bool PointcloudReceiver::initImpl() {
  auto nh = ianvs::NodeHandle::this_node(ns_);
  sub_ = nh.create_subscription<PointCloud2>(
      "pointcloud", config.queue_size, &PointcloudReceiver::callback, this);
  return true;
}

void PointcloudReceiver::callback(const PointCloud2::ConstSharedPtr& msg) {
  const auto stamp = rclcpp::Time(msg->header.stamp).nanoseconds();
  VLOG(5) << "[Hydra Reconstruction] Got raw pointcloud input @ " << stamp << " [ns]";

  if (!checkInputTimestamp(stamp)) {
    return;
  }

  auto packet = std::make_shared<CloudInputPacket>(stamp, sensor_name_);
  fillPointcloudPacket(
      *msg, *packet, config.instance_ids, config.discard_transparent_color);
  packet->in_world_frame = config.in_world_frame;
  queue.push(packet);
}

}  // namespace hydra
