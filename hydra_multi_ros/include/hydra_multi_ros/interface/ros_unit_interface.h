#pragma once
#include <hydra_multi/interface/unit_interface.h>
#include <ianvs/node_handle.h>
#include <kimera_pgmo_ros/conversion/mesh_delta.h>
#include <pose_graph_tools_ros/conversions.h>
#include <tf2_ros/transform_broadcaster.h>

#include <hydra_msgs/msg/dsg_update.hpp>
#include <kimera_pgmo_msgs/msg/mesh_delta.hpp>
#include <rclcpp/subscription.hpp>
#include <thread>

#include "hydra_multi_ros/input/ros_dgraph_input.h"
#include "hydra_multi_ros/input/ros_dsg_input.h"
namespace hydra_multi {

class RosUnitInterface : public UnitInterface {
 public:
  struct Config : UnitInterface::Config {
    std::string ns;
    float rate = 1.0;
    std::string robot_frame;
  } const config;

  RosUnitInterface(const Config& config);

  ~RosUnitInterface() override;

 private:
  void init() override;

  void start() override;

  void spin() override;

  void stop() override;

  void publishTf();

 private:
  ianvs::NodeHandle nh_;
  std::unique_ptr<std::thread> spin_thread_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
  inline static const auto registration_ =
      config::RegistrationWithConfig<UnitInterface,
                                     RosUnitInterface,
                                     RosUnitInterface::Config>("RosUnitInterface");
};

void declare_config(RosUnitInterface::Config& config);
}  // namespace hydra_multi
