#pragma once
#include <hydra_multi/interface/unit_interface.h>
#include <ianvs/node_handle.h>
#include <tf2_ros/transform_broadcaster.h>

#include <thread>

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
};

void declare_config(RosUnitInterface::Config& config);

}  // namespace hydra_multi
