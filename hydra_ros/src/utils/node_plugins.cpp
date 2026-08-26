#include "hydra_ros/utils/node_plugins.h"

#include <config_utilities/config.h>
#include <config_utilities/factory.h>
#include <config_utilities_ros/ros_dynamic_config_server.h>
#include <ianvs/node_handle.h>

namespace hydra {
namespace {

const auto config_server_registration =
    config::RegistrationWithConfig<AppPlugin,
                                   ConfigServerPlugin,
                                   ConfigServerPlugin::Config>("ConfigServerPlugin");

const auto spin_registration =
    config::RegistrationWithConfig<AppPlugin, SpinPlugin, SpinPlugin::Config>(
        "SpinPlugin");

}  // namespace

void declare_config(ConfigServerPlugin::Config&) {
  config::name("ConfigServerPlugin::Config");
}

ConfigServerPlugin::ConfigServerPlugin(const Config&)
    : server_(std::make_unique<config::RosDynamicConfigServer>(
          ianvs::NodeHandle::this_node().node())) {}

ConfigServerPlugin::~ConfigServerPlugin() = default;

void declare_config(SpinPlugin::Config& config) {
  using namespace config;
  name("SpinPlugin::Config");
  field(config.multi_thread, "multi_thread");
  field(config.spin_rate, "spin_rate");
}

SpinPlugin::SpinPlugin(const Config& config)
    : config(config), should_shutdown_(false), spin_thread_([this]() { spin(); }) {}

SpinPlugin::~SpinPlugin() {
  should_shutdown_ = true;
  spin_thread_.join();
}

void SpinPlugin::spin() {
  std::unique_ptr<rclcpp::Executor> executor;
  if (config.multi_thread) {
    executor = std::make_unique<rclcpp::executors::MultiThreadedExecutor>();
  } else {
    executor = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
  }

  rclcpp::WallRate r(50);
  auto nh = ianvs::NodeHandle::this_node();
  auto base = nh.node().get<rclcpp::node_interfaces::NodeBaseInterface>();
  while (rclcpp::ok() && !should_shutdown_) {
    executor->spin_node_some(base);
    r.sleep();
  }
}

}  // namespace hydra
