#include "hydra_ros/utils/config_server_plugin.h"

#include <config_utilities/config.h>
#include <config_utilities/factory.h>
#include <config_utilities_ros/ros_dynamic_config_server.h>
#include <ianvs/node_handle.h>

namespace hydra {
namespace {

const auto registration =
    config::RegistrationWithConfig<AppPlugin,
                                   ConfigServerPlugin,
                                   ConfigServerPlugin::Config>("ConfigServerPlugin");

}

void declare_config(ConfigServerPlugin::Config&) {
  config::name("ConfigServerPlugin::Config");
}

ConfigServerPlugin::ConfigServerPlugin(const Config&)
    : server_(std::make_unique<config::RosDynamicConfigServer>(
          ianvs::NodeHandle::this_node().node())) {}

ConfigServerPlugin::~ConfigServerPlugin() = default;

}  // namespace hydra
