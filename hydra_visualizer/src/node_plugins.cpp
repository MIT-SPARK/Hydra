#include "hydra_visualizer/node_plugins.h"

#include <config_utilities/config.h>
#include <config_utilities/factory.h>
#include <config_utilities_ros/ros_dynamic_config_server.h>

namespace hydra {
namespace {

const auto registration =
    config::RegistrationWithConfig<NodePlugin,
                                   DynamicConfigServer,
                                   DynamicConfigServer::Config,
                                   ianvs::NodeHandle>("DynamicConfigServer");

}

void declare_config(DynamicConfigServer::Config&) {
  config::name("DynamicConfigServer::Config");
}

DynamicConfigServer::DynamicConfigServer(const Config&, ianvs::NodeHandle nh)
    : server_(std::make_unique<config::RosDynamicConfigServer>(nh.node())) {}

DynamicConfigServer::~DynamicConfigServer() = default;

}  // namespace hydra
