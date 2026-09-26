#include <config_utilities/config_utilities.h>
#include <config_utilities/external_registry.h>
#include <config_utilities/formatting/asl.h>
#include <config_utilities/logging/log_to_glog.h>
#include <config_utilities/parsing/context.h>
#include <config_utilities/types/path.h>
#include <glog/logging.h>
#include <ianvs/node_handle_factory.h>
#include <ianvs/spin_functions.h>

#include <rclcpp/node.hpp>

#include "hydra_multi_ros/multi_ros_pipeline.h"

namespace hydra_multi {

struct RunSettings {
  bool exit_after_clock = false;
  bool force_shutdown = false;
  std::vector<std::string> paths;
  int config_verbosity = 1;
  int glog_level = 0;
  int glog_verbosity = 0;
  std::filesystem::path log_path;
  hydra::DataDirectory::Config output;
};

void declare_config(RunSettings& config) {
  using namespace config;
  name("RunSettings");
  field(config.exit_after_clock, "exit_after_clock");
  field(config.force_shutdown, "force_shutdown");
  field(config.paths, "paths");
  field(config.config_verbosity, "config_verbosity");
  field(config.glog_level, "glog_level");
  field(config.glog_verbosity, "glog_verbosity");
  field<Path::Absolute>(config.log_path, "log_path");
  field(config.output, "output");
}

}  // namespace hydra_multi

int main(int argc, char* argv[]) {
  config::initContext(argc, argv, true);
  config::setConfigSettingsFromContext();
  rclcpp::init(argc, argv);

  const auto settings = config::fromContext<hydra_multi::RunSettings>();

  FLAGS_minloglevel = settings.glog_level;
  FLAGS_v = settings.glog_verbosity;
  FLAGS_logtostderr = 1;
  FLAGS_colorlogtostderr = 1;

  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();

  config::Settings().setLogger("glog");

  auto node = std::make_shared<rclcpp::Node>("hydra_multi_ros_node");
  ianvs::NodeHandle nh(*node);
  ianvs::NodeHandleFactory::addNode("hydra_multi_ros_node", *node);
  hydra_multi::MultiGlobalInfo::instance().setForceShutdown(settings.force_shutdown);
  {  // start hydra_multi scope
    hydra_multi::MultiRosPipeline hydra_multi(settings.config_verbosity);
    hydra_multi.init();
    hydra_multi.start();
    ianvs::spinAndWait(nh, settings.exit_after_clock);
    hydra_multi.stop();
    hydra_multi.save(hydra::DataDirectory(settings.log_path, settings.output));
  }

  return 0;
}
