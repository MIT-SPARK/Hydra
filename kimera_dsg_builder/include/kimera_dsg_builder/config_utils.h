#pragma once
#include <hydra_utils/config.h>
#include <hydra_utils/eigen_config_types.h>

#include <glog/logging.h>

namespace kimera {

struct DsgParamLogger : config_parser::Logger {
  inline void log_missing(const std::string& message) const override {
    LOG(INFO) << message;
  }
};

template <typename Config>
Config load_config(const ros::NodeHandle& nh, const std::string& ns = "") {
  auto logger = std::make_shared<DsgParamLogger>();
  return config_parser::load_from_ros_nh<Config>(nh, ns, logger);
}

}  // namespace kimera
