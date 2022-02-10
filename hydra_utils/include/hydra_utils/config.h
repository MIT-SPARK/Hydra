#pragma once
#include <hydra_utils/config_display.h>
#include <hydra_utils/config_parser.h>
#include <hydra_utils/ros_config.h>
#include <hydra_utils/yaml_config.h>

namespace config_parser {

template <typename Config>
Config load_from_yaml(const std::string& filepath) {
  Config config;
  visit_config(YamlParser(filepath), config);
  return config;
}

template <typename Config>
Config load_from_ros(const std::string& ns) {
  Config config;
  visit_config(RosParser::FromNs(ns), config);
  return config;
}

template <typename Config>
Config load_from_ros_nh(const ros::NodeHandle& nh) {
  Config config;
  visit_config(RosParser(nh), config);
  return config;
}

template <typename T>
struct is_config : std::false_type {};

}  // namespace config_parser

template <typename Config,
          std::enable_if_t<config_parser::is_config<Config>::value, bool> = true>
std::ostream& operator<<(std::ostream& out, const Config& config) {
  config_parser::ConfigDisplay visitor(out);
  config_parser::visit_config(visitor, config);
  return out;
}
