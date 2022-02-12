#pragma once
#include <hydra_utils/config_display.h>
#include <hydra_utils/config_parser.h>
#include <hydra_utils/ros_config.h>
#include <hydra_utils/yaml_config.h>

namespace config_parser {

template <>
struct is_parser<YamlParser> : public std::true_type {};

template <>
struct is_parser<RosParser> : public std::true_type {};

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

inline std::string to_uppercase(const std::string& original) {
  std::string result = original;
  std::transform(result.begin(), result.end(), result.begin(), [](unsigned char c) {
    return std::toupper(c);
  });
  return result;
}

}  // namespace config_parser

// must be used within the namespace of the config
#define DECLARE_CONFIG_OSTREAM_OPERATOR(name)                        \
  inline std::ostream& operator<<(std::ostream& out, name& config) { \
    config_parser::ConfigDisplay visitor(out);                       \
    config_parser::visit_config(visitor, config);                    \
    return out;                                                      \
  }
