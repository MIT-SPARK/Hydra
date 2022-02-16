#pragma once
#include "hydra_utils/config_display.h"
#include "hydra_utils/config_visitor.h"
#include "hydra_utils/ros_config.h"
#include "hydra_utils/yaml_config.h"

namespace config_parser {

template <>
struct is_parser<YamlParser> : std::true_type {};

template <>
struct is_parser<RosParser> : std::true_type {};

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

// must be called in the global namespace
#define DECLARE_CONFIG_OSTREAM_OPERATOR(ns, name)                    \
  template <>                                                        \
  struct config_parser::is_config<ns::name> : std::true_type {};     \
                                                                     \
  namespace ns {                                                     \
  inline std::ostream& operator<<(std::ostream& out, name& config) { \
    config_parser::ConfigDisplay visitor(out);                       \
    config_parser::visit_config(visitor, config);                    \
    return out;                                                      \
  }                                                                  \
  }

// must be called in the global namespace
#define DECLARE_CONFIG_ENUM(ns, enum_name, values...)                            \
  namespace ns {                                                                 \
  inline enum_name read##enum_name##FromString(const std::string& enum_string) { \
    std::map<enum_name, std::string> value_map{values};                          \
    for (const auto& kv_pair : value_map) {                                      \
      if (kv_pair.second == enum_string) {                                       \
        return kv_pair.first;                                                    \
      }                                                                          \
    }                                                                            \
    throw std::domain_error("invalid value for " #enum_name ": " + enum_string); \
  }                                                                              \
                                                                                 \
  inline std::ostream& operator<<(std::ostream& out, enum_name v) {              \
    std::map<enum_name, std::string> value_map{values};                          \
    if (value_map.count(v)) {                                                    \
      out << value_map.at(v);                                                    \
    } else {                                                                     \
      out << "INVALID";                                                          \
    }                                                                            \
    return out;                                                                  \
  }                                                                              \
                                                                                 \
  inline void readRosParam(const ros::NodeHandle& nh,                            \
                           const std::string& name,                              \
                           enum_name& value) {                                   \
    std::string value_str = "";                                                  \
    if (!nh.getParam(name, value_str)) {                                         \
      return;                                                                    \
    }                                                                            \
    value = read##enum_name##FromString(value_str);                              \
  }                                                                              \
  }                                                                              \
                                                                                 \
  template <>                                                                    \
  struct YAML::convert<ns::enum_name> {                                          \
    static ::YAML::Node encode(const ns::enum_name& rhs) {                       \
      std::stringstream ss;                                                      \
      ss << rhs;                                                                 \
      return YAML::Node(ss.str());                                               \
    }                                                                            \
                                                                                 \
    static bool decode(const Node& node, ns::enum_name& rhs) {                   \
      if (node.IsNull()) {                                                       \
        return false;                                                            \
      }                                                                          \
      rhs = ns::read##enum_name##FromString(node.as<std::string>());             \
      return true;                                                               \
    }                                                                            \
  };
