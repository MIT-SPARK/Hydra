/* -----------------------------------------------------------------------------
 * Copyright 2022 Massachusetts Institute of Technology.
 * All Rights Reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  1. Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *
 *  2. Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Research was sponsored by the United States Air Force Research Laboratory and
 * the United States Air Force Artificial Intelligence Accelerator and was
 * accomplished under Cooperative Agreement Number FA8750-19-2-1000. The views
 * and conclusions contained in this document are those of the authors and should
 * not be interpreted as representing the official policies, either expressed or
 * implied, of the United States Air Force or the U.S. Government. The U.S.
 * Government is authorized to reproduce and distribute reprints for Government
 * purposes notwithstanding any copyright notation herein.
 * -------------------------------------------------------------------------- */
#pragma once
#include "hydra_utils/ostream_formatter.h"
#include "hydra_utils/ros_parser.h"
#include "hydra_utils/yaml_parser.h"

namespace config_parser {

template <typename Config>
Config load_from_yaml(const std::string& filepath, Logger::Ptr logger = nullptr) {
  YamlParser parser(std::make_unique<YamlParserImpl>(filepath));
  if (logger) {
    parser.setLogger(logger);
  }

  Config config;
  ConfigVisitor<Config>::visit_config(parser, config);
  return config;
}

template <typename Config>
Config load_from_ros(const std::string& ns, Logger::Ptr logger = nullptr) {
  RosParser parser(std::make_unique<RosParserImpl>(ros::NodeHandle(ns)));
  if (logger) {
    parser.setLogger(logger);
  }

  Config config;
  ConfigVisitor<Config>::visit_config(parser, config);
  return config;
}

template <typename Config>
Config load_from_ros_nh(const ros::NodeHandle& nh,
                        const std::string& ns = "",
                        Logger::Ptr logger = nullptr) {
  RosParser parser(std::make_unique<RosParserImpl>(nh, ns));
  if (logger) {
    parser.setLogger(logger);
  }

  Config config;
  ConfigVisitor<Config>::visit_config(parser, config);
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
#define DECLARE_CONFIG_OSTREAM_OPERATOR(ns, name)                        \
  template <>                                                            \
  struct config_parser::is_config<ns::name> : std::true_type {};         \
                                                                         \
  namespace ns {                                                         \
  inline std::ostream& operator<<(std::ostream& out, name& config) {     \
    auto impl = std::make_unique<config_parser::OstreamFormatImpl>(out); \
    config_parser::OstreamFormatter visitor(std::move(impl));            \
    config_parser::visit_config(visitor, config);                        \
    return out;                                                          \
  }                                                                      \
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
  inline bool readRosParam(const ros::NodeHandle& nh,                            \
                           const std::string& name,                              \
                           enum_name& value) {                                   \
    std::string value_str = "";                                                  \
    if (!nh.getParam(name, value_str)) {                                         \
      return false;                                                              \
    }                                                                            \
    value = read##enum_name##FromString(value_str);                              \
    return true;                                                                 \
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
