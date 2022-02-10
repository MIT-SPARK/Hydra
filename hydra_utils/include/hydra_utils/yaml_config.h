#pragma once
#include <yaml-cpp/yaml.h>

namespace config_parser {

class YamlParser {
 public:
  explicit YamlParser(const std::string& file) : node_(YAML::LoadFile(file)) {}

  explicit YamlParser(const YAML::Node& node) : node_(node) {}

  YamlParser operator[](const std::string& new_name) const {
    return YamlParser(node_[new_name]);
  }

  template <typename T>
  void visit(T& value) const {
    if (!node_) {
      return;
    }

    value = node_.as<T>();
  }

  void visit(uint8_t& value) const {
    if (!node_) {
      return;
    }

    value = node_.as<uint16_t>();
  }

 private:
  YAML::Node node_;
};

}  // namespace config_parser
