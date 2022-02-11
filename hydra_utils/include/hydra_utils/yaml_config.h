#pragma once
#include <yaml-cpp/yaml.h>

namespace config_parser {

class YamlParser {
 public:
  explicit YamlParser(const std::string& file);

  explicit YamlParser(const YAML::Node& node);

  YamlParser operator[](const std::string& new_name) const;

  template <typename T>
  void visit(T& value) const {
    if (!node_) {
      return;
    }

    visitImpl(value);
  }

 private:
  template <typename T>
  void visitImpl(T& value) const {
    value = node_.as<T>();
  }

  void visitImpl(uint8_t& value) const;

  YAML::Node node_;
};

}  // namespace config_parser
