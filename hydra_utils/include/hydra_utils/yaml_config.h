#pragma once
#include "hydra_utils/config_visitor.h"

#include <yaml-cpp/yaml.h>

namespace config_parser {

class YamlParser {
 public:
  explicit YamlParser(const std::string& file);

  explicit YamlParser(const YAML::Node& node);

  YamlParser operator[](const std::string& new_name) const;

  template <typename T>
  void visit(const std::string& name, T& value) const {
    auto new_parser = this->operator[](name);
    ConfigVisitor<T>::visit_config(new_parser, value);
  }

 template <typename T, typename C>
  void visit(const std::string& name, T& value, const C& converter) const {
    auto new_parser = this->operator[](name);
    auto intermediate_value = converter.from(value);
    ConfigVisitor<T>::visit_config(new_parser, intermediate_value);
    value = converter.from(intermediate_value);
  }

  template <typename T>
  void parse(T& value) const {
    if (!node_) {
      return;
    }

    parseImpl(value);
  }

 private:
  template <typename T>
  void parseImpl(T& value) const {
    value = node_.as<T>();
  }

  void parseImpl(uint8_t& value) const;

  YAML::Node node_;
};

}  // namespace config_parser
