#pragma once
#include "hydra_utils/config_visitor.h"

#include <yaml-cpp/yaml.h>
#include <vector>

namespace config_parser {

class YamlParser {
 public:
  YamlParser() = default;

  YamlParser(const YamlParser& other) = default;

  explicit YamlParser(const std::string& file);

  explicit YamlParser(const YAML::Node& node);

  YamlParser& operator=(const YamlParser& other) = default;

  ~YamlParser() = default;

  YamlParser operator[](const std::string& new_name) const;

  std::vector<std::string> children() const;

  inline std::string name() const { return name_; }

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
  bool parse(T& value) const {
    if (!node_) {
      return false;
    }

    parseImpl(value);
    return true;
  }

 private:
  YamlParser(const YAML::Node& node, const std::string& name);

  template <typename T>
  void parseImpl(T& value) const {
    value = node_.as<T>();
  }

  void parseImpl(uint8_t& value) const;

  YAML::Node node_;
  std::string name_;
};

}  // namespace config_parser
