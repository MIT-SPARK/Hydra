#pragma once
#include "hydra_utils/config_parser.h"

#include <yaml-cpp/yaml.h>
#include <vector>

namespace config_parser {

class YamlParserImpl {
 public:
  YamlParserImpl() = default;

  YamlParserImpl(const YamlParserImpl& other) = default;

  explicit YamlParserImpl(const std::string& file);

  explicit YamlParserImpl(const YAML::Node& node);

  YamlParserImpl& operator=(const YamlParserImpl& other) = default;

  ~YamlParserImpl() = default;

  YamlParserImpl child(const std::string& new_name) const;

  std::vector<std::string> children() const;

  inline std::string name() const { return name_; }

  template <typename T>
  bool parse(T& value) const {
    if (!node_) {
      return false;
    }

    parseImpl(value);
    return true;
  }

 private:
  YamlParserImpl(const YAML::Node& node, const std::string& name);

  template <typename T>
  void parseImpl(T& value) const {
    value = node_.as<T>();
  }

  void parseImpl(uint8_t& value) const;

  YAML::Node node_;
  std::string name_;
};

using YamlParser = Parser<YamlParserImpl>;

}  // namespace config_parser
