#include <hydra_utils/yaml_config.h>

namespace config_parser {

YamlParser::YamlParser(const std::string& file) : node_(YAML::LoadFile(file)) {}

YamlParser::YamlParser(const YAML::Node& node) : node_(node) {}

YamlParser YamlParser::operator[](const std::string& new_name) const {
  return YamlParser(node_[new_name]);
}

void YamlParser::parseImpl(uint8_t& value) const {
  if (!node_) {
    return;
  }

  value = node_.as<uint16_t>();
}

std::vector<std::string> YamlParser::children() const {
  if (!node_.IsMap()) {
    return {};
  }

  std::vector<std::string> children;
  for (const auto& kv_pair : node_) {
    children.push_back(kv_pair.first.as<std::string>());
  }

  return children;
}

}  // namespace config_parser
