#include <hydra_utils/yaml_config.h>

namespace config_parser {

YamlParser::YamlParser(const YAML::Node& node, const std::string& name)
    : node_(node), name_(name) {}

YamlParser::YamlParser(const std::string& file)
    : YamlParser(YAML::LoadFile(file), "") {}

YamlParser::YamlParser(const YAML::Node& node) : YamlParser(node, "") {}

YamlParser YamlParser::operator[](const std::string& child_name) const {
  auto new_name = name_ + "/" + child_name;
  return YamlParser(node_[child_name], new_name);
}

void YamlParser::parseImpl(uint8_t& value) const { value = node_.as<uint16_t>(); }

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
