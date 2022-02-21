#include <hydra_utils/yaml_parser.h>

namespace config_parser {

YamlParserImpl::YamlParserImpl(const YAML::Node& node, const std::string& name)
    : node_(node), name_(name) {}

YamlParserImpl::YamlParserImpl(const std::string& file)
    : YamlParserImpl(YAML::LoadFile(file), "") {}

YamlParserImpl::YamlParserImpl(const YAML::Node& node) : YamlParserImpl(node, "") {}

YamlParserImpl YamlParserImpl::child(const std::string& child_name) const {
  auto new_name = name_ + "/" + child_name;
  return YamlParserImpl(node_[child_name], new_name);
}

void YamlParserImpl::parseImpl(uint8_t& value) const { value = node_.as<uint16_t>(); }

std::vector<std::string> YamlParserImpl::children() const {
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
