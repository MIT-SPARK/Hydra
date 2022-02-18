#include "hydra_utils/ros_config.h"

#include <iostream>

namespace config_parser {

RosParser::RosParser(const ros::NodeHandle& nh, const std::string& name)
    : nh_(nh), name_(name) {}

RosParser::RosParser(const ros::NodeHandle& nh) : RosParser(nh, "") {}

RosParser::RosParser() : RosParser(ros::NodeHandle(), "") {}

RosParser RosParser::FromNs(const std::string& ns) {
  return RosParser(ros::NodeHandle(ns));
}

RosParser RosParser::operator[](const std::string& new_name) const {
  // push name onto nodehandle namespace if name isn't empty
  ros::NodeHandle new_nh = (name_ == "") ? nh_ : ros::NodeHandle(nh_, name_);
  return RosParser(new_nh, new_name);
}

std::vector<std::string> RosParser::children() const {
  const std::string resolved_name = nh_.resolveName(name_);
  if (resolved_name == "") {
    return {};
  }

  XmlRpc::XmlRpcValue value;
  nh_.getParam(name_, value);
  if (value.getType() != XmlRpc::XmlRpcValue::Type::TypeStruct) {
    return {};
  }

  std::vector<std::string> children;
  for (const auto& nv_pair : value) {
    children.push_back(nv_pair.first);
  }

  return children;
}

}  // namespace config_parser
