#include "hydra_utils/ros_config.h"

namespace config_parser {

RosParser::RosParser(const ros::NodeHandle& nh, const std::string& name)
    : nh_(nh), name_(name) {}

RosParser::RosParser(const ros::NodeHandle& nh) : RosParser(nh, "") {}

RosParser RosParser::FromNs(const std::string& ns) {
  return RosParser(ros::NodeHandle(ns));
}

RosParser RosParser::operator[](const std::string& new_name) const {
  // push name onto nodehandle namespace if name isn't empty
  ros::NodeHandle new_nh = (name_ != "") ? nh_ : ros::NodeHandle(nh_, name_);
  return RosParser(new_nh, new_name);
}

}  // namespace config_parser
