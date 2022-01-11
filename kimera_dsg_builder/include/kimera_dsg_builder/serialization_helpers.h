#pragma once
#include <kimera_dsg/serialization_helpers.h>
#include <ros/ros.h>

namespace ros {

void to_json(nlohmann::json& j, const ros::Time& time);

void from_json(const nlohmann::json& j, ros::Time& time);

}  // namespace ros
