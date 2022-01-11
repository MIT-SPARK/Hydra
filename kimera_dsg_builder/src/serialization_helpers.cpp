#include "kimera_dsg_builder/serialization_helpers.h"

namespace ros {

using nlohmann::json;

void to_json(json& j, const ros::Time& time) { j = json{time.toNSec()}; }

void from_json(const json& j, ros::Time& time) { time.fromNSec(j.get<uint64_t>()); }

}  // namespace ros
