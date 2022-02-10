#include "hydra_utils/config_parser.h"

namespace kimera {

#define READ_PARAM(nh, config, name, bounds...)                 \
  if (!parseParam(nh, #name, config.name, ##bounds)) {          \
    std::stringstream ss;                                       \
    ss << nh.resolveName(#name) + " not found! Defaulting to "; \
    outputParamDefault(ss, config.name);                        \
    ROS_DEBUG_STREAM(ss.str());                                 \
  }                                                             \
  static_assert(true, "")

}  // namespace kimera
