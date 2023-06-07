#include "hydra_rviz_plugin/edge_config.h"

#include <algorithm>

namespace hydra {

std::ostream& operator<<(std::ostream& out, const EdgeConfig& conf) {
  out << std::boolalpha << "{insertion_skip: " << conf.insertion_skip
      << ", visualize: " << conf.visualize << ", edge_scale: " << conf.edge_scale
      << ", edge_alpha: " << conf.edge_alpha << "}";
  return out;
}

EdgeConfig::ColorMode stringToEdgeColorMode(const std::string& input) {
  std::string result = input;
  std::transform(result.begin(), result.end(), result.begin(), [](unsigned char c) {
    return std::toupper(c);
  });

  if (result == "PARENT") {
    return EdgeConfig::ColorMode::PARENT;
  } else if (result == "CHILD") {
    return EdgeConfig::ColorMode::CHILD;
  } else {
    return EdgeConfig::ColorMode::NONE;
  }
}

}  // namespace hydra
