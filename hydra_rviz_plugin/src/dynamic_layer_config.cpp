#include "hydra_rviz_plugin/dynamic_layer_config.h"

#include <algorithm>

namespace hydra {

std::ostream& operator<<(std::ostream& out, DynamicLayerConfig::ColorMode mode) {
  switch (mode) {
    case DynamicLayerConfig::ColorMode::SINGLE_COLOR:
      out << "SINGLE_COLOR";
      break;
    case DynamicLayerConfig::ColorMode::NONE:
    default:
      out << "NONE";
      break;
  }
  return out;
}

std::ostream& operator<<(std::ostream& out, const DynamicLayerConfig& conf) {
  out << std::boolalpha << "{offset_scale: " << conf.offset_scale
      << ", visualize: " << conf.visualize << ", node_scale: " << conf.node_scale
      << ", node_alpha: " << conf.node_alpha << ", use_spheres: " << conf.use_spheres
      << ", use_label: " << conf.use_label
      << ", label_height_ratio: " << conf.label_height_ratio
      << ", label_scale: " << conf.label_scale
      << ", collapse_bounding_box: " << conf.collapse_layer
      << ", edge_scale: " << conf.edge_scale << ", edge_alpha: " << conf.edge_alpha
      << ", color_mode: " << conf.color_mode << "}";
  return out;
}

DynamicLayerConfig::ColorMode stringToDynamicColorMode(const std::string& input) {
  std::string result = input;
  std::transform(result.begin(), result.end(), result.begin(), [](unsigned char c) {
    return std::toupper(c);
  });

  if (result == "SINGLE_COLOR") {
    return DynamicLayerConfig::ColorMode::SINGLE_COLOR;
  } else {
    return DynamicLayerConfig::ColorMode::NONE;
  }
}

}  // namespace hydra
