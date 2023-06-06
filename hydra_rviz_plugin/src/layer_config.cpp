#include "hydra_rviz_plugin/layer_config.h"

#include <algorithm>

namespace hydra {

std::ostream& operator<<(std::ostream& out, const LayerConfig& conf) {
  out << std::boolalpha << "{offset_scale: " << conf.offset_scale
      << ", visualize: " << conf.visualize << ", node_scale: " << conf.node_scale
      << ", node_alpha: " << conf.node_alpha << ", use_spheres: " << conf.use_spheres
      << ", use_label: " << conf.use_label
      << ", label_height_ratio: " << conf.label_height_ratio
      << ", label_scale: " << conf.label_scale
      << ", use_bounding_box: " << conf.use_bounding_box
      << ", collapse_bounding_box: " << conf.collapse_bounding_box
      << ", bounding_box_scale: " << conf.bounding_box_scale
      << ", bounding_box_alpha: " << conf.bounding_box_alpha
      << ", edge_scale: " << conf.edge_scale << ", edge_alpha: " << conf.edge_alpha
      << "}";
  return out;
}

LayerConfig::ColorMode stringToColorMode(const std::string& input) {
  std::string result = input;
  std::transform(result.begin(), result.end(), result.begin(), [](unsigned char c) {
    return std::toupper(c);
  });

  if (result == "SINGLE_COLOR") {
    return LayerConfig::ColorMode::SINGLE_COLOR;
  } else if (result == "COLORMAP") {
    return LayerConfig::ColorMode::COLORMAP;
  } else if (result == "PARENT_COLOR") {
    return LayerConfig::ColorMode::PARENT_COLOR;
  } else {
    return LayerConfig::ColorMode::NONE;
  }
}

}  // namespace hydra
