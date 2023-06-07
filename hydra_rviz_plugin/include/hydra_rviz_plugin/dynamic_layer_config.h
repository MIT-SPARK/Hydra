#pragma once
#include <iostream>

namespace hydra {

struct DynamicLayerConfig {
  double offset_scale = 0.0;
  bool visualize = true;
  double node_scale = 0.25;
  double node_alpha = 1.0;
  bool use_spheres = true;
  bool use_label = false;
  double label_height_ratio = 2.0;
  double label_scale = 0.5;
  bool collapse_layer = true;
  double edge_scale = 0.1;
  double edge_alpha = 0.6;

  enum class ColorMode : int {
    SINGLE_COLOR,
    NONE,
  } color_mode = ColorMode::SINGLE_COLOR;
};

DynamicLayerConfig::ColorMode stringToDynamicColorMode(const std::string& input);

std::ostream& operator<<(std::ostream& out, const DynamicLayerConfig& conf);

}  // namespace hydra
