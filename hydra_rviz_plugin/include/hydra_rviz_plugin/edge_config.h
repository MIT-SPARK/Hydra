#pragma once
#include <iostream>

namespace hydra {

struct EdgeConfig {
  int insertion_skip = 0;
  bool visualize = true;
  double edge_scale = 0.1;
  double edge_alpha = 0.6;

  enum class ColorMode : int {
    PARENT,
    CHILD,
    NONE,
  } color_mode = ColorMode::NONE;
  bool label_colormap = false;
};

EdgeConfig::ColorMode stringToEdgeColorMode(const std::string& input);

std::ostream& operator<<(std::ostream& out, const EdgeConfig& conf);

}  // namespace hydra
