#pragma once

namespace hydra {

struct LayerConfig {
  double offset_scale = 0.0;
  bool visualize = true;
  double node_scale = 0.25;
  double node_alpha = 1.0;
  bool use_spheres = true;
  bool use_label = false;
  double label_height_ratio = 2.0;
  double label_scale = 0.5;
  bool use_bounding_box = false;
  bool collapse_bounding_box = false;
  double bounding_box_scale = 0.05;
  double bounding_box_alpha = 1.0;
  double edge_scale = 0.1;
  double edge_alpha = 0.6;

  // TODO(nathan) color mode
};

}
