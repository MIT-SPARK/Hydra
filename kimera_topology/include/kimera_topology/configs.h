#pragma once
#include <hydra_utils/config.h>
#include <voxblox_ros/mesh_vis.h>

namespace config_parser {

void RosParser::visit<voxblox::ColorMode>(voxblox::ColorMode& value) {
  std::string color = "";
  if (!nh_.getParam(name_, color)) {
    return;
  }

  std::transform(color.begin(), color.end(), color.begin(), [](unsigned char c) {
    return std::toupper(c);
  });

  if (color == "KCOLOR") {
    value = voxblox::ColorMode::kColor;
  } else if (color == "KHEIGHT") {
    value = voxblox::ColorMode::kHeight;
  } else if (color == "KNORMALS") {
    value = voxblox::ColorMode::kNormals;
  } else if (color == "KGRAY") {
    value = voxblox::ColorMode::kGray;
  } else if (color == "KLAMBERT") {
    value = voxblox::ColorMode::kLambert;
  } else if (color == "KLAMBERTCOLOR") {
    value = voxblox::ColorMode::kLambertColor;
  }
}

}  // namespace config_parser
