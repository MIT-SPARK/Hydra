#include "kimera_topology/configs.h"

namespace voxblox {

void readRosParam(const ros::NodeHandle& nh, const std::string& name, ColorMode& mode) {
  std::string mode_str = "";
  if (!nh.getParam(name, mode_str)) {
    return;
  }

  mode = getColorModeFromString(mode_str);
}

std::ostream& operator<<(std::ostream& out, ColorMode mode) {
  switch (mode) {
    case ColorMode::kColor:
      out << "color";
      break;
    case ColorMode::kHeight:
      out << "height";
      break;
    case ColorMode::kNormals:
      out << "normals";
      break;
    case ColorMode::kGray:
      out << "gray";
      break;
    case ColorMode::kLambert:
      out << "lambert";
      break;
    case ColorMode::kLambertColor:
      out << "lambert_color";
      break;
    default:
      out << "INVALID";
      break;
  }
  return out;
}

}  // namespace voxblox

namespace kimera {
namespace topology {

ParentUniquenessMode getParentUniquenessModeFromString(const std::string& mode) {
  auto to_compare = config_parser::to_uppercase(mode);

  if (to_compare == "ANGLE") {
    return ParentUniquenessMode::ANGLE;
  }
  if (to_compare == "L1_DISTANCE") {
    return ParentUniquenessMode::L1_DISTANCE;
  }
  if (to_compare == "L1_THEN_ANGLE") {
    return ParentUniquenessMode::L1_THEN_ANGLE;
  }

  return ParentUniquenessMode::L1_THEN_ANGLE;
}

void readRosParam(const ros::NodeHandle& nh,
                  const std::string& name,
                  ParentUniquenessMode& mode) {
  std::string mode_str = "";
  if (!nh.getParam(name, mode_str)) {
    return;
  }

  mode = getParentUniquenessModeFromString(mode_str);
}

std::ostream& operator<<(std::ostream& out, ParentUniquenessMode mode) {
  switch (mode) {
    case ParentUniquenessMode::ANGLE:
      out << "ANGLE";
      break;
    case ParentUniquenessMode::L1_DISTANCE:
      out << "L1_DISTANCE";
      break;
    case ParentUniquenessMode::L1_THEN_ANGLE:
      out << "L1_THEN_ANGLE";
      break;
    default:
      out << "INVALID";
      break;
  }
  return out;
}

}  // namespace topology
}  // namespace kimera
