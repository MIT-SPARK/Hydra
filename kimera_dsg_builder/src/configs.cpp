#include "kimera_dsg_builder/configs.h"

namespace kimera {
namespace incremental {

RoomFinder::Config::ClusterMode getRoomClusterModeFromString(const std::string& mode) {
  auto to_compare = config_parser::to_uppercase(mode);

  if (to_compare == "SPECTRAL") {
    return RoomFinder::Config::ClusterMode::SPECTRAL;
  }
  if (to_compare == "MODULARITY") {
    return RoomFinder::Config::ClusterMode::MODULARITY;
  }
  if (to_compare == "NONE") {
    return RoomFinder::Config::ClusterMode::NONE;
  }

  ROS_ERROR_STREAM("Unrecognized room clustering mode: " << to_compare
                                                         << ". Defaulting to NONE");
  return RoomFinder::Config::ClusterMode::NONE;
}

void readRosParam(const ros::NodeHandle& nh,
                  const std::string& name,
                  RoomFinder::Config::ClusterMode& mode) {
  std::string mode_str;
  if (!nh.getParam(name, mode_str)) {
    return;
  }

  mode = getRoomClusterModeFromString(mode_str);
}

std::ostream& operator<<(std::ostream& out, RoomFinder::Config::ClusterMode mode) {
  switch (mode) {
    case RoomFinder::Config::ClusterMode::SPECTRAL:
      out << "SPECTRAL";
      break;
    case RoomFinder::Config::ClusterMode::MODULARITY:
      out << "MODULARITY";
      break;
    case RoomFinder::Config::ClusterMode::NONE:
      out << "NONE";
      break;
    default:
      out << "INVALID";
      break;
  }
  return out;
}

}  // namespace incremental
}  // namespace kimera

namespace KimeraRPGO {

Verbosity getRpgoVerbosityFromString(const std::string& verb_str) {
  auto to_check = config_parser::to_uppercase(verb_str);

  if (to_check == "UPDATE") {
    return Verbosity::UPDATE;
  } else if (to_check == "QUIET") {
    return Verbosity::QUIET;
  } else if (to_check == "VERBOSE") {
    return Verbosity::VERBOSE;
  } else {
    ROS_ERROR_STREAM("unrecognized verbosity option: " << to_check
                                                       << ". defaulting to UPDATE");
    return Verbosity::UPDATE;
  }
}

void readRosParam(const ros::NodeHandle& nh, const std::string& name, Verbosity& mode) {
  std::string mode_str;
  if (!nh.getParam(name, mode_str)) {
    return;
  }

  mode = getRpgoVerbosityFromString(mode_str);
}

std::ostream& operator<<(std::ostream& out, Verbosity mode) {
  switch (mode) {
    case Verbosity::UPDATE:
      out << "UPDATE";
      break;
    case Verbosity::QUIET:
      out << "QUIET";
      break;
    case Verbosity::VERBOSE:
      out << "VERBOSE";
      break;
    default:
      out << "INVALID";
      break;
  }
  return out;
}

Solver getRpgoSolverFromString(const std::string& mode) {
  auto to_check = config_parser::to_uppercase(mode);

  if (to_check == "LM") {
    return Solver::LM;
  } else if (to_check == "GN") {
    return Solver::GN;
  } else {
    ROS_ERROR_STREAM("unrecognized solver option: " << to_check
                                                    << ". defaulting to LM");
    return Solver::LM;
  }
}

void readRosParam(const ros::NodeHandle& nh, const std::string& name, Solver& mode) {
  std::string mode_str;
  if (!nh.getParam(name, mode_str)) {
    return;
  }

  mode = getRpgoSolverFromString(mode_str);
}

std::ostream& operator<<(std::ostream& out, Solver mode) {
  switch (mode) {
    case Solver::LM:
      out << "LM";
      break;
    case Solver::GN:
      out << "GN";
      break;
    default:
      out << "INVALID";
      break;
  }
  return out;
}

}  // namespace KimeraRPGO
