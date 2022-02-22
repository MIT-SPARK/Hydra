#pragma once
#include <Eigen/Dense>
#include <sstream>
#include "hydra_utils/config.h"

namespace YAML {

template <typename Scalar, int N>
struct convert<Eigen::Matrix<Scalar, N, 1>> {
  static Node encode(const Eigen::Matrix<Scalar, N, 1>& rhs) {
    Node node;
    for (int i = 0; i < N; ++i) {
      node.push_back(rhs(i));
    }

    return node;
  }

  static bool decode(const Node& node, Eigen::Matrix<Scalar, N, 1>& rhs) {
    std::cout << node << std::endl;
    if (!node.IsSequence() || node.size() != N) {
      return false;
    }

    for (int i = 0; i < N; ++i) {
      rhs(i) = node[i].as<Scalar>();
    }

    return true;
  }
};

template <int N>
struct convert<Eigen::Matrix<uint8_t, N, 1>> {
  static Node encode(const Eigen::Matrix<uint8_t, N, 1>& rhs) {
    Node node;
    for (int i = 0; i < N; ++i) {
      node.push_back(static_cast<int>(rhs(i)));
    }

    return node;
  }

  static bool decode(const Node& node, Eigen::Matrix<uint8_t, N, 1>& rhs) {
    if (!node.IsSequence() || node.size() != N) {
      return false;
    }

    for (int i = 0; i < N; ++i) {
      rhs(i) = node[i].as<uint16_t>();
    }

    return true;
  }
};

}  // namespace YAML

namespace Eigen {

template <typename Scalar, int N>
bool readRosParam(const ros::NodeHandle& nh,
                  const std::string& name,
                  Matrix<Scalar, N, 1>& value) {
  std::vector<Scalar> raw_values;
  config_parser::readRosParam(nh, name, raw_values);
  if (raw_values.empty()) {
    return false;
  }

  if (raw_values.size() != static_cast<size_t>(N)) {
    std::stringstream ss;
    ss << "invalid param length: " << raw_values.size() << " != " << N;
    throw std::domain_error(ss.str());
  }

  for (int i = 0; i < N; ++i) {
    value(i) = raw_values[i];
  }

  return true;
}

template <typename Scalar, int N>
void displayParam(std::ostream& out, const Eigen::Matrix<Scalar, N, 1>& value) {
  Eigen::IOFormat format(
      Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", ", ", "", "", "[", "]");
  out << value.format(format);
}

template <int N>
void displayParam(std::ostream& out, const Eigen::Matrix<uint8_t, N, 1>& value) {
  Eigen::IOFormat format(
      Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", ", ", "", "", "[", "]");
  Eigen::Matrix<int, N, 1> to_show = value.template cast<int>();
  out << to_show.format(format);
}

}  // namespace Eigen
