/* -----------------------------------------------------------------------------
 * Copyright 2022 Massachusetts Institute of Technology.
 * All Rights Reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  1. Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *
 *  2. Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Research was sponsored by the United States Air Force Research Laboratory and
 * the United States Air Force Artificial Intelligence Accelerator and was
 * accomplished under Cooperative Agreement Number FA8750-19-2-1000. The views
 * and conclusions contained in this document are those of the authors and should
 * not be interpreted as representing the official policies, either expressed or
 * implied, of the United States Air Force or the U.S. Government. The U.S.
 * Government is authorized to reproduce and distribute reprints for Government
 * purposes notwithstanding any copyright notation herein.
 * -------------------------------------------------------------------------- */
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
