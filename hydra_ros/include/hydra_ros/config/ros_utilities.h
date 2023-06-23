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
#include <glog/logging.h>
#include <hydra/config/config.h>
#include <ros/ros.h>

#include <Eigen/Dense>

#include "hydra_ros/config/ros_parser.h"

namespace config_parser {

template <typename Config>
Config load_from_ros(const std::string& ns, Logger::Ptr logger = nullptr) {
  RosParser parser(std::make_unique<RosParserImpl>(ros::NodeHandle(ns)));
  if (logger) {
    parser.setLogger(logger);
  }

  Config config;
  ConfigVisitor<Config>::visit_config(parser, config);
  return config;
}

template <typename Config>
Config load_from_ros_nh(const ros::NodeHandle& nh,
                        const std::string& ns = "",
                        Logger::Ptr logger = nullptr) {
  RosParser parser(std::make_unique<RosParserImpl>(nh, ns));
  if (logger) {
    parser.setLogger(logger);
  }

  Config config;
  ConfigVisitor<Config>::visit_config(parser, config);
  return config;
}

}  // namespace config_parser

namespace hydra {

struct HydraParamLogger : config_parser::Logger {
  inline void log_missing(const std::string& message) const override {
    LOG(INFO) << message;
  }
};

template <typename T>
struct ConfigStructName {
  static std::string get() { return "Unknown Config"; }
};

template <typename Config>
Config load_config(const ros::NodeHandle& nh,
                   const std::string& ns = "",
                   bool verbose = true,
                   int dump_verbosity = 5) {
  auto logger = std::make_shared<HydraParamLogger>();
  auto config =
      config_parser::load_from_ros_nh<Config>(nh, ns, verbose ? logger : nullptr);

  if (verbose) {
    const auto name = ConfigStructName<Config>::get();
    std::string filler(name.size() + 50, '=');
    VLOG(dump_verbosity) << name << std::endl << filler << std::endl << config;
  }

  return config;
}

}  // namespace hydra

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

}  // namespace Eigen

#define DECLARE_STRUCT_NAME(name)              \
  template <>                                  \
  struct ConfigStructName<name> {              \
    static std::string get() { return #name; } \
  }
