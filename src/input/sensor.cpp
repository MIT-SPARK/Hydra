// The contents of this file are originally from Panoptic-Mapping,
// under the following license:
//
// BSD 3-Clause License
// Copyright (c) 2021, ETHZ ASL
// All rights reserved.
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice, this
// list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
// this list of conditions and the following disclaimer in the documentation
// and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its
// contributors may be used to endorse or promote products derived from
// this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// See https://github.com/ethz-asl/panoptic_mapping for original code and paper
//
// Modifications (including work done by Lukas Schmid for Khronos) fall under the same
// license as Hydra and are subject to the following copyright and disclaimer:
//
// Copyright 2022 Massachusetts Institute of Technology.
// All Rights Reserved
//
// Research was sponsored by the United States Air Force Research Laboratory and
// the United States Air Force Artificial Intelligence Accelerator and was
// accomplished under Cooperative Agreement Number FA8750-19-2-1000. The views
// and conclusions contained in this document are those of the authors and should
// not be interpreted as representing the official policies, either expressed or
// implied, of the United States Air Force or the U.S. Government. The U.S.
// Government is authorized to reproduce and distribute reprints for Government
// purposes notwithstanding any copyright notation herein.
#include "hydra/input/sensor.h"

#include <config_utilities/config_utilities.h>
#include <config_utilities/types/eigen_matrix.h>
#include <glog/logging.h>

#include "hydra/common/config_utilities.h"

namespace hydra {

SensorExtrinsics::SensorExtrinsics()
    : SensorExtrinsics(Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero()) {}

SensorExtrinsics::SensorExtrinsics(const Eigen::Quaterniond& body_R_sensor)
    : SensorExtrinsics(body_R_sensor, Eigen::Vector3d::Zero()) {}

SensorExtrinsics::SensorExtrinsics(const Eigen::Vector3d& body_p_sensor)
    : SensorExtrinsics(Eigen::Quaterniond::Identity(), body_p_sensor) {}

SensorExtrinsics::SensorExtrinsics(const Eigen::Quaterniond& _body_R_sensor,
                                   const Eigen::Vector3d& _body_p_sensor)
    : body_R_sensor(_body_R_sensor), body_p_sensor(_body_p_sensor) {}

IdentitySensorExtrinsics::IdentitySensorExtrinsics(const Config&)
    : SensorExtrinsics() {}

ParamSensorExtrinsics::ParamSensorExtrinsics(const Config& config)
    : SensorExtrinsics(config.body_R_sensor, config.body_p_sensor) {}

KimeraSensorExtrinsics::KimeraSensorExtrinsics(const Config& config)
    : SensorExtrinsics() {
  config::checkValid(config);
  const auto node = YAML::LoadFile(config.sensor_filepath);
  const auto elements = node["T_BS"]["data"].as<std::vector<double>>();
  CHECK_EQ(elements.size(), 16u);

  Eigen::Matrix4d body_T_sensor = Eigen::Matrix4d::Identity();
  for (int r = 0; r < 4; ++r) {
    for (int c = 0; c < 4; ++c) {
      body_T_sensor(r, c) = elements.at(4 * r + c);
    }
  }

  body_R_sensor = Eigen::Quaterniond(body_T_sensor.block<3, 3>(0, 0)).normalized();
  body_p_sensor = body_T_sensor.block<3, 1>(0, 3);
}

Sensor::Sensor(const Config& config)
    : config(config::checkValid(config)), extrinsics_(config.extrinsics.create()) {
  CHECK(extrinsics_ != nullptr) << "invalid extrinsics!";
  const Eigen::Isometry3d sensor_body_pose = body_T_sensor();
  VLOG(1) << "Parsed sensor with extrinsics: " << std::endl
          << sensor_body_pose.matrix();
}

void declare_config(IdentitySensorExtrinsics::Config&) {
  using namespace config;
  name("IdentitySensorExtrinsics");
}

void declare_config(ParamSensorExtrinsics::Config& conf) {
  using namespace config;
  name("ParamSensorExtrinsics");
  field<QuaternionConverter>(conf.body_R_sensor, "body_R_sensor");
  field(conf.body_p_sensor, "body_p_sensor");
  checkCondition(std::abs(conf.body_R_sensor.norm() - 1.0) < 1.0e-9,
                 "Quaternion is not normalized");
}

void declare_config(KimeraSensorExtrinsics::Config& conf) {
  using namespace config;
  name("KimeraSensorExtrinsics");
  field(conf.sensor_filepath, "sensor_filepath");
  // TODO(nathan) validate file
}

void declare_config(Sensor::Config& conf) {
  using namespace config;
  name("Sensor");
  field(conf.min_range, "min_range", "m");
  field(conf.max_range, "max_range", "m");
  field(conf.extrinsics, "extrinsics");
  check(conf.min_range, GT, 0.0, "min_range");
  checkCondition(conf.max_range > conf.min_range,
                 "param 'max_range' is expected > 'min_range'");
}

}  // namespace hydra
