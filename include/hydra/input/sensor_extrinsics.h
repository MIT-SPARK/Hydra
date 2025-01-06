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

#include <Eigen/Geometry>

namespace hydra {

struct SensorExtrinsics {
  SensorExtrinsics();

  explicit SensorExtrinsics(const Eigen::Quaterniond& body_R_sensor);

  explicit SensorExtrinsics(const Eigen::Vector3d& body_p_sensor);

  SensorExtrinsics(const Eigen::Quaterniond& body_R_sensor,
                   const Eigen::Vector3d& body_p_sensor);

  inline operator Eigen::Isometry3d() const {
    return Eigen::Translation3d(body_p_sensor) * body_R_sensor;
  }

  Eigen::Quaterniond body_R_sensor;
  Eigen::Vector3d body_p_sensor;
};

struct IdentitySensorExtrinsics : public SensorExtrinsics {
  struct Config {};

  explicit IdentitySensorExtrinsics(const Config& config);
};

struct ParamSensorExtrinsics : public SensorExtrinsics {
  struct Config {
    Eigen::Quaterniond body_R_sensor = Eigen::Quaterniond::Identity();
    Eigen::Vector3d body_p_sensor = Eigen::Vector3d::Zero();
  };

  explicit ParamSensorExtrinsics(const Config& config);
};

struct KimeraSensorExtrinsics : public SensorExtrinsics {
  struct Config {
    std::string sensor_filepath = "";
  };

  explicit KimeraSensorExtrinsics(const Config& config);
};

void declare_config(IdentitySensorExtrinsics::Config& config);
void declare_config(ParamSensorExtrinsics::Config& config);
void declare_config(KimeraSensorExtrinsics::Config& config);

}  // namespace hydra
