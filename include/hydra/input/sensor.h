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
#pragma once
#include <config_utilities/virtual_config.h>

#include <Eigen/Geometry>
#include <limits>
#include <vector>

namespace hydra {

struct InputData;

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

 private:
  inline static const auto registration_ =
      config::RegistrationWithConfig<SensorExtrinsics,
                                     IdentitySensorExtrinsics,
                                     Config>("identity");
};

struct ParamSensorExtrinsics : public SensorExtrinsics {
  struct Config {
    Eigen::Quaterniond body_R_sensor = Eigen::Quaterniond::Identity();
    Eigen::Vector3d body_p_sensor = Eigen::Vector3d::Identity();
  };

  explicit ParamSensorExtrinsics(const Config& config);

 private:
  inline static const auto registration_ =
      config::RegistrationWithConfig<SensorExtrinsics, ParamSensorExtrinsics, Config>(
          "param");
};

struct KimeraSensorExtrinsics : public SensorExtrinsics {
  struct Config {
    std::string sensor_filepath = "";
  };

  explicit KimeraSensorExtrinsics(const Config& config);

 private:
  inline static const auto registration_ =
      config::RegistrationWithConfig<SensorExtrinsics, KimeraSensorExtrinsics, Config>(
          "kimera");
};

/**
 * @brief Base class for different sensors the system could use.
 *
 * Current design is around any sensor that can operate around images such as RGBD
 * cameras or LiDARs.
 */
class Sensor {
 public:
  using Ptr = std::shared_ptr<Sensor>;
  using ConstPtr = std::shared_ptr<const Sensor>;

  struct Config {
    double min_range = 0.0f;
    double max_range = std::numeric_limits<double>::infinity();
    config::VirtualConfig<SensorExtrinsics> extrinsics;
  } const config;

  explicit Sensor(const Config& config);

  virtual ~Sensor() = default;

  /**
   * @brief get the minimum valid range of the sensor
   */
  virtual float min_range() const { return config.min_range; }

  /**
   * @brief get the maximum valid range of the sensor
   */
  virtual float max_range() const { return config.max_range; }

  /**
   * @brief Get the sensor extrinsics (i.e., sensor pose in the body frame
   */
  template <typename T = double>
  Eigen::Transform<T, 3, Eigen::Isometry> body_T_sensor() const {
    Eigen::Isometry3d extrinsics_transform = *extrinsics_;
    return extrinsics_transform.cast<T>();
  }

  /**
   * @brief Get the average ray density at a voxel at a given range
   * @param voxel_size Voxel size to compute the density for
   * @param depth Depth to compute the density at
   */
  virtual float computeRayDensity(float voxel_size, float depth) const = 0;

  /**
   * @brief Compute any necessary alternate data representations
   *
   * For cameras, this would compute a range image and pointcloud from the depth
   * For lidar, this would compute a range image from the pointcloud
   *
   * @param input Input struct to fill
   * @param force_world_frame Always convert point cloud to be in the world frame, not the sensor frame
   * @return True if the sensor was able to provide all necessary information
   */
  virtual bool finalizeRepresentations(InputData& input,
                                       bool force_world_frame = false) const = 0;

  /**
   * @brief Projects a point in camera frame (C) into the image plane.
   * @param p_C Point in camera frame.
   * @param u Output x image plane coordinate in px.
   * @param v Output y image plane coordinate in px.
   * @return True if the point is in front of the camera and within the image.
   */
  virtual bool projectPointToImagePlane(const Eigen::Vector3f& p_C,
                                        float& u,
                                        float& v) const = 0;

  /**
   * @brief Projects a point in camera frame (C) into the image plane and rounds to the
   * closest integer pixel.
   * @param p_C Point in camera frame.
   * @param u Output x image plane coordinate in px.
   * @param v Output y image plane coordinate in px.
   * @return True if the point is in front of the camera and within the image.
   */
  virtual bool projectPointToImagePlane(const Eigen::Vector3f& p_C,
                                        int& u,
                                        int& v) const = 0;

  /**
   * @brief Checks if a point is in the camera's view frustum. Does not check for
   * occlusion.
   * @param point_C Point in camera frame.
   * @param inflation_distance Distance to inflate the frustum by.
   * @return True if the point is in the camera's (inflated) view frustum.
   */
  virtual bool pointIsInViewFrustum(const Eigen::Vector3f& point_C,
                                    float inflation_distance = 0.f) const = 0;

 protected:
  const std::unique_ptr<SensorExtrinsics> extrinsics_;
};

void declare_config(IdentitySensorExtrinsics::Config& config);
void declare_config(ParamSensorExtrinsics::Config& config);
void declare_config(KimeraSensorExtrinsics::Config& config);
void declare_config(Sensor::Config& config);

}  // namespace hydra
