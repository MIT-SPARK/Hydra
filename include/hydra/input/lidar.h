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

#include <config_utilities/factory.h>

#include <vector>

#include "hydra/input/input_data.h"
#include "hydra/input/sensor.h"

namespace hydra {

/**
 * @brief Utility class bundling lidar related operations and data
 *
 * Based on LIDAR sensor model found here:
 * https://github.com/ethz-asl/voxblox/blob/dynablox/release/voxblox/include/voxblox/integrator/projective_tsdf_integrator_inl.h
 */
class Lidar : public Sensor {
 public:
  // Note: negative parameters are REQUIRED
  struct Config : public Sensor::Config {
    /// Lidar angular resolution (points/degrees)
    double horizontal_resolution = -1;
    /// Lidar resolution (points/degrees)
    double vertical_resolution = -1;
    /// Horizontal field of view (degrees)
    double horizontal_fov = 360.0;
    /// vertical field of view (degrees)
    double vertical_fov = -1.0;
    /// is vertical fov asymmetric?
    bool is_asymmetric = false;
    /// top offset of vertical field of view (degrees)
    double vertical_fov_top = -1.0;
  };

  explicit Lidar(const Config& config);

  virtual ~Lidar() = default;

  const Config& getConfig() const { return config_; }

  float computeRayDensity(float voxel_size, float depth) const override;

  /**
   * @brief Compute range image from pointcloud
   */
  bool finalizeRepresentations(InputData& input,
                               bool force_world_frame = false) const override;

  bool projectPointToImagePlane(const Eigen::Vector3f& p_C,
                                float& u,
                                float& v) const override;

  bool projectPointToImagePlane(const Eigen::Vector3f& p_C,
                                int& u,
                                int& v) const override;

  bool pointIsInViewFrustum(const Eigen::Vector3f& point_C,
                            float inflation_distance = 0.0f) const override;

 private:
  const Config config_;
  const int width_;
  const int height_;
  const float vertical_fov_rad_;
  const float vertical_fov_top_rad_;
  const float horizontal_fov_rad_;

  // Pre-computed stored values.
  Eigen::Vector3f top_frustum_normal_;
  Eigen::Vector3f bottom_frustum_normal_;
  Eigen::Vector3f left_frustum_normal_;
  Eigen::Vector3f right_frustum_normal_;

  inline static const auto registration_ =
      config::RegistrationWithConfig<Sensor, Lidar, Lidar::Config>("lidar");
};

void declare_config(Lidar::Config& config);

}  // namespace hydra
