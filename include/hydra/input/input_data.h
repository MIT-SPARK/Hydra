#pragma once

#include <Eigen/Geometry>
#include <limits>
#include <opencv2/core/mat.hpp>

#include "hydra/input/sensor.h"

namespace hydra {

struct InputData {
  using Ptr = std::shared_ptr<InputData>;

  uint64_t timestamp_ns;
  size_t sensor_id;
  Eigen::Isometry3d world_T_body;

  cv::Mat color_image;
  bool color_is_bgr = false;
  cv::Mat depth_image;
  cv::Mat label_image;
  cv::Mat range_image;
  cv::Mat vertex_map;
  bool points_in_world_frame = false;
  float min_range = 0.0f;
  float max_range = std::numeric_limits<float>::infinity();

  virtual ~InputData() = default;

  /**
   * @brief check that we have enough data to round out the rest of the information
   */
  virtual bool hasData() const;

  template <typename T = double>
  Eigen::Transform<T, 3, Eigen::Isometry> getSensorPose(const Sensor& sensor) const {
    return world_T_body.cast<T>() * sensor.body_T_sensor<T>();
  }
};

};  // namespace hydra
