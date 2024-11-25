#pragma once

#include <Eigen/Geometry>
#include <limits>
#include <opencv2/core/mat.hpp>
#include <utility>

#include "hydra/common/common_types.h"
#include "hydra/input/sensor.h"
#include "hydra/openset/openset_types.h"

namespace hydra {

struct InputData {
  using Ptr = std::shared_ptr<InputData>;

  // Types of the stored image data.
  using ColorType = cv::Vec3b;
  using RangeType = float;
  using VertexType = cv::Vec3f;
  using LabelType = int;

  explicit InputData(Sensor::ConstPtr sensor) : sensor_(std::move(sensor)) {}
  virtual ~InputData() = default;

  //! Time stamp this input data was captured.
  TimeStamp timestamp_ns;

  //! Pose of the robot body in the world frame.
  Eigen::Isometry3d world_T_body;

  //! Color image as RGB.
  cv::Mat color_image;

  //! Depth image as planar depth in meters.
  cv::Mat depth_image;

  //! Ray lengths in meters.
  cv::Mat range_image;

  //! Label image for semantic input data.
  cv::Mat label_image;

  //! 3D points of the range image in sensor or world frame.
  cv::Mat vertex_map;
  //! Whether or not the vertex map is in the world frame (or sensor frame).
  bool points_in_world_frame = false;

  //! Min range observed in the range image.
  float min_range = 0.0f;
  //! Max range observed in the range image.
  float max_range = std::numeric_limits<float>::infinity();

  //! Feature associated with current input data
  FeatureVector feature;

  //! Features associated with each label
  FeatureMap<int> label_features;

  /**
   * @brief Get the sensor that captured this data.
   */
  const Sensor& getSensor() const { return *sensor_; }

  /**
   * @brief Get the pose of the sensor in world frame when this data was captured.
   */
  Eigen::Isometry3d getSensorPose() const {
    return world_T_body * sensor_->body_T_sensor();
  }

  bool inRange(float range_m) const {
    return range_m >= sensor_->min_range() && range_m <= sensor_->max_range() &&
           range_m <= max_range;
  }

 private:
  Sensor::ConstPtr sensor_;
};

};  // namespace hydra
