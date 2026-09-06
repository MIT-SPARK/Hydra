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
#include <limits>
#include <opencv2/core/mat.hpp>

#include "hydra/common/common_types.h"
#include "hydra/input/sensor.h"
#include "hydra/openset/openset_types.h"

namespace hydra {

struct InputData {
  using Ptr = std::shared_ptr<InputData>;
  using ConstPtr = std::shared_ptr<const InputData>;

  // Types of the stored image data.
  using ColorType = cv::Vec3b;
  inline static constexpr auto ColorMatType = CV_8UC3;
  using RangeType = float;
  inline static constexpr auto RangeMatType = CV_32FC1;
  using VertexType = cv::Vec3f;
  inline static constexpr auto VertexMatType = CV_32FC3;
  using LabelType = int;
  inline static constexpr auto LabelMatType = CV_32SC1;
  using InstanceType = int16_t;
  inline static constexpr auto InstanceMatType = CV_16SC1;
  using MaskType = uint8_t;
  inline static constexpr auto MaskMatType = CV_8UC1;

  explicit InputData(Sensor::ConstPtr sensor);

  virtual ~InputData() = default;

  //! Get the sensor that captured this data.
  const Sensor& getSensor() const;

  //! Get the pose of the sensor in world frame when this data was captured.
  Eigen::Isometry3d getSensorPose() const;

  //! Check if range value is in allowable sensor and data range
  bool inRange(float range_m) const;

  /**
   * @brief Normalize and fill all fields
   * @param vertices_in_world_frame Convert the vertex image to be in world frame.
   * @param normalize_labels Force label normalization.
   * @return Whether or not conversions succeeded.
   */
  bool finalize(bool vertices_in_world_frame = false, bool normalize_labels = true);

  //! Time stamp this input data was captured.
  TimeStamp timestamp_ns;
  //! Pose of the robot body in the world frame.
  Eigen::Isometry3d world_T_body;

  //! Color image as RGB.
  cv::Mat color_image;
  //! Color mask (primarily for backprojected LiDAR)
  cv::Mat color_mask;
  //! Depth image as planar depth in meters.
  cv::Mat depth_image;
  //! Ray lengths in meters.
  cv::Mat range_image;
  //! Label image for semantic input data.
  cv::Mat label_image;
  //! Instance image for semantic input data.
  cv::Mat instance_image;
  //! 3D points of the range image in sensor or world frame.
  cv::Mat vertex_map;
  //! Image of pixel-wise traversability estimates
  cv::Mat traversability_image;
  //! Feature associated with current input data
  FeatureVector feature;
  //! Features associated with each label
  FeatureMap<int> label_features;

  //! Whether or not the vertex map is in the world frame (or sensor frame).
  bool points_in_world_frame = false;
  //! Min range observed in the range image.
  float min_range = 0.0f;
  //! Max range observed in the range image.
  float max_range = std::numeric_limits<float>::infinity();

 private:
  Sensor::ConstPtr sensor_;
};

};  // namespace hydra
