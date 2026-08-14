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
#include <hydra/places/traversability_estimator.h>
#include <ianvs/node_handle.h>
#include <spark_dsg/bounding_box.h>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/publisher.hpp>

#include <Eigen/Geometry>

namespace hydra {

class TsdfGradientOccupancyPublisher
    : public hydra::places::GradientTraversabilityEstimator::Sink {
 public:
  struct Config {
    std::string ns = "~/tsdf_gradient";
    bool collate = false;
    bool use_relative_height = true;
    bool add_robot_footprint = false;  // force voxels around robot to be free
    Eigen::Vector3f footprint_min = Eigen::Vector3f::Zero();
    Eigen::Vector3f footprint_max = Eigen::Vector3f::Zero();

    float gradient_threshold = 0.5f;  // m/m - max traversable gradient
    float min_confidence = 0.5f;      // min confidence (neighbors/8) for valid cell
    bool probabilistic = false;       // continuous vs binary occupancy
    bool filter_disjoint = false;     // remove free space not connected to robot
  } const config;

  explicit TsdfGradientOccupancyPublisher(const Config& config);

  virtual ~TsdfGradientOccupancyPublisher() = default;

  std::string printInfo() const override;

  void call(const hydra::places::HeightMap& height_map,
            const hydra::places::GradientMap& gradient_map,
            const ActiveWindowOutput& output,
            const TsdfLayer& tsdf_layer) const override;

 private:
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr height_map_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr gradient_map_pub_;

  void fillOccupancyGrid(const hydra::places::GradientMap& gradient_map,
                         const Eigen::Isometry3d& world_T_sensor,
                         const TsdfLayer& layer,
                         nav_msgs::msg::OccupancyGrid& msg) const;

  void filterDisjointFreeSpace(nav_msgs::msg::OccupancyGrid& msg,
                               const Eigen::Isometry3d& world_T_body) const;

  void publishHeightMapViz(const hydra::places::HeightMap& height_map,
                           const TsdfLayer& layer,
                           uint64_t timestamp_ns,
                           float robot_z) const;

  void publishGradientMapViz(const hydra::places::GradientMap& gradient_map,
                             const TsdfLayer& layer,
                             uint64_t timestamp_ns,
                             float robot_z) const;

  int8_t gradientToOccupancy(float gradient, float confidence) const;
};

void declare_config(TsdfGradientOccupancyPublisher::Config& config);

}  // namespace hydra
