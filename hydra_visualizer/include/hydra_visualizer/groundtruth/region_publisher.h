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
#include <filesystem>

#include <rclcpp/node.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "hydra_visualizer/utils/marker_tracker.h"
#include <Eigen/Dense>

namespace hydra {

struct Region {
  Eigen::MatrixXd points;
  Eigen::Vector3d centroid;
  std::string name;
  std_msgs::msg::ColorRGBA color;
};

class RegionPublisher : rclcpp::Node {
 public:
  struct Config {
    std::string frame_id = "map";
    std::filesystem::path region_filepath;
    bool skip_unknown = true;
    bool draw_labels = false;
    bool fill_polygons = true;
    bool draw_polygon_boundaries = true;
    bool draw_polygon_vertices = true;
    double line_width = 0.05;
    double line_alpha = 0.8;
    double mesh_alpha = 0.6;
    double label_scale = 0.7;
    double label_offset = 0.0;
    double z_offset = 0.1;
    bool use_boundary_color = true;
  } const config;

  explicit RegionPublisher(const rclcpp::NodeOptions& options);
  virtual ~RegionPublisher() = default;

 protected:
  void publish();
  void reset();

  bool published_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_;
  MarkerTracker tracker_;
  std::vector<Region> regions_;
};

void declare_config(RegionPublisher::Config& config);

}  // namespace hydra
