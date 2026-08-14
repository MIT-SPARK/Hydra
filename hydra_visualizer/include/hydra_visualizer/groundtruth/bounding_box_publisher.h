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
#include <spark_dsg/bounding_box.h>

#include <filesystem>

#include <rclcpp/node.hpp>
#include <std_msgs/msg/string.hpp>

#include "hydra_visualizer/color/colormap_utilities.h"
#include "hydra_visualizer/utils/marker_tracker.h"

namespace hydra {

class BoundingBoxPublisher : public rclcpp::Node {
 public:
  using Ptr = std::shared_ptr<BoundingBoxPublisher>;
  using Annotations = std::map<std::string, std::vector<spark_dsg::BoundingBox>>;

  struct Config {
    std::filesystem::path filepath;
    std::string frame_id = "map";
    std::string marker_ns = "";
    bool draw_labels = true;
    double scale = 0.015;
    double text_scale = 0.25;
    double text_line_scale = 0.01;
    double text_height = 0.8;
    double alpha = 1.0;
    visualizer::DiscreteColormap::Config colormap;
  } const config;

  explicit BoundingBoxPublisher(const rclcpp::NodeOptions& options);

  void load(const std_msgs::msg::String& msg);

 private:
  void drawBoxes(const std_msgs::msg::Header& header,
                 const Annotations& boxes,
                 visualization_msgs::msg::MarkerArray& msg) const;

  void drawLabels(const std_msgs::msg::Header& header,
                  const Annotations& boxes,
                  visualization_msgs::msg::MarkerArray& msg) const;

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
  std::string marker_ns_;
  mutable MarkerTracker tracker_;
  const visualizer::DiscreteColormap colormap_;
};

void declare_config(BoundingBoxPublisher::Config& config);

}  // namespace hydra
