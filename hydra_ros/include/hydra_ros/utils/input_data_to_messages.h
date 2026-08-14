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
#include <hydra_visualizer/color/colormap_utilities.h>
#include <spark_dsg/color.h>

#include <functional>
#include <limits>

#include <opencv2/core/mat.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace hydra {

struct InputData;
using CmapFunc = std::function<spark_dsg::Color(cv::Mat, int, int)>;

struct DisplayConfig {
  float width_scale = 1.0f;
  float height_scale = 1.0f;
  float overlay_alpha = 0.5f;
  float min_distance = -1.0f;
  float max_distance = -1.0f;
  visualizer::RangeColormap::Config distance_colormap;
};

void declare_config(DisplayConfig& config);

/**
 * @brief Turn an image into a message, optionally applying display config
 */
sensor_msgs::msg::Image::SharedPtr convertImage(const std_msgs::msg::Header& header,
                                                const cv::Mat& img,
                                                const DisplayConfig& config);

/**
 * @brief Make a colored image for the current labels in the input data
 */
sensor_msgs::msg::Image::SharedPtr makeImage(const std_msgs::msg::Header& header,
                                             const cv::Mat& img_in,
                                             const CmapFunc& colormap,
                                             const DisplayConfig& config = {});

/**
 * @brief Make a colored image for the current labels in the input data
 */
sensor_msgs::msg::Image::SharedPtr makeOverlayImage(const std_msgs::msg::Header& header,
                                                    const cv::Mat& img_in,
                                                    const cv::Mat& color_in,
                                                    const CmapFunc& colormap,
                                                    const DisplayConfig& config = {});

sensor_msgs::msg::Image::SharedPtr makeDistImage(const std_msgs::msg::Header& header,
                                                 const cv::Mat& img_in,
                                                 const DisplayConfig& config = {});

/**
 * @brief Convert the input pointcloud to a ROS type
 */
sensor_msgs::msg::PointCloud2::UniquePtr makeCloud(const std_msgs::msg::Header& header,
                                                   const InputData& sensor_data,
                                                   bool filter_by_range);

}  // namespace hydra
