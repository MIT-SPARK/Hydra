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
#include "hydra_ros/utils/input_data_to_messages.h"

#include <config_utilities/config.h>
#include <hydra/input/input_data.h>

#include <cv_bridge/cv_bridge.hpp>
#include <sensor_msgs/msg/point_field.hpp>

namespace hydra {

using sensor_msgs::msg::Image;
using sensor_msgs::msg::PointCloud2;
using sensor_msgs::msg::PointField;

namespace {

PointField makeField(const std::string& name,
                     uint32_t offset,
                     uint8_t datatype,
                     uint32_t count = 1) {
  PointField field;
  field.name = name;
  field.offset = offset;
  field.datatype = datatype;
  field.count = count;
  return field;
}

cv::Mat resizeImage(const cv::Mat& input, const DisplayConfig& config) {
  if (config.width_scale == 1.0f && config.height_scale == 1.0f) {
    return input;
  }

  cv::Mat output;
  cv::resize(input,
             output,
             cv::Size(),
             config.width_scale,
             config.height_scale,
             cv::INTER_NEAREST);
  return output;
}

}  // namespace

void declare_config(DisplayConfig& config) {
  using namespace config;
  name("DisplayConfig");
  field(config.width_scale, "width_scale");
  field(config.height_scale, "height_scale");
  field(config.overlay_alpha, "overlay_alpha");
  field(config.min_distance, "min_distance", "m");
  field(config.max_distance, "max_distance", "m");
  field(config.distance_colormap, "distance_colormap", "m");
  // overlay == 0.0 doesn't make sense, but overlay == 1.0 does
  checkInRange(config.overlay_alpha, 0.0f, 1.0f, "overlay_alpha", false, true);
}

Image::SharedPtr convertImage(const std_msgs::msg::Header& header,
                              const cv::Mat& img,
                              const DisplayConfig& config) {
  cv_bridge::CvImagePtr msg(new cv_bridge::CvImage());
  msg->header = header;
  msg->encoding = "rgb8";
  msg->image = resizeImage(img, config);
  return msg->toImageMsg();
}

Image::SharedPtr makeImage(const std_msgs::msg::Header& header,
                           const cv::Mat& img_in,
                           const CmapFunc& colormap,
                           const DisplayConfig& config) {
  cv::Mat img(img_in.rows, img_in.cols, CV_8UC3);
  for (int r = 0; r < img_in.rows; ++r) {
    for (int c = 0; c < img_in.cols; ++c) {
      auto pixel = img.ptr<uint8_t>(r, c);
      const auto color = colormap(img_in, r, c);
      *pixel = color.r;
      *(pixel + 1) = color.g;
      *(pixel + 2) = color.b;
    }
  }

  return convertImage(header, img, config);
}

Image::SharedPtr makeOverlayImage(const std_msgs::msg::Header& header,
                                  const cv::Mat& img_in,
                                  const cv::Mat& color_in,
                                  const CmapFunc& colormap,
                                  const DisplayConfig& config) {
  if (color_in.empty()) {
    return makeImage(header, img_in, colormap, config);
  }

  cv::Mat img(img_in.rows, img_in.cols, CV_8UC3);
  for (int r = 0; r < img_in.rows; ++r) {
    for (int c = 0; c < img_in.cols; ++c) {
      auto pixel = img.ptr<uint8_t>(r, c);
      const auto c_in = colormap(img_in, r, c);
      const auto c_cv = color_in.at<cv::Vec3b>(r, c);
      const auto color =
          c_in.blend(spark_dsg::Color(c_cv[0], c_cv[1], c_cv[2]), config.overlay_alpha);

      *pixel = color.r;
      *(pixel + 1) = color.g;
      *(pixel + 2) = color.b;
    }
  }

  return convertImage(header, img, config);
}

Image::SharedPtr makeDistImage(const std_msgs::msg::Header& header,
                               const cv::Mat& distances,
                               const DisplayConfig& config) {
  double min_v = 0.0;
  double max_v = std::numeric_limits<double>::infinity();
  cv::minMaxIdx(distances, &min_v, &max_v);
  const float v_min = config.min_distance >= 0.0f ? config.min_distance : min_v;
  const float v_max = config.max_distance >= 0.0f ? config.max_distance : max_v;
  const visualizer::RangeColormap cmap(config.distance_colormap);
  return makeImage(
      header,
      distances,
      [&](const cv::Mat& img, int r, int c) {
        return cmap(img.at<float>(r, c), v_min, v_max);
      },
      config);
}

// TODO(nathan) pcl_ros would avoid this but it causes compile issues (and would also
// require going to pcl pointcloud as an intermediate type)
PointCloud2::UniquePtr makeCloud(const std_msgs::msg::Header& header,
                                 const InputData& sensor_data,
                                 bool filter_by_range) {
  const auto& sensor = sensor_data.getSensor();
  const auto& labels = sensor_data.label_image;
  const auto& points = sensor_data.vertex_map;
  const auto& colors = sensor_data.color_image;
  const auto& ranges = sensor_data.range_image;
  const auto has_color = !colors.empty();
  const auto has_labels = !labels.empty();

  auto cloud = std::make_unique<PointCloud2>();
  cloud->header = header;
  cloud->height = points.rows;
  cloud->width = points.cols;

  auto& step = cloud->point_step;
  cloud->fields.push_back(makeField("x", step, PointField::FLOAT32));
  step += sizeof(float);
  cloud->fields.push_back(makeField("y", step, PointField::FLOAT32));
  step += sizeof(float);
  cloud->fields.push_back(makeField("z", step, PointField::FLOAT32));
  step += sizeof(float);
  if (has_color) {
    cloud->fields.push_back(makeField("rgba", step, PointField::UINT32));
    step += sizeof(uint32_t);
  }

  if (has_labels) {
    cloud->fields.push_back(makeField("label", step, PointField::UINT32));
    step += sizeof(uint32_t);
  }

  cloud->row_step = cloud->point_step * cloud->width;
  cloud->data.resize(cloud->point_step * cloud->width * cloud->height);
  cloud->is_dense = true;

  const Eigen::Isometry3f world_T_sensor = sensor_data.getSensorPose().cast<float>();
  for (int r = 0; r < points.rows; ++r) {
    for (int c = 0; c < points.cols; ++c) {
      size_t offset = r * cloud->row_step + c * cloud->point_step;
      const auto range = ranges.at<float>(r, c);
      const auto& p = points.at<cv::Vec3f>(r, c);

      Eigen::Vector3f point;
      point << p[0], p[1], p[2];
      if (!sensor_data.points_in_world_frame) {
        point = (world_T_sensor * point).eval();
      }

      auto xyz = reinterpret_cast<float*>(cloud->data.data() + offset);
      if (filter_by_range &&
          (range < sensor.min_range() || range > sensor.max_range())) {
        *xyz = std::numeric_limits<float>::quiet_NaN();
        *(xyz + 1) = std::numeric_limits<float>::quiet_NaN();
        *(xyz + 2) = std::numeric_limits<float>::quiet_NaN();

      } else {
        *xyz = point.x();
        *(xyz + 1) = point.y();
        *(xyz + 2) = point.z();
      }
      offset += 3 * sizeof(float);

      if (has_color) {
        const auto color = colors.at<cv::Vec3b>(r, c);
        auto rgb = cloud->data.data() + offset;
        *rgb = color[2];
        *(rgb + 1) = color[1];
        *(rgb + 2) = color[0];
        *(rgb + 3) = 255u;
        offset += sizeof(uint32_t);
      }

      if (has_labels) {
        auto label = reinterpret_cast<uint32_t*>(cloud->data.data() + offset);
        *label = labels.at<uint32_t>(r, c);
      }
    }
  }

  return cloud;
}

}  // namespace hydra
