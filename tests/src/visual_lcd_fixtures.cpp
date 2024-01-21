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
#include "hydra_test/visual_lcd_fixtures.h"

#include <glog/logging.h>
#include <gtest/gtest.h>
#include <gtsam/geometry/Pose3.h>
#include <hydra/loop_closure/visual_descriptors.h>
#include <hydra/reconstruction/camera.h>
#include <yaml-cpp/yaml.h>

#include <opencv2/imgcodecs.hpp>

#include "hydra_test/resources.h"

namespace hydra::test {

Sensor::Ptr makeUHumans2Camera() {
  // uhumans2 camera parameters
  Camera::Config cam_config;
  cam_config.min_range = 0.1;
  cam_config.max_range = 5.0;
  cam_config.width = 720;
  cam_config.height = 480;
  cam_config.fx = 415.69219381653056;
  cam_config.fy = 415.69219381653056;
  cam_config.cx = 360.0;
  cam_config.cy = 240.0;
  cam_config.extrinsics = config::VirtualConfig<SensorExtrinsics>(
      IdentitySensorExtrinsics::Config(), "identity");
  return std::make_shared<Camera>(cam_config);
}

std::shared_ptr<FrameData> loadFrameData(const Sensor& sensor, size_t index) {
  std::stringstream ss;
  ss << std::setfill('0') << std::setw(3) << index;
  const auto img_path =
      test::get_resource_path("loop_closure/visual/img_" + ss.str() + ".png");
  const auto depth_path =
      test::get_resource_path("loop_closure/visual/depth_" + ss.str() + ".tiff");

  auto data = std::make_shared<FrameData>();
  data->color_image = cv::imread(img_path);
  data->color_is_bgr = true;
  data->depth_image = cv::imread(depth_path, cv::IMREAD_UNCHANGED);
  data->normalizeData();
  sensor.finalizeRepresentations(*data);
  return data;
}

lcd::SensorDescriptorFactory::Ptr loadDBoW2Factory(const std::string& vocabulary_path) {
  lcd::DBoWFactory::Config config;
  config.detector = config::VirtualConfig<lcd::FeatureDetector>(
      lcd::OrbFeatureDetector::Config(), "ORB");
  config.vocabulary_path = vocabulary_path;
  return std::make_unique<lcd::DBoWFactory>(config);
}

std::vector<gtsam::Pose3> loadPoses(const std::string& pose_filepath) {
  const auto node = YAML::LoadFile(pose_filepath);

  std::vector<gtsam::Pose3> poses;
  for (const auto& pose : node) {
    const auto& q_node = pose.second["q"];
    const auto& t_node = pose.second["t"];
    Eigen::Vector3d t;
    t << t_node["x"].as<double>(), t_node["y"].as<double>(), t_node["z"].as<double>();
    Eigen::Quaterniond q(q_node["w"].as<double>(),
                         q_node["x"].as<double>(),
                         q_node["y"].as<double>(),
                         q_node["z"].as<double>());
    poses.push_back(gtsam::Pose3(gtsam::Rot3(q), t));
  }

  return poses;
}

}  // namespace hydra::test
