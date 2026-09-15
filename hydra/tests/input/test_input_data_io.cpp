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
#include <gtest/gtest.h>
#include <hydra/input/camera.h>
#include <hydra/input/input_data_io.h>
#include <hydra/input/lidar.h>
#include <hydra/utils/zip_archive.h>
#include <unistd.h>
#include <yaml-cpp/yaml.h>

#include <cmath>
#include <cstring>
#include <map>
#include <opencv2/imgcodecs.hpp>

namespace hydra {
namespace {

Camera::Config cameraConfig() {
  Camera::Config config;
  config.width = 4;
  config.height = 3;
  config.fx = 3;
  config.fy = 4;
  config.cx = 1;
  config.cy = 2;
  config.min_range = 0.2;
  config.max_range = 17.5;
  config.invalid_labels = {4, 8};

  ParamSensorExtrinsics::Config extrinsics;
  extrinsics.body_p_sensor = Eigen::Vector3d(0.25, -0.5, 0.75);
  extrinsics.body_R_sensor = Eigen::AngleAxisd(0.4, Eigen::Vector3d::UnitY());
  config.extrinsics = extrinsics;
  return config;
}

Lidar::Config lidarConfig() {
  Lidar::Config config;
  config.min_range = 0.5;
  config.max_range = 120;
  config.horizontal_resolution = 1;
  config.vertical_resolution = 2;
  config.vertical_fov = 40;
  config.is_asymmetric = true;
  config.vertical_fov_top = 12;

  ParamSensorExtrinsics::Config extrinsics;
  extrinsics.body_p_sensor = Eigen::Vector3d(0.25, -0.5, 0.75);
  extrinsics.body_R_sensor = Eigen::AngleAxisd(0.4, Eigen::Vector3d::UnitY());
  config.extrinsics = extrinsics;
  return config;
}

InputData sampleInput(const Sensor::Ptr& sensor) {
  auto mask = cv::Mat(3, 4, CV_8UC3, cv::Scalar(1, 2, 3));
  auto sensor_with_mask = Sensor::fromRecord(sensor->dump(), sensor->name, mask);

  InputData data(sensor_with_mask);
  data.timestamp_ns = 18446744073709551614ULL;
  data.world_T_body =
      Eigen::Translation3d(2, -3, 4) * Eigen::AngleAxisd(0.7, Eigen::Vector3d::UnitX());
  data.points_in_world_frame = true;
  data.min_range = -0.5;
  data.max_range = std::numeric_limits<float>::infinity();

  data.color_image = cv::Mat(3, 4, CV_8UC3, cv::Scalar(10, 20, 30));
  data.color_mask = cv::Mat(3, 4, CV_8UC1, cv::Scalar(255));

  // Use an ROI to verify that row strides are not treated as contiguous storage.
  cv::Mat depth(3, 6, CV_32FC1, cv::Scalar(0.125));
  data.depth_image = depth(cv::Rect(1, 0, 4, 3));
  data.depth_image.at<float>(0, 0) = std::numeric_limits<float>::quiet_NaN();
  data.depth_image.at<float>(0, 1) = std::numeric_limits<float>::infinity();
  data.depth_image.at<float>(0, 2) = -std::numeric_limits<float>::infinity();
  data.depth_image.at<float>(0, 3) = -0.0f;

  data.range_image = cv::Mat(3, 4, CV_32FC1, cv::Scalar(9.75));

  data.label_image = cv::Mat(3, 4, CV_32SC1, cv::Scalar(-1));
  data.label_image.at<int32_t>(0, 1) = std::numeric_limits<int32_t>::max();
  data.label_image.at<int32_t>(0, 2) = std::numeric_limits<int32_t>::min();

  data.instance_image = cv::Mat(3, 4, CV_16SC1, cv::Scalar(-32768));
  data.instance_image.at<int16_t>(0, 1) = 32767;

  data.vertex_map = cv::Mat(3, 4, CV_32FC3, cv::Scalar(1.25, -2.5, 7.75));
  data.vertex_map.at<cv::Vec3f>(1, 2)[1] = std::numeric_limits<float>::quiet_NaN();

  data.traversability_image = cv::Mat(3, 4, CV_32FC1, cv::Scalar(0.375));

  data.feature = FeatureVector::Constant(1, 0.25f);
  data.label_features[-1] = FeatureVector::LinSpaced(3, -1, 1);
  data.label_features[42] = FeatureVector();

  return data;
}

void expectImage(const cv::Mat& expected, const cv::Mat& actual) {
  if (expected.empty()) {
    EXPECT_TRUE(actual.empty());
    return;
  }

  ASSERT_EQ(expected.size(), actual.size());
  ASSERT_EQ(expected.type(), actual.type());
  const auto row_bytes = expected.cols * expected.elemSize();
  for (int row = 0; row < expected.rows; ++row) {
    const auto result = std::memcmp(expected.ptr(row), actual.ptr(row), row_bytes);
    EXPECT_EQ(result, 0);
  }
}

void expectInput(const InputData& expected, const InputData& actual) {
  EXPECT_EQ(expected.timestamp_ns, actual.timestamp_ns);
  EXPECT_TRUE(expected.world_T_body.matrix().isApprox(actual.world_T_body.matrix()));
  EXPECT_TRUE(
      expected.getSensorPose().matrix().isApprox(actual.getSensorPose().matrix()));
  EXPECT_EQ(expected.points_in_world_frame, actual.points_in_world_frame);
  EXPECT_EQ(expected.min_range, actual.min_range);
  EXPECT_EQ(expected.max_range, actual.max_range);

  const auto& expected_sensor = expected.getSensor();
  const auto& actual_sensor = actual.getSensor();
  EXPECT_EQ(expected_sensor.name, actual_sensor.name);
  EXPECT_EQ(expected_sensor.min_range(), actual_sensor.min_range());
  EXPECT_EQ(expected_sensor.max_range(), actual_sensor.max_range());
  EXPECT_EQ(expected_sensor.config.invalid_labels, actual_sensor.config.invalid_labels);

  EXPECT_EQ(expected.feature, actual.feature);
  EXPECT_EQ(expected.label_features, actual.label_features);

  expectImage(expected.color_image, actual.color_image);
  expectImage(expected.color_mask, actual.color_mask);
  expectImage(expected.depth_image, actual.depth_image);
  expectImage(expected.range_image, actual.range_image);
  expectImage(expected.label_image, actual.label_image);
  expectImage(expected.instance_image, actual.instance_image);
  expectImage(expected.vertex_map, actual.vertex_map);
  expectImage(expected.traversability_image, actual.traversability_image);
  expectImage(expected_sensor.getStaticMask(), actual_sensor.getStaticMask());
}

class InputDataIo : public ::testing::Test {
 protected:
  void SetUp() override {
    const auto path = (std::filesystem::temp_directory_path() / "hydra-input-XXXXXX");
    auto pattern = path.string();
    ASSERT_NE(mkdtemp(pattern.data()), nullptr);
    directory = pattern;
  }

  void TearDown() override { std::filesystem::remove_all(directory); }

  std::filesystem::path directory;
};

}  // namespace

TEST_F(InputDataIo, CameraArchiveRoundTrip) {
  const auto path = directory / "frame.input.zip";
  const auto camera = std::make_shared<Camera>(cameraConfig(), "front_camera");
  const auto data = sampleInput(camera);
  data.save(path);

  const auto loaded = InputData::load(path);
  ASSERT_TRUE(loaded);
  expectInput(data, *loaded);

  const auto result_camera = dynamic_cast<const Camera*>(&loaded->getSensor());
  ASSERT_TRUE(result_camera);

  const auto result = result_camera->getConfig();
  EXPECT_EQ(result.fx, camera->getConfig().fx);
  EXPECT_EQ(result.fy, camera->getConfig().fy);
  EXPECT_EQ(result.cx, camera->getConfig().cx);
  EXPECT_EQ(result.cy, camera->getConfig().cy);
  EXPECT_EQ(result.width, camera->getConfig().width);
  EXPECT_EQ(result.height, camera->getConfig().height);
}

TEST_F(InputDataIo, LidarArchiveRoundTrip) {
  const auto path = directory / "frame.input.zip";
  const auto lidar = std::make_shared<Lidar>(lidarConfig(), "lidar");
  auto data = sampleInput(lidar);
  data.points_in_world_frame = false;
  data.depth_image = cv::Mat();
  data.save(path);

  const auto loaded = InputData::load(path);
  ASSERT_TRUE(loaded);
  expectInput(data, *loaded);

  const auto result_lidar = dynamic_cast<const Lidar*>(&loaded->getSensor());
  ASSERT_TRUE(result_lidar);

  const auto result = result_lidar->getConfig();
  EXPECT_EQ(result.vertical_fov_top, result.vertical_fov_top);
  EXPECT_EQ(result.horizontal_resolution, result.horizontal_resolution);
  EXPECT_EQ(result.vertical_resolution, result.vertical_resolution);
  EXPECT_EQ(result.is_asymmetric, result.is_asymmetric);
}

TEST_F(InputDataIo, EmptyArchiveRoundTrip) {
  const auto path = directory / "empty.zip";
  ASSERT_NO_THROW(io::writeArchive(path, [](const auto&) {}));
  EXPECT_GT(std::filesystem::file_size(path), 0);
  EXPECT_NO_THROW(io::readArchive(path, [](const auto&) {}));
}

TEST_F(InputDataIo, EmptyArchiveReplacesExistingArchive) {
  const auto path = directory / "empty.zip";
  io::writeArchive(path, [](const auto& write) { write("previous", {1, 2, 3}); });

  ASSERT_NO_THROW(io::writeArchive(path, [](const auto&) {}));
  EXPECT_NO_THROW(io::readArchive(path, [](const auto& read) {
    EXPECT_THROW(read("previous"), std::runtime_error);
  }));
  EXPECT_EQ(std::distance(std::filesystem::directory_iterator(directory),
                          std::filesystem::directory_iterator()),
            1);
}

TEST_F(InputDataIo, ArchiveOwnsEntryBuffers) {
  const auto path = directory / "buffers.zip";
  for (const auto level : {-1, 0, 1, 9}) {
    io::ArchiveOptions options;
    options.compression_level = level;
    io::writeArchive(
        path,
        [](const auto& write) {
          auto bytes = io::Bytes(1024 * 1024 + 1, 42);
          write("labels.tiff", bytes);
          bytes.assign(100, 7);
          write("color.png", bytes);
          write("empty", {});
        },
        options);
    io::readArchive(path, [](const auto& read) {
      EXPECT_EQ(read("labels.tiff"), io::Bytes(1024 * 1024 + 1, 42));
      EXPECT_EQ(read("color.png"), io::Bytes(100, 7));
      EXPECT_TRUE(read("empty").empty());
      EXPECT_THROW(read("missing"), std::runtime_error);
    });
  }
}

TEST_F(InputDataIo, FailedArchiveWritePreservesDestination) {
  const auto path = directory / "frame.input.zip";
  const auto camera = std::make_shared<Camera>(cameraConfig(), "camera");
  const auto data = sampleInput(camera);
  data.save(path);

  EXPECT_THROW(io::writeArchive(path,
                                [](const auto& write) {
                                  write("metadata.yaml", {1, 2, 3});
                                  throw std::runtime_error("interrupted write");
                                }),
               std::runtime_error);

  const auto loaded = InputData::load(path);
  ASSERT_TRUE(loaded);
  expectInput(data, *loaded);
  EXPECT_EQ(std::distance(std::filesystem::directory_iterator(directory),
                          std::filesystem::directory_iterator()),
            1);
}

}  // namespace hydra
