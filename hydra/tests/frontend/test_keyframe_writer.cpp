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
#include <hydra/frontend/keyframe_writer.h>

#include <filesystem>
#include <fstream>
#include <nlohmann/json.hpp>
#include <opencv2/imgcodecs.hpp>

#include "hydra_test/temp_directory.h"

namespace hydra {
namespace {

nlohmann::json readJson(const std::filesystem::path& path) {
  std::ifstream file(path);
  return nlohmann::json::parse(file);
}

}  // namespace

TEST(KeyframeWriter, WritesRgbDepthMeta) {
  test::TempDirectory tmp;
  const auto dir = tmp.path / "frames";
  KeyframeWriter writer(dir, "subkf_");

  const cv::Mat color(4, 4, CV_8UC3, cv::Scalar(10, 20, 30));
  const cv::Mat depth(4, 4, CV_32FC1, cv::Scalar(1.5f));  // 1.5 m
  EXPECT_TRUE(writer.write(42, colorToKeyframe(color), depthToKeyframe(depth)));

  EXPECT_TRUE(std::filesystem::exists(dir / "subkf_42_rgb.jpg"));
  EXPECT_TRUE(std::filesystem::exists(dir / "subkf_42_depth.png"));
  EXPECT_TRUE(std::filesystem::exists(dir / "subkf_42_meta.json"));
  // the stored image folder is relative to the parent of the output directory
  EXPECT_EQ(writer.prefix(42), dir / "subkf_42");
  EXPECT_EQ(writer.imageFolder(42), "frames/subkf_42");

  // depth round-trips to 16-bit mm: 1.5 m -> 1500
  const auto loaded =
      cv::imread((dir / "subkf_42_depth.png").string(), cv::IMREAD_UNCHANGED);
  ASSERT_EQ(loaded.type(), CV_16UC1);
  EXPECT_EQ(loaded.at<uint16_t>(0, 0), 1500);

  // the pose is only part of the metadata when provided by the caller
  const auto meta = readJson(dir / "subkf_42_meta.json");
  EXPECT_EQ(meta.at("timestamp_ns").get<uint64_t>(), 42u);
  EXPECT_EQ(meta.at("rgb_file"), "subkf_42_rgb.jpg");
  EXPECT_EQ(meta.at("depth_file"), "subkf_42_depth.png");
  EXPECT_EQ(meta.at("calib"), "camera_calib.json");
  EXPECT_FALSE(meta.contains("world_T_body"));
}

TEST(KeyframeWriter, WritesCalibOnce) {
  test::TempDirectory tmp;
  KeyframeWriter writer(tmp.path, "agent_");

  CameraCalib calib;
  calib.fx = 500.0;
  calib.fy = 501.0;
  calib.cx = 320.0;
  calib.cy = 240.0;
  calib.width = 640;
  calib.height = 480;
  EXPECT_TRUE(writer.writeCalib(calib));

  const auto calib_path = tmp.path / "camera_calib.json";
  const auto contents = readJson(calib_path);
  EXPECT_DOUBLE_EQ(contents.at("fx").get<double>(), 500.0);
  EXPECT_EQ(contents.at("width").get<int>(), 640);
  EXPECT_DOUBLE_EQ(contents.at("depth_scale").get<double>(), 1.0e-3);
  EXPECT_EQ(contents.at("depth_encoding"), "16UC1_mm");
  EXPECT_EQ(contents.at("body_T_sensor").size(), 16u);

  std::filesystem::remove(calib_path);
  EXPECT_TRUE(writer.writeCalib(calib));
  EXPECT_FALSE(std::filesystem::exists(calib_path));
}

TEST(KeyframeWriter, WritesExtraMetadata) {
  test::TempDirectory tmp;
  KeyframeWriter writer(tmp.path, "agent_");

  Eigen::Isometry3d world_T_body = Eigen::Isometry3d::Identity();
  world_T_body.translation() << 1.0, 2.0, 3.0;
  const nlohmann::json extra{{"world_T_body", isometryToJson(world_T_body)}};
  EXPECT_TRUE(writer.write(7, cv::Mat(), cv::Mat(), extra));

  EXPECT_FALSE(std::filesystem::exists(tmp.path / "agent_7_rgb.jpg"));
  const auto meta = readJson(tmp.path / "agent_7_meta.json");
  EXPECT_FALSE(meta.contains("rgb_file"));
  const auto pose = meta.at("world_T_body").get<std::vector<double>>();
  const std::vector<double> expected{1, 0, 0, 1, 0, 1, 0, 2, 0, 0, 1, 3, 0, 0, 0, 1};
  EXPECT_EQ(pose, expected);
}

TEST(KeyframeWriter, ReportsWriteFailures) {
  test::TempDirectory tmp;
  KeyframeWriter writer(tmp.path / "missing" / "frames", "agent_");
  std::filesystem::remove_all(tmp.path / "missing");
  EXPECT_FALSE(writer.write(7, cv::Mat(4, 4, CV_8UC3), cv::Mat()));
}

}  // namespace hydra
