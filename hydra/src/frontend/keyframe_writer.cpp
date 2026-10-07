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
#include "hydra/frontend/keyframe_writer.h"

#include <glog/logging.h>

#include <fstream>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "hydra/input/camera.h"

namespace hydra {

CameraCalib CameraCalib::fromCamera(const Camera& camera) {
  const auto& config = camera.getConfig();
  CameraCalib calib;
  calib.fx = config.fx;
  calib.fy = config.fy;
  calib.cx = config.cx;
  calib.cy = config.cy;
  calib.width = config.width;
  calib.height = config.height;
  calib.body_T_sensor = camera.body_T_sensor();
  return calib;
}

nlohmann::json isometryToJson(const Eigen::Isometry3d& transform) {
  const Eigen::Matrix4d m = transform.matrix();
  auto values = nlohmann::json::array();
  for (int r = 0; r < 4; ++r) {
    for (int c = 0; c < 4; ++c) {
      values.push_back(m(r, c));
    }
  }

  return values;
}

cv::Mat colorToKeyframe(const cv::Mat& color_rgb) {
  cv::Mat color;
  if (color_rgb.channels() == 3) {
    cv::cvtColor(color_rgb, color, cv::COLOR_RGB2BGR);
  } else {
    color = color_rgb.clone();
  }

  return color;
}

cv::Mat depthToKeyframe(const cv::Mat& depth_m) {
  cv::Mat depth;
  if (depth_m.empty()) {
    return depth;
  }

  if (depth_m.type() != CV_32FC1) {
    LOG_FIRST_N(WARNING, 1) << "Unsupported keyframe depth type " << depth_m.type()
                            << " (expected metric CV_32FC1)";
    return depth;
  }

  // PNG cannot store floats
  depth_m.convertTo(depth, CV_16UC1, 1.0 / kKeyframeDepthScale);
  return depth;
}

bool writeImage(const std::filesystem::path& path, const cv::Mat& image) {
  try {
    if (cv::imwrite(path.string(), image)) {
      return true;
    }
  } catch (const cv::Exception& e) {
    LOG(WARNING) << "Failed to write " << path << ": " << e.what();
    return false;
  }

  LOG(WARNING) << "Failed to write " << path;
  return false;
}

bool writeJson(const std::filesystem::path& path, const nlohmann::json& contents) {
  std::ofstream out(path);
  out << contents.dump(2) << std::endl;
  if (!out) {
    LOG(WARNING) << "Failed to write " << path;
    return false;
  }

  return true;
}

bool writeCameraCalib(const std::filesystem::path& output_dir,
                      const CameraCalib& calib) {
  const nlohmann::json contents{{"fx", calib.fx},
                                {"fy", calib.fy},
                                {"cx", calib.cx},
                                {"cy", calib.cy},
                                {"width", calib.width},
                                {"height", calib.height},
                                {"depth_scale", kKeyframeDepthScale},
                                {"depth_encoding", kKeyframeDepthEncoding},
                                {"body_T_sensor", isometryToJson(calib.body_T_sensor)}};
  return writeJson(output_dir / utils::kCameraCalibFile, contents);
}

KeyframeWriter::KeyframeWriter(const std::filesystem::path& output_dir,
                               const std::string& file_prefix,
                               const std::string& calib_file)
    : output_dir_(output_dir), file_prefix_(file_prefix), calib_file_(calib_file) {
  std::error_code ec;
  if (!output_dir_.empty() && !std::filesystem::create_directories(output_dir_, ec) &&
      ec) {
    LOG(WARNING) << "Failed to create " << output_dir_ << ": " << ec.message();
  }
}

bool KeyframeWriter::writeCalib(const CameraCalib& calib) {
  if (!calib_written_) {
    calib_written_ = writeCameraCalib(output_dir_, calib);
  }

  return calib_written_;
}

std::filesystem::path KeyframeWriter::prefix(uint64_t timestamp_ns) const {
  return output_dir_ / utils::keyframeStem(file_prefix_, timestamp_ns);
}

std::string KeyframeWriter::imageFolder(uint64_t timestamp_ns) const {
  return utils::relativeImageFolder(output_dir_, prefix(timestamp_ns));
}

bool KeyframeWriter::write(uint64_t timestamp_ns,
                           const cv::Mat& color,
                           const cv::Mat& depth,
                           nlohmann::json meta) const {
  const auto stem = utils::keyframeStem(file_prefix_, timestamp_ns);
  const auto base = (output_dir_ / stem).string();
  bool valid = true;
  meta["timestamp_ns"] = timestamp_ns;
  if (!color.empty()) {
    valid &= writeImage(base + utils::kKeyframeRgbSuffix, color);
    meta["rgb_file"] = stem + utils::kKeyframeRgbSuffix;
  }

  if (!depth.empty()) {
    valid &= writeImage(base + utils::kKeyframeDepthSuffix, depth);
    meta["depth_file"] = stem + utils::kKeyframeDepthSuffix;
  }

  meta["calib"] = calib_file_;
  return writeJson(base + utils::kKeyframeMetaSuffix, meta) && valid;
}

}  // namespace hydra
