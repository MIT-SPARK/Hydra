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
#include <cstdint>
#include <filesystem>
#include <nlohmann/json.hpp>
#include <opencv2/core/mat.hpp>
#include <string>

#include "hydra/utils/image_folder.h"

namespace hydra {

class Camera;

//! Pinhole calibration of the camera that keyframe images come from
struct CameraCalib {
  double fx = 0.0;
  double fy = 0.0;
  double cx = 0.0;
  double cy = 0.0;
  int width = 0;
  int height = 0;
  Eigen::Isometry3d body_T_sensor = Eigen::Isometry3d::Identity();

  static CameraCalib fromCamera(const Camera& camera);
};

//! Depth images are stored as 16-bit PNGs in millimeters: depth_scale [m] per unit
inline constexpr double kKeyframeDepthScale = 1.0e-3;
inline constexpr char kKeyframeDepthEncoding[] = "16UC1_mm";

//! @brief Serialize a transform as a flat row-major 4x4 array
nlohmann::json isometryToJson(const Eigen::Isometry3d& transform);

//! @brief Copy a color image (RGB) into the format it is written in (BGR)
cv::Mat colorToKeyframe(const cv::Mat& color_rgb);

//! @brief Copy a metric depth image (CV_32FC1) into the format it is written in (see
//! kKeyframeDepthScale). Other image types are not supported and result in an empty
//! image
cv::Mat depthToKeyframe(const cv::Mat& depth_m);

//! @brief Write an image, logging failures
bool writeImage(const std::filesystem::path& path, const cv::Mat& image);

//! @brief Write JSON, logging failures
bool writeJson(const std::filesystem::path& path, const nlohmann::json& contents);

//! @brief Write `camera_calib.json` (intrinsics, depth encoding and body_T_sensor)
bool writeCameraCalib(const std::filesystem::path& output_dir,
                      const CameraCalib& calib);

/**
 * @brief Writes keyframe images and metadata to a directory.
 *
 * Files are named `<file_prefix><timestamp_ns>_{rgb.jpg,depth.png,meta.json}`. The
 * calibration shared by all keyframes is written to `camera_calib.json` and
 * referenced from the metadata. Image folder values of keyframes are their file
 * prefix relative to the parent of the output directory (see imageFolderBase), e.g.,
 * `agents/agent_<timestamp_ns>`.
 */
class KeyframeWriter {
 public:
  /**
   * @param output_dir Directory to write to (created if missing)
   * @param file_prefix Prefix of all file names, e.g., `agent_`
   * @param calib_file Calibration file referenced by the metadata (relative to the
   * metadata file)
   */
  KeyframeWriter(const std::filesystem::path& output_dir,
                 const std::string& file_prefix,
                 const std::string& calib_file = utils::kCameraCalibFile);

  //! @brief Write the calibration to the output directory unless already written
  bool writeCalib(const CameraCalib& calib);

  /**
   * @brief Write one keyframe.
   * @param timestamp_ns Timestamp of the keyframe (used for the file names)
   * @param color Color image in the format it is written in (empty skips the image)
   * @param depth Depth image in the format it is written in (empty skips the image)
   * @param meta Additional metadata (the timestamp and file names are added)
   * @returns Whether all files were written
   */
  bool write(uint64_t timestamp_ns,
             const cv::Mat& color,
             const cv::Mat& depth,
             nlohmann::json meta = nlohmann::json::object()) const;

  //! @brief File prefix (path without suffixes) of the files for a timestamp
  std::filesystem::path prefix(uint64_t timestamp_ns) const;

  //! @brief Image folder value to store for a timestamp
  std::string imageFolder(uint64_t timestamp_ns) const;

 private:
  const std::filesystem::path output_dir_;
  const std::string file_prefix_;
  const std::string calib_file_;
  bool calib_written_ = false;
};

}  // namespace hydra
