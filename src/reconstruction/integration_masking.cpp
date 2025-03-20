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
#include "hydra/reconstruction/integration_masking.h"

#include <glog/logging.h>

#include <opencv2/core.hpp>

namespace hydra {
namespace {

inline std::string showImageDim(const cv::Mat& mat) {
  std::stringstream ss;
  ss << "[rows=" << mat.rows << ", cols=" << mat.cols << "]";
  return ss.str();
}

inline bool initMask(const cv::Mat& input, cv::Mat& mask) {
  if (mask.empty()) {
    mask = cv::Mat::zeros(input.rows, input.cols, CV_32SC1);
    return true;
  }

  if (mask.rows != input.rows || mask.cols != input.cols) {
    LOG(ERROR) << "Dimensions do not match! input: " << showImageDim(input)
               << " vs. mask: " << showImageDim(mask);
    return false;
  }

  if (mask.type() != CV_32SC1) {
    LOG(WARNING) << "Invalid mask type! Must be CV_32SC!";

    cv::Mat converted;
    mask.convertTo(converted, CV_32SC1);
    mask = converted;
  }

  return true;
}

}  // namespace

bool maskInvalidSemantics(const cv::Mat& labels,
                          const std::set<int32_t> to_mask,
                          cv::Mat& mask) {
  if (labels.empty() || to_mask.empty()) {
    return true;  // more efficient to not init mask if it is going to be empty
  }

  if (labels.type() != CV_32SC1) {
    LOG(ERROR) << "Invalid label image! Type must be CV_32SC1";
    return false;
  }

  if (!initMask(labels, mask)) {
    return false;
  }

  for (int r = 0; r < labels.rows; ++r) {
    for (int c = 0; c < labels.cols; ++c) {
      const auto label = labels.at<int32_t>(r, c);
      mask.at<int32_t>(r, c) |= to_mask.count(label);
    }
  }

  return true;
}

bool maskNonZero(const cv::Mat input, cv::Mat& mask) {
  if (input.empty()) {
    return true;  // more efficient to not init mask if it is going to be empty
  }

  // if opencv implemented bitwise_or across integer types, we could just use that, but
  // this is likely more efficient than converting the entire input
  // TODO(nathan) port to label remapper as well
  std::function<bool(const cv::Mat&, int, int)> input_getter;
  switch (input.depth()) {
    case CV_8U:
      input_getter = [](const cv::Mat& mat, int r, int c) -> bool {
        return mat.at<int8_t>(r, c);
      };
      break;
    case CV_8S:
      input_getter = [](const cv::Mat& mat, int r, int c) -> bool {
        return mat.at<uint8_t>(r, c);
      };
      break;
    case CV_16U:
      input_getter = [](const cv::Mat& mat, int r, int c) -> bool {
        return mat.at<uint16_t>(r, c);
      };
      break;
    case CV_16S:
      input_getter = [](const cv::Mat& mat, int r, int c) -> bool {
        return mat.at<int16_t>(r, c);
      };
      break;
    case CV_32S:
      input_getter = [](const cv::Mat& mat, int r, int c) -> bool {
        return mat.at<int32_t>(r, c);
      };
      break;
    default:
      LOG(ERROR) << "Unhandled depth: " << input.depth();
      return false;
  }

  if (!initMask(input, mask)) {
    return false;
  }

  for (int r = 0; r < input.rows; ++r) {
    for (int c = 0; c < input.cols; ++c) {
      mask.at<int32_t>(r, c) |= input_getter(input, r, c);
    }
  }

  return true;
}

}  // namespace hydra
