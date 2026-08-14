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
#include "hydra/bindings/color_parser.h"

#include <glog/logging.h>

namespace py = pybind11;

namespace hydra::python {

template <typename T>
struct MonoParserImpl : ColorParser {
  cv::Vec3b read(const PythonImage& img, size_t row, size_t col) const override;
};

template <>
cv::Vec3b MonoParserImpl<uint8_t>::read(const PythonImage& img,
                                        size_t row,
                                        size_t col) const {
  uint8_t value;
  std::memcpy(&value, img.ptr(row, col), sizeof(value));
  return {value, value, value};
}

template <>
cv::Vec3b MonoParserImpl<uint16_t>::read(const PythonImage& img,
                                         size_t row,
                                         size_t col) const {
  uint16_t value;
  std::memcpy(&value, img.ptr(row, col), sizeof(value));
  const auto int_value = cv::saturate_cast<uint8_t>(value / 255);
  return {int_value, int_value, int_value};
}

template <>
cv::Vec3b MonoParserImpl<float>::read(const PythonImage& img,
                                      size_t row,
                                      size_t col) const {
  float value;
  std::memcpy(&value, img.ptr(row, col), sizeof(value));
  const auto int_value = cv::saturate_cast<uint8_t>(value * 255);
  return {int_value, int_value, int_value};
}

template <typename T>
struct RgbParserImpl : ColorParser {
  cv::Vec3b read(const PythonImage& img, size_t row, size_t col) const override;
};

template <typename T>
T readPixelChannel(const PythonImage& img, size_t row, size_t col, size_t channel) {
  T value;
  std::memcpy(&value, img.ptr(row, col, channel), sizeof(value));
  return value;
}

template <>
cv::Vec3b RgbParserImpl<uint8_t>::read(const PythonImage& img,
                                       size_t row,
                                       size_t col) const {
  cv::Vec3b pixel;
  pixel[0] = readPixelChannel<uint8_t>(img, row, col, 0);
  pixel[1] = readPixelChannel<uint8_t>(img, row, col, 1);
  pixel[2] = readPixelChannel<uint8_t>(img, row, col, 2);
  return pixel;
}

template <>
cv::Vec3b RgbParserImpl<float>::read(const PythonImage& img,
                                     size_t row,
                                     size_t col) const {
  cv::Vec3b p;
  p[0] = cv::saturate_cast<uint8_t>(255 * readPixelChannel<float>(img, row, col, 0));
  p[1] = cv::saturate_cast<uint8_t>(255 * readPixelChannel<float>(img, row, col, 1));
  p[2] = cv::saturate_cast<uint8_t>(255 * readPixelChannel<float>(img, row, col, 2));
  return p;
}

ColorParser::Ptr ColorParser::create(const PythonImage& img) {
  CHECK(img) << "invalid image";
  if (img.channels() != 1 && img.channels() != 3) {
    LOG(ERROR) << "only single-channel or 3 channel color images supported";
    return nullptr;
  }

  const auto& img_format = img.format();
  if (img.channels() == 1) {
    if (img_format == py::format_descriptor<uint8_t>::format()) {
      return std::make_unique<MonoParserImpl<uint8_t>>();
    } else if (img_format == py::format_descriptor<uint16_t>::format()) {
      return std::make_unique<MonoParserImpl<uint16_t>>();
    } else if (img_format == py::format_descriptor<float>::format()) {
      return std::make_unique<MonoParserImpl<float>>();
    } else {
      LOG(ERROR) << "invalid color format for mono image: " << img_format;
      return nullptr;
    }
  }

  if (img_format == py::format_descriptor<uint8_t>::format()) {
    return std::make_unique<RgbParserImpl<uint8_t>>();
  } else if (img_format == py::format_descriptor<float>::format()) {
    return std::make_unique<RgbParserImpl<float>>();
  } else {
    LOG(ERROR) << "invalid color format for rgb image: " << img_format;
    return nullptr;
  }
}

}  // namespace hydra::python
