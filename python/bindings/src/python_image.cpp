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
#include "hydra/bindings/python_image.h"

#include <glog/logging.h>
#include <pybind11/stl.h>

#include "hydra/bindings/color_parser.h"
#include "hydra/bindings/depth_parser.h"
#include "hydra/bindings/label_parser.h"

namespace py = pybind11;

namespace hydra::python {

std::string getShapeStr(const std::vector<ssize_t>& shape) {
  std::stringstream ss;
  ss << "[";
  auto iter = shape.begin();
  while (iter != shape.end()) {
    ss << *iter;
    ++iter;
    if (iter != shape.end()) {
      ss << ", ";
    }
  }
  ss << "]";
  return ss.str();
}

PythonImage::PythonImage() : valid_(false), rows_(0), cols_(0), channels_(0) {}

PythonImage::PythonImage(const pybind11::buffer& img)
    : img_(img.request()), valid_(false), rows_(0), cols_(0), channels_(0) {
  std::vector<ssize_t> shape;
  std::vector<ssize_t> indices;
  for (ssize_t i = 0; i < img_.ndim; ++i) {
    const auto curr_shape = img_.shape.at(i);
    if (curr_shape == 1) {
      continue;
    }

    shape.push_back(curr_shape);
    indices.push_back(i);
  }

  if (shape.size() != 2 && shape.size() != 3) {
    LOG(ERROR) << "invalid (squeezed) image shape: " << getShapeStr(shape);
    return;
  }

  rows_ = shape[0];
  row_stride_ = img_.strides.at(indices[0]);
  cols_ = shape[1];
  col_stride_ = img_.strides.at(indices[1]);
  channels_ = shape.size() == 3 ? shape[2] : 1;
  channel_stride_ = shape.size() == 3 ? img_.strides.at(indices[2]) : 0;
  valid_ = true;
}

template <typename Parser>
cv::Mat getImage(const PythonImage& img) {
  if (!img) {
    return {};
  }

  const auto parser = Parser::create(img);
  if (!parser) {
    LOG(ERROR) << "could not create parser for image";
    return {};
  }

  cv::Mat mat(img.rows(), img.cols(), Parser::MatType);
  for (size_t r = 0; r < img.rows(); ++r) {
    for (size_t c = 0; c < img.cols(); ++c) {
      mat.at<typename Parser::Element>(r, c) = parser->read(img, r, c);
    }
  }

  return mat;
}

cv::Mat getLabelImage(const PythonImage& img) { return getImage<LabelParser>(img); }

cv::Mat getDepthImage(const PythonImage& img) { return getImage<DepthParser>(img); }

cv::Mat getColorImage(const PythonImage& img) { return getImage<ColorParser>(img); }

namespace python_image {

// binding for unit tests
struct CvImageWrapper {
  explicit CvImageWrapper(const cv::Mat& img) : mat(img) {}

  cv::Mat mat;
};

std::vector<ssize_t> getSizeVector(const cv::Mat& img) {
  const auto size = img.size;
  std::vector<ssize_t> sizes;
  for (int i = 0; i < size.dims(); ++i) {
    sizes.push_back(size[i]);
  }

  if (img.channels() != 1) {
    sizes.push_back(img.channels());
  }

  return sizes;
}

std::vector<ssize_t> getStrideVector(const cv::Mat& img) {
  const auto steps = img.step;
  std::vector<ssize_t> strides;
  for (int i = 0; i < img.dims; ++i) {
    strides.push_back(steps[i]);
  }

  if (img.channels() != 1) {
    strides.push_back(img.elemSize1());
  }
  return strides;
}

size_t getDims(const cv::Mat& img) {
  return img.channels() != 1 ? img.dims + 1 : img.dims;
}

// these bindings are directly for tests
void addBindings(py::module_& m) {
  // GCOVR_EXCL_START
  py::class_<CvImageWrapper>(m, "_HydraImage", py::buffer_protocol())
      .def_buffer([](const CvImageWrapper& img) -> py::buffer_info {
        std::string format;
        switch (img.mat.depth()) {
          case CV_8U:
            format = py::format_descriptor<uint8_t>::format();
            break;
          case CV_8S:
            format = py::format_descriptor<int8_t>::format();
            break;
          case CV_16U:
            format = py::format_descriptor<uint16_t>::format();
            break;
          case CV_16S:
            format = py::format_descriptor<int16_t>::format();
            break;
          case CV_32S:
            format = py::format_descriptor<int32_t>::format();
            break;
          case CV_32F:
            format = py::format_descriptor<float>::format();
            break;
          case CV_64F:
            format = py::format_descriptor<double>::format();
            break;
          default:
            throw std::runtime_error("unknown CV depth");
        }
        return py::buffer_info(img.mat.data,
                               img.mat.elemSize1(),
                               format,
                               getDims(img.mat),
                               getSizeVector(img.mat),
                               getStrideVector(img.mat));
      });
  // GCOVR_EXCL_STOP

  m.def("_get_color_image", [](const py::buffer& img) -> std::optional<CvImageWrapper> {
    const auto mat = getColorImage(img);
    if (mat.empty()) {
      return std::nullopt;
    }

    return CvImageWrapper(mat);
  });

  m.def("_get_label_image", [](const py::buffer& img) -> std::optional<CvImageWrapper> {
    const auto mat = getLabelImage(img);
    if (mat.empty()) {
      return std::nullopt;
    }

    return CvImageWrapper(mat);
  });

  m.def("_get_depth_image", [](const py::buffer& img) -> std::optional<CvImageWrapper> {
    const auto mat = getDepthImage(img);
    if (mat.empty()) {
      return std::nullopt;
    }

    return CvImageWrapper(mat);
  });
}
}  // namespace python_image

}  // namespace hydra::python
