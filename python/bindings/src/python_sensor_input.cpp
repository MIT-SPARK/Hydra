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
#include "hydra/bindings/python_sensor_input.h"

#include <glog/logging.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>

namespace hydra::python {

std::string showDim(const PythonImage& img) {
  return "[" + std::to_string(img.rows()) + ", " + std::to_string(img.cols()) + "]";
}

PythonSensorInput::PythonSensorInput(uint64_t timestamp_ns,
                                     const PythonImage& _depth,
                                     const PythonImage& _labels,
                                     const PythonImage& _color,
                                     size_t sensor_index)
    : SensorInputPacket(timestamp_ns, sensor_index) {
  depth = getDepthImage(_depth);

  if (_color.rows() == _depth.rows() && _color.cols() == _depth.cols()) {
    color = getColorImage(_color);
  } else if (_color) {
    LOG(ERROR) << "depth dimensions " << showDim(_depth)
               << " do not match color dimensions " << showDim(_color);
  }

  if (_labels.rows() == _depth.rows() && _labels.cols() == _depth.cols()) {
    labels = getLabelImage(_labels);
  } else if (_labels) {
    LOG(ERROR) << "depth dimensions " << showDim(_depth)
               << " do not match label dimensions " << showDim(_labels);
  }
}

PythonSensorInput::PythonSensorInput(uint64_t timestamp_ns,
                                     const PointVec& pos_vec,
                                     const LabelVec& label_vec,
                                     const ColorVec& color_vec,
                                     size_t sensor_index)
    : SensorInputPacket(timestamp_ns, sensor_index) {
  if (pos_vec.cols() == 0) {
    LOG(ERROR) << "received input without any points";
    return;
  }

  bool colors_empty = color_vec.cols() == 0;
  if (!colors_empty && color_vec.cols() != pos_vec.cols()) {
    LOG(ERROR) << "received a different number of colors than points";
    colors_empty = true;
  }

  bool labels_empty = label_vec.cols() == 0;
  if (!labels_empty && label_vec.cols() != pos_vec.cols()) {
    LOG(ERROR) << "received a different number of labels than points";
    labels_empty = true;
  }

  points = cv::Mat(1, pos_vec.cols(), CV_32FC3);
  if (!colors_empty) {
    color = cv::Mat(1, pos_vec.cols(), CV_8UC3);
  }
  if (!labels_empty) {
    labels = cv::Mat(1, pos_vec.cols(), CV_32SC1);
  }

  for (int i = 0; i < pos_vec.cols(); ++i) {
    const auto& point = pos_vec.block<3, 1>(0, i);
    points.at<cv::Vec3f>(0, i) = {static_cast<float>(point.x()),
                                  static_cast<float>(point.y()),
                                  static_cast<float>(point.z())};

    if (!colors_empty) {
      const auto& rgb = color_vec.block<3, 1>(0, i);
      color.at<cv::Vec3b>(0, i) = {rgb.x(), rgb.y(), rgb.z()};
    }

    if (!labels_empty) {
      labels.at<int32_t>(0, i) = label_vec(0, i);
    }
  }
}

bool PythonSensorInput::fillInputData(InputData& msg) const {
  if ((depth.empty() && points.empty()) || (color.empty() && labels.empty())) {
    LOG(ERROR) << "Missing required data";
    return false;
  }

  msg.timestamp_ns = timestamp_ns;
  msg.vertex_map = points;
  msg.points_in_world_frame = false;
  msg.color_image = color;
  msg.depth_image = depth;
  msg.label_image = labels;
  return true;
}

bool PythonSensorInput::valid() const {
  return (!points.empty() || !depth.empty()) && (!color.empty() || !labels.empty());
}

namespace python_sensor_input {

using namespace pybind11::literals;
namespace py = pybind11;

void addBindings(pybind11::module_& m) {
  py::class_<PythonSensorInput>(m, "PythonSensorInput")
      .def_static(
          "from_images",
          [](uint64_t timestamp_ns,
             const py::buffer& depth,
             const std::optional<py::buffer>& labels,
             const std::optional<py::buffer>& colors,
             size_t sensor_index) {
            return PythonSensorInput(
                timestamp_ns,
                depth,
                labels ? PythonImage(labels.value()) : PythonImage(),
                colors ? PythonImage(colors.value()) : PythonImage(),
                sensor_index);
          },
          "timestamp_ns"_a,
          "depth"_a,
          "labels"_a = std::nullopt,
          "colors"_a = std::nullopt,
          "sensor_index"_a = 0)
      .def_static(
          "from_points",
          [](uint64_t timestamp_ns,
             const PythonSensorInput::PointVec& points,
             const PythonSensorInput::LabelVec& labels,
             const PythonSensorInput::ColorVec& colors,
             size_t sensor_index) {
            return PythonSensorInput(
                timestamp_ns, points, labels, colors, sensor_index);
          },
          "timestamp_ns"_a,
          "points"_a,
          "labels"_a = Eigen::Matrix<int32_t, 1, Eigen::Dynamic>(),
          "colors"_a = Eigen::Matrix<uint8_t, 3, Eigen::Dynamic>(),
          "sensor_index"_a = 0)
      .def("__bool__", [](const PythonSensorInput& input) { return input.valid(); });
}

}  // namespace python_sensor_input

}  // namespace hydra::python
