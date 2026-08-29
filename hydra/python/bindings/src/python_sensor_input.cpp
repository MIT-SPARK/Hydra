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

PythonImageInput::PythonImageInput(uint64_t timestamp_ns,
                                   const std::string& sensor_name,
                                   const PythonImage& _color,
                                   const PythonImage& _depth,
                                   const PythonImage& _labels,
                                   const PythonImage& _instances)
    : ImageInputPacket(timestamp_ns, sensor_name) {
  depth = getDepthImage(_depth);
  color = getColorImage(_color);
  labels = getLabelImage(_labels);
  instances = getLabelImage(_instances);
}

PythonCloudInput::PythonCloudInput(uint64_t timestamp_ns,
                                   const std::string& sensor_name,
                                   const PointVec& pos_vec,
                                   const LabelVec& label_vec,
                                   const ColorVec& color_vec)
    : CloudInputPacket(timestamp_ns, sensor_name) {
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
    colors = cv::Mat(1, pos_vec.cols(), CV_8UC3);
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
      colors.at<cv::Vec3b>(0, i) = {rgb.x(), rgb.y(), rgb.z()};
    }

    if (!labels_empty) {
      labels.at<int32_t>(0, i) = label_vec(0, i);
    }
  }
}

namespace python_sensor_input {
void addBindings(pybind11::module_&) {}
}  // namespace python_sensor_input

}  // namespace hydra::python
