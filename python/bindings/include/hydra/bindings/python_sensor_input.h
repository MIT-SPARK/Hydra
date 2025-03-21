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
#include <hydra/input/sensor_input_packet.h>
#include <pybind11/pybind11.h>

#include <Eigen/Dense>

#include "hydra/bindings/python_image.h"

namespace hydra::python {

struct PythonSensorInput : public SensorInputPacket {
  using PointVec = Eigen::Matrix<double, 3, Eigen::Dynamic>;
  using LabelVec = Eigen::Matrix<int32_t, 1, Eigen::Dynamic>;
  using ColorVec = Eigen::Matrix<uint8_t, 3, Eigen::Dynamic>;

  PythonSensorInput(uint64_t timestamp_ns,
                    const PythonImage& depth,
                    const PythonImage& labels,
                    const PythonImage& color,
                    const std::string& name);

  PythonSensorInput(uint64_t timestamp_ns,
                    const PointVec& points,
                    const LabelVec& labels,
                    const ColorVec& colors,
                    const std::string& name);

  bool valid() const;

  cv::Mat points;
  cv::Mat depth;
  cv::Mat labels;
  cv::Mat color;

 protected:
  bool fillInputDataImpl(InputData& msg) const override;
};

namespace python_sensor_input {
void addBindings(pybind11::module_& module);
}  // namespace python_sensor_input

}  // namespace hydra::python
