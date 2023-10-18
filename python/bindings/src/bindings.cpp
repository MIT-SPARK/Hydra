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
#include <pybind11/pybind11.h>

#include "hydra/bindings/batch.h"
#include "hydra/bindings/config_bindings.h"
#include "hydra/bindings/glog_utilities.h"
#include "hydra/bindings/hydra_python_pipeline.h"
#include "hydra/bindings/python_config.h"
#include "hydra/bindings/python_image.h"
#include "hydra/bindings/python_reconstruction.h"
#include "hydra/bindings/python_sensor_input.h"

namespace py = pybind11;
using namespace py::literals;

PYBIND11_MODULE(_hydra_bindings, m) {
  py::module_::import("spark_dsg");
  py::options options;

  ::hydra::python::batch::addBindings(m);
  ::hydra::python::config_bindings::addBindings(m);
  ::hydra::python::glog_utilities::addBindings(m);
  ::hydra::python::hydra_python_pipeline::addBindings(m);
  ::hydra::python::python_config::addBindings(m);
  ::hydra::python::python_image::addBindings(m);
  ::hydra::python::python_sensor_input::addBindings(m);
  ::hydra::python::python_reconstruction::addBindings(m);

  py::class_<Eigen::Quaterniond>(m, "Quaterniond")
      .def(py::init([]() { return Eigen::Quaterniond::Identity(); }))
      .def(py::init([](double w, double x, double y, double z) {
             return Eigen::Quaterniond(w, x, y, z);
           }),
           "w"_a,
           "x"_a,
           "y"_a,
           "z"_a)
      .def(py::init([](const Eigen::Vector4d& coefficients) {
        return Eigen::Quaterniond(
            coefficients(0), coefficients(1), coefficients(2), coefficients(3));
      }))
      .def_property("w",
                    py::overload_cast<>(&Eigen::Quaterniond::w, py::const_),
                    [](Eigen::Quaterniond& q, double w) { q.w() = w; })
      .def_property("x",
                    py::overload_cast<>(&Eigen::Quaterniond::x, py::const_),
                    [](Eigen::Quaterniond& q, double x) { q.x() = x; })
      .def_property("y",
                    py::overload_cast<>(&Eigen::Quaterniond::y, py::const_),
                    [](Eigen::Quaterniond& q, double y) { q.y() = y; })
      .def_property("z",
                    py::overload_cast<>(&Eigen::Quaterniond::z, py::const_),
                    [](Eigen::Quaterniond& q, double z) { q.z() = z; })
      .def("__repr__", [](const Eigen::Quaterniond& q) {
        std::stringstream ss;
        ss << "Quaterniond<w=" << q.w() << ", x=" << q.x() << ", y=" << q.y()
           << ", z=" << q.z() << ">";
        return ss.str();
      });
}
