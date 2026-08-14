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
#include "hydra/bindings/python_sensors.h"

#include <hydra/input/camera.h>
#include <hydra/input/lidar.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>

namespace hydra::python::python_sensors {

using namespace pybind11::literals;
namespace py = pybind11;

void addBindings(pybind11::module_& m) {
  py::class_<ParamSensorExtrinsics::Config>(m, "ExtrinsicsConfig")
      .def(py::init<>())
      .def_readwrite("body_R_sensor", &ParamSensorExtrinsics::Config::body_R_sensor)
      .def_readwrite("body_p_sensor", &ParamSensorExtrinsics::Config::body_p_sensor);

  py::class_<Sensor::Config>(m, "SensorConfig")
      .def_readwrite("min_range", &Sensor::Config::min_range)
      .def_readwrite("max_range", &Sensor::Config::max_range)
      .def_property(
          "extrinsics",
          [](const Sensor::Config& config) {
            if (!config.extrinsics) {
              return ParamSensorExtrinsics::Config();
            }

            const auto extrinsics = config.extrinsics.create();
            return ParamSensorExtrinsics::Config{extrinsics->body_R_sensor,
                                                 extrinsics->body_p_sensor};
          },
          [](Sensor::Config& config, const ParamSensorExtrinsics::Config& extrinsics) {
            config.extrinsics = extrinsics;
          });

  py::class_<Camera::Config, Sensor::Config>(m, "CameraConfig")
      .def(py::init<>())
      .def_readwrite("width", &Camera::Config::width)
      .def_readwrite("height", &Camera::Config::height)
      .def_readwrite("cx", &Camera::Config::cx)
      .def_readwrite("cy", &Camera::Config::cy)
      .def_readwrite("fx", &Camera::Config::fx)
      .def_readwrite("fy", &Camera::Config::fy);

  py::class_<Lidar::Config, Sensor::Config>(m, "LidarConfig")
      .def(py::init<>())
      .def_readwrite("horizontal_resolution", &Lidar::Config::horizontal_resolution)
      .def_readwrite("vertical_resolution", &Lidar::Config::vertical_resolution)
      .def_readwrite("horizontal_fov", &Lidar::Config::horizontal_fov)
      .def_readwrite("vertical_fov", &Lidar::Config::vertical_fov)
      .def_readwrite("is_asymmetric", &Lidar::Config::is_asymmetric)
      .def_readwrite("vertical_fov_top", &Lidar::Config::vertical_fov_top);

  py::class_<Sensor, std::shared_ptr<Sensor>> sensor(m, "Sensor");
  sensor.def_readonly("config", &Sensor::config)
      .def("min_range", &Sensor::min_range)
      .def("max_range", &Sensor::max_range)
      .def("body_T_sensor",
           [](const Sensor& sensor) { return sensor.body_T_sensor().matrix(); })
      .def("compute_ray_density", &Sensor::computeRayDensity)
      .def("project_point",
           [](const Sensor& sensor,
              const Eigen::Vector3f& p_xyz) -> std::optional<Eigen::Vector2f> {
             float u, v;
             if (!sensor.projectPointToImagePlane(p_xyz, u, v)) {
               return std::nullopt;
             }

             Eigen::Vector2f p_uv;
             p_uv << u, v;
             return p_uv;
           })
      .def("point_in_view_frustum", &Sensor::pointIsInViewFrustum)
      .def_readonly("name", &Sensor::name);

  py::class_<Camera, std::shared_ptr<Camera>>(m, "Camera", sensor)
      .def(py::init<const Camera::Config&, const std::string&>())
      .def_property_readonly("config",
                             [](const Camera& camera) { return camera.getConfig(); });

  py::class_<Lidar, std::shared_ptr<Lidar>>(m, "Lidar", sensor)
      .def(py::init<const Lidar::Config&, const std::string&>())
      .def_property_readonly("config",
                             [](const Lidar& lidar) { return lidar.getConfig(); });
}

}  // namespace hydra::python::python_sensors
