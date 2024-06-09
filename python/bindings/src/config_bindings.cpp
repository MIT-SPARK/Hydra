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
#include "hydra/bindings/config_bindings.h"

#include <config_utilities/config.h>
#include <config_utilities/parsing/yaml.h>
#include <hydra/common/global_info.h>
#include <pybind11/stl.h>
#include <pybind11/stl/filesystem.h>

#include "hydra/bindings/python_config.h"

namespace hydra::python {

namespace config_bindings {

using namespace pybind11::literals;
namespace py = pybind11;

void addBindings(pybind11::module_& m) {
  // GCOVR_EXCL_START
  py::class_<LogConfig>(m, "LogConfig")
      .def(py::init<>())
      .def_readwrite("log_dir", &LogConfig::log_dir)
      .def_readwrite("log_timing_incrementally", &LogConfig::log_timing_incrementally)
      .def_readwrite("timing_stats_name", &LogConfig::timing_stats_name)
      .def_readwrite("timing_suffix", &LogConfig::timing_suffix);

  py::class_<FrameConfig>(m, "FrameConfig")
      .def(py::init<>())
      .def_readwrite("robot", &FrameConfig::robot)
      .def_readwrite("odom", &FrameConfig::odom)
      .def_readwrite("map", &FrameConfig::map);

  py::class_<VolumetricMap::Config>(m, "MapConfig")
      .def(py::init<>())
      .def_readwrite("voxel_size", &VolumetricMap::Config::voxel_size)
      .def_readwrite("voxels_per_side", &VolumetricMap::Config::voxels_per_side)
      .def_readwrite("truncation_distance",
                     &VolumetricMap::Config::truncation_distance);

  py::class_<LabelSpaceConfig>(m, "LabelSpaceConfig")
      .def(py::init<>())
      .def_readwrite("total_labels", &LabelSpaceConfig::total_labels)
      .def_readwrite("colormap_filepath", &LabelSpaceConfig::colormap_filepath)
      .def_readwrite("dynamic_labels", &LabelSpaceConfig::dynamic_labels)
      .def_readwrite("invalid_labels", &LabelSpaceConfig::invalid_labels)
      .def_readwrite("colormap", &LabelSpaceConfig::colormap);

  py::class_<PipelineConfig>(m, "PipelineConfig")
      .def(py::init([](const PythonConfig& config) {
        const auto node = config.toYaml();
        return config::fromYaml<PipelineConfig>(node);
      }))
      .def_readwrite("enable_reconstruction", &PipelineConfig::enable_reconstruction)
      .def_readwrite("enable_lcd", &PipelineConfig::enable_lcd)
      .def_readwrite("timing_disabled", &PipelineConfig::timing_disabled)
      .def_readwrite("disable_timer_output", &PipelineConfig::disable_timer_output)
      .def_readwrite("layer_id_map", &PipelineConfig::layer_id_map)
      .def_readwrite("logs", &PipelineConfig::logs)
      .def_readwrite("frames", &PipelineConfig::frames)
      .def_readwrite("map", &PipelineConfig::map)
      .def_readwrite("label_space", &PipelineConfig::label_space)
      .def_readwrite("label_names", &PipelineConfig::label_names)
      .def_readwrite("room_colors", &PipelineConfig::room_colors);
  // GCOVR_EXCL_STOP
}

}  // namespace config_bindings

}  // namespace hydra::python
