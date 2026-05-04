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
#include "hydra/bindings/python_batch.h"

#include <config_utilities/config.h>
#include <config_utilities/formatting/asl.h>
#include <config_utilities/logging/log_to_glog.h>
#include <config_utilities/parsing/yaml.h>
#include <config_utilities/validation.h>
#include <hydra/common/batch_pipeline.h>
#include <hydra/common/global_info.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pybind11/stl/filesystem.h>

#include <limits>

#include "hydra/bindings/glog_utilities.h"

namespace hydra::python {

class PythonBatchPipeline : public BatchPipeline {
 public:
  struct Config : PipelineConfig {
    config::VirtualConfig<GraphBuilder> frontend;
    RoomFinderConfig room_finder;
  } const config;

  PythonBatchPipeline(const Config& config, int robot_id = 0);
  virtual ~PythonBatchPipeline() = default;
  DynamicSceneGraph::Ptr construct(const VolumetricMap& map) const;
};

void declare_config(PythonBatchPipeline::Config& config) {
  using namespace config;
  name("PythonBatchPipeline::Config");
  base<PipelineConfig>(config);
  field(config.frontend, "frontend");
  field(config.room_finder, "backend/room_finder");
}

PythonBatchPipeline::PythonBatchPipeline(const Config& config, int robot_id)
    : BatchPipeline(config, robot_id) {
  GlobalInfo::init(config, robot_id);
  GlogSingleton::instance().setLogLevel(0, 0, false);
}

DynamicSceneGraph::Ptr PythonBatchPipeline::construct(const VolumetricMap& map) const {
  const auto new_map = map.clone();
  return BatchPipeline::construct(config.frontend, *new_map, &config.room_finder);
}

namespace python_batch {

using namespace pybind11::literals;
namespace py = pybind11;

void addBindings(pybind11::module_& m) {
  py::class_<VolumetricMap>(m, "VolumetricMap")
      .def_static(
          "load",
          [](const std::string& filepath) { return VolumetricMap::load(filepath); })
      .def_static("load",
                  [](const std::filesystem::path& filepath) {
                    return VolumetricMap::load(filepath.string());
                  })
      .def_property_readonly("voxel_size",
                             [](const VolumetricMap& m) { return m.config.voxel_size; })
      .def_property_readonly("block_size",
                             [](const VolumetricMap& m) { return m.blockSize(); })
      .def_property_readonly("truncation_distance",
                             [](const VolumetricMap& m) {
                               return m.config.truncation_distance;
                             })
      .def("get_active_window_bounds",
           [](const VolumetricMap& m) -> py::object {
             const auto indices = m.getTsdfLayer().allocatedBlockIndices();
             if (indices.empty()) {
               return py::none();
             }
             const float bs = m.blockSize();
             Eigen::Vector3f mn =
                 Eigen::Vector3f::Constant(std::numeric_limits<float>::max());
             Eigen::Vector3f mx =
                 Eigen::Vector3f::Constant(std::numeric_limits<float>::lowest());
             for (const auto& idx : indices) {
               Eigen::Vector3f origin(idx.x() * bs, idx.y() * bs, idx.z() * bs);
               mn = mn.cwiseMin(origin);
               mx = mx.cwiseMax(origin + Eigen::Vector3f::Constant(bs));
             }
             return py::make_tuple(mn, mx);
           })
      .def("get_active_block_indices",
           [](const VolumetricMap& m) {
             const auto indices = m.getTsdfLayer().allocatedBlockIndices();
             std::vector<std::tuple<int, int, int>> result;
             result.reserve(indices.size());
             for (const auto& idx : indices) {
               result.emplace_back(idx.x(), idx.y(), idx.z());
             }
             return result;
           })
      .def("get_voxel_semantics_at",
           [](const VolumetricMap& m, Eigen::Vector3f pos) -> py::object {
             if (!m.hasSemantics()) {
               return py::none();
             }
             const auto* layer = m.getSemanticLayer();
             const SemanticVoxel* vox = layer->getVoxelPtr(pos);
             if (!vox || vox->empty) {
               return py::none();
             }
             py::dict out;
             out["mle_label"] = vox->semantic_label;
             out["weights"] =
                 std::vector<float>(vox->semantic_likelihoods.data(),
                                    vox->semantic_likelihoods.data() +
                                        vox->semantic_likelihoods.size());
             out["label_ids"] =
                 std::vector<uint32_t>(vox->semantic_labels.data(),
                                       vox->semantic_labels.data() +
                                           vox->semantic_labels.size());
             return out;
           })
      .def("get_voxel_tsdf_at",
           [](const VolumetricMap& m, Eigen::Vector3f pos) -> py::object {
             const TsdfVoxel* vox = m.getTsdfLayer().getVoxelPtr(pos);
             if (!vox || vox->weight <= 0.0f) {
               return py::none();
             }
             py::dict out;
             out["distance"] = vox->distance;
             out["weight"] = vox->weight;
             return out;
           })
      .def_static("get_label_names", []() {
        return GlobalInfo::instance().getLabelToNameMap();
      });

  py::class_<PythonBatchPipeline>(m, "BatchPipeline")
      .def_static(
          "from_config",
          [](const std::string& config, int robot_id) {
            const auto node = YAML::Load(config);
            return std::make_unique<PythonBatchPipeline>(
                config::fromYaml<PythonBatchPipeline::Config>(node), robot_id);
          },
          "config"_a,
          "robot_id"_a = 0)
      .def_static(
          "from_file",
          [](const std::filesystem::path& config, int robot_id) {
            const auto node = YAML::LoadFile(config);
            return std::make_unique<PythonBatchPipeline>(
                config::fromYaml<PythonBatchPipeline::Config>(node), robot_id);
          },
          "config"_a,
          "robot_id"_a = 0)
      .def("construct", &PythonBatchPipeline::construct);
}

}  // namespace python_batch

};  // namespace hydra::python
