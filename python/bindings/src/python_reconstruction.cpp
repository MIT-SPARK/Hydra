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
#include "hydra/bindings/python_reconstruction.h"

#include <config_utilities/config.h>
#include <config_utilities/formatting/asl.h>
#include <config_utilities/logging/log_to_glog.h>
#include <config_utilities/parsing/yaml.h>
#include <config_utilities/validation.h>
#include <hydra/common/global_info.h>
#include <hydra/reconstruction/reconstruction_module.h>
#include <kimera_pgmo/compression/delta_compression.h>
#include <kimera_pgmo/utils/mesh_io.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pybind11/stl/filesystem.h>
#include <pybind11/stl_bind.h>
#include <spark_dsg/dynamic_scene_graph.h>
#include <spark_dsg/zmq_interface.h>

#include "hydra/bindings/glog_utilities.h"
#include "hydra/bindings/python_config.h"
#include "hydra/bindings/python_image.h"
#include "hydra/bindings/python_sensor_input.h"
#include "hydra/utils/mesh_utilities.h"
#include "hydra/utils/pgmo_mesh_interface.h"
#include "hydra/utils/pgmo_mesh_traits.h"

namespace hydra::python {

struct PythonReconstructionConfig {
  int verbosity = 1;
  bool visualize_mesh = true;
  std::string zmq_url = "tcp://127.0.0.1:8001";
};

void declare_config(PythonReconstructionConfig& conf) {
  using namespace config;
  name("PythonReconstructionConfig");
  field(conf.verbosity, "verbosity");
  field(conf.visualize_mesh, "visualize_mesh");
  field(conf.zmq_url, "zmq_url");
}

struct MeshUpdater {
  MeshUpdater(double voxel_size, const std::string& url)
      : compression(voxel_size / 4.0),
        queue(new ReconstructionModule::OutputQueue()),
        zmq_sender(url, 2) {
    graph.reset(new DynamicSceneGraph(DynamicSceneGraph::LayerIds{2, 3, 4, 5}));
    graph->setMesh(std::make_shared<Mesh>());
  }

  void spin() {
    while (!should_shutdown) {
      bool has_data = queue->poll();
      if (!has_data) {
        continue;
      }

      update(queue->front());
      queue->pop();
    }
  }

  void update(const ReconstructionOutput::Ptr& msg) {
    if (!msg) {
      return;
    }

    auto mesh = getActiveMesh(msg->map().getMeshLayer(), msg->archived_blocks);
    if (!mesh) {
      return;
    }

    auto interface = PgmoMeshLayerInterface(*mesh);
    const auto delta = compression.update(interface, msg->timestamp_ns);
    delta->updateMesh(*graph->mesh());
    zmq_sender.send(*graph, true);
  }

 public:
  std::atomic<bool> should_shutdown = false;
  kimera_pgmo::DeltaCompression compression;
  std::shared_ptr<spark_dsg::DynamicSceneGraph> graph;
  std::vector<uint64_t> mesh_timestamps;
  ReconstructionModule::OutputQueue::Ptr queue;
  ZmqSender zmq_sender;
};

PythonReconstruction::PythonReconstruction(const PipelineConfig& hydra_config,
                                           const PythonConfig& config) {
  GlogSingleton::instance().setLogLevel(0, 0, false);
  config::Settings().setLogger("glog");
  config::Settings().print_width = 100;
  config::Settings().print_indent = 45;

  const auto node = config.toYaml();
  const auto& map_config = GlobalInfo::init(hydra_config).getMapConfig();
  const auto py_config = config::fromYaml<PythonReconstructionConfig>(node);

  ReconstructionModule::OutputQueue::Ptr queue;
  if (py_config.visualize_mesh) {
    mesh_updater_.reset(new MeshUpdater(map_config.voxel_size, py_config.zmq_url));
    mesh_thread_.reset(new std::thread(&MeshUpdater::spin, mesh_updater_.get()));
    queue = mesh_updater_->queue;
  }

  module_ = config::createFromYaml<ReconstructionModule>(node, queue);
  if (!module_) {
    throw std::runtime_error("could not recreate reconstruction module");
  }

  VLOG(py_config.verbosity) << std::endl << GlobalInfo::instance();
  VLOG(py_config.verbosity) << std::endl << module_->printInfo();
}

void PythonReconstruction::stop() {
  if (mesh_thread_) {
    mesh_updater_->should_shutdown = true;
    mesh_thread_->join();
    mesh_thread_.reset();
  }
}

PythonReconstruction::~PythonReconstruction() { stop(); }

bool PythonReconstruction::step(const InputPacket& input) {
  return module_->spinOnce(input);
}

void PythonReconstruction::save() {
  const auto map = module_->getMap();
  if (!map) {
    return;
  }

  const auto& logs = GlobalInfo::instance().getLogs();
  if (logs && logs->valid()) {
    map->save(logs->getLogDir() + "/map");
  }

  stop();

  if (!mesh_updater_) {
    return;
  }

  auto mesh = mesh_updater_->graph->mesh();
  if (!mesh || mesh->empty()) {
    return;
  }

  kimera_pgmo::WriteMesh(logs->getLogDir() + "/mesh.ply", *mesh);
}

namespace python_reconstruction {
using namespace pybind11::literals;
namespace py = pybind11;

void addBindings(pybind11::module_& m) {
  py::class_<PythonReconstruction>(m, "HydraReconstruction")
      .def(py::init<const PipelineConfig&, const PythonConfig&>(),
           "hydra_config"_a,
           "config"_a)
      .def("save", &PythonReconstruction::save)
      .def("stop", &PythonReconstruction::stop)
      .def("step",
           [](PythonReconstruction& pipeline,
              size_t timestamp_ns,
              const Eigen::Vector3d& world_t_body,
              const Eigen::Vector4d& world_R_body,
              const py::buffer& depth,
              const py::buffer& labels,
              const py::buffer& rgb) {
             auto input = std::make_shared<InputPacket>();
             input->timestamp_ns = timestamp_ns;
             input->world_t_body = world_t_body;
             input->world_R_body = Eigen::Quaterniond(
                 world_R_body[0], world_R_body[1], world_R_body[2], world_R_body[3]);
             input->sensor_input =
                 std::make_unique<PythonSensorInput>(timestamp_ns, depth, labels, rgb);
             return pipeline.step(*input);
           })
      .def("step",
           [](PythonReconstruction& pipeline,
              size_t timestamp_ns,
              const Eigen::Vector3d& world_t_body,
              const Eigen::Vector4d& world_R_body,
              const PythonSensorInput::PointVec& points,
              const PythonSensorInput::LabelVec& labels,
              const PythonSensorInput::ColorVec& colors) {
             auto input = std::make_shared<InputPacket>();
             input->timestamp_ns = timestamp_ns;
             input->world_t_body = world_t_body;
             input->world_R_body = Eigen::Quaterniond(
                 world_R_body[0], world_R_body[1], world_R_body[2], world_R_body[3]);
             input->sensor_input = std::make_unique<PythonSensorInput>(
                 timestamp_ns, points, labels, colors);
             return pipeline.step(*input);
           });
}

}  // namespace python_reconstruction

};  // namespace hydra::python
