#include "hydra/bindings/python_evaluation.h"

#include <pybind11/eigen.h>
#include <pybind11/stl.h>

#include <cstdint>
#include <map>

#include "hydra/eval/place_evaluator.h"
#include "hydra/eval/room_evaluator.h"

namespace hydra::python::python_evaluation {
namespace py = pybind11;
using namespace py::literals;

void addBindings(py::module_& parent) {
  auto m = parent.def_submodule("eval", "Room and place evaluation primitives.");
  using namespace hydra::eval;
  using spark_dsg::SceneGraph;

  py::class_<PlaceMetrics>(m, "PlaceMetrics")
      .def_readonly("is_valid", &PlaceMetrics::is_valid)
      .def_readonly("num_missing", &PlaceMetrics::num_missing)
      .def_readonly("num_unobserved", &PlaceMetrics::num_unobserved)
      .def_readonly("num_valid", &PlaceMetrics::num_valid)
      .def_readonly("gvd_distance_errors", &PlaceMetrics::gvd_distance_errors)
      .def_readonly("node_gvd_distances", &PlaceMetrics::node_gvd_distances)
      .def_readonly("node_order", &PlaceMetrics::node_order);

  py::class_<RoomMetrics>(m, "RoomMetrics")
      .def_property_readonly("is_valid", &RoomMetrics::valid)
      .def_readonly("total_recall", &RoomMetrics::total_recall)
      .def_readonly("total_precision", &RoomMetrics::total_precision)
      .def_readonly("recalls", &RoomMetrics::recalls)
      .def_readonly("precisions", &RoomMetrics::precisions)
      .def_readonly("gt_sizes", &RoomMetrics::gt_sizes)
      .def_readonly("est_sizes", &RoomMetrics::est_sizes)
      .def_readonly("overlaps", &RoomMetrics::overlaps);

  py::class_<RoomEvaluator::Config>(m, "RoomEvaluatorConfig")
      .def(py::init<>())
      .def_readwrite("only_labeled", &RoomEvaluator::Config::only_labeled)
      .def_readwrite("min_weight", &RoomEvaluator::Config::min_weight)
      .def_readwrite("min_distance", &RoomEvaluator::Config::min_distance)
      .def_readwrite("min_room_nodes", &RoomEvaluator::Config::min_room_nodes);

  py::class_<RoomGeometry>(m, "RoomGeometry")
      .def(py::init<>())
      .def_static("from_file", &RoomGeometry::fromFile, "path"_a)
      .def_static("from_yaml", &RoomGeometry::fromYaml, "contents"_a)
      .def("add_room", &RoomGeometry::addRoom, "room_id"_a, "boxes"_a)
      .def("find_room_index", &RoomGeometry::findRoomIndex, "position"_a)
      .def("get_room_ids", &RoomGeometry::getRoomIds);

  py::class_<PlaceEvaluator>(m, "PlaceEvaluator")
      .def_static("from_file",
                  &PlaceEvaluator::fromFile,
                  "config_path"_a,
                  "tsdf_path"_a,
                  "max_distance_m"_a = py::none(),
                  py::call_guard<py::gil_scoped_release>())
      .def("eval",
           py::overload_cast<const SceneGraph&, size_t, const std::string&>(
               &PlaceEvaluator::eval, py::const_),
           "graph"_a,
           "min_basis"_a = 1,
           "layer_id"_a = spark_dsg::DsgLayers::PLACES,
           py::call_guard<py::gil_scoped_release>())
      .def("eval_file",
           py::overload_cast<const std::string&, size_t, const std::string&>(
               &PlaceEvaluator::eval, py::const_),
           "path"_a,
           "min_basis"_a = 1,
           "layer_id"_a = spark_dsg::DsgLayers::PLACES,
           py::call_guard<py::gil_scoped_release>());

  py::class_<RoomEvaluator>(m, "RoomEvaluator")
      .def_static("from_file",
                  &RoomEvaluator::fromFile,
                  "config"_a,
                  "room_path"_a,
                  "tsdf_path"_a,
                  py::call_guard<py::gil_scoped_release>())
      .def("eval",
           py::overload_cast<const SceneGraph&>(&RoomEvaluator::eval, py::const_),
           "graph"_a,
           py::call_guard<py::gil_scoped_release>())
      .def("eval_file",
           py::overload_cast<const std::string&>(&RoomEvaluator::eval, py::const_),
           "path"_a,
           py::call_guard<py::gil_scoped_release>())
      .def("get_room_indices", [](const RoomEvaluator& evaluator) {
        using Indices = Eigen::Matrix<int64_t, Eigen::Dynamic, 3>;
        std::map<size_t, Indices> result;
        for (const auto& [room, indices] : evaluator.getRoomIndices()) {
          auto& voxels = result[room];
          voxels.resize(static_cast<Eigen::Index>(indices.size()), 3);
          Eigen::Index row = 0;
          for (const auto& index : indices) {
            voxels.row(row) << index[0], index[1], index[2];
            ++row;
          }
        }
        return result;
      });

  m.def("score_rooms",
        &scoreRooms,
        "ground_truth"_a,
        "estimated"_a,
        py::call_guard<py::gil_scoped_release>());
}
}  // namespace hydra::python::python_evaluation
