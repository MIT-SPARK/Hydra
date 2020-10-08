#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/stl_bind.h>
#include <pybind11/eigen.h>
#include "kimera_scene_graph/scene_graph_node.h"
#include "kimera_scene_graph/scene_graph_edge.h"
#include "kimera_scene_graph/scene_graph_layer.h"
#include "kimera_scene_graph/scene_graph_serialization.h"
#include "kimera_scene_graph/scene_graph_visualizer.h"
#include "kimera_scene_graph/common.h"
#include "kimera_semantics/common.h"
#include <string>
#include <iostream>
#include <sstream>
#include <pcl_ros/point_cloud.h>
#include <pcl/common/centroid.h>



namespace py = pybind11;

/*Convenience Fxn to test module import success*/
int add(int i, int j) {
	return i + j;
}

PYBIND11_MODULE(kimerasg_python, m) {
    m.def("add", &add, py::arg("i"), py::arg("j"));
    m.def("load_scene_graph", &kimera::load<kimera::SceneGraph>);
    py::class_<kimera::NodePosition>(m, "NodePosition")
        .def(py::init<>())
        .def(py::init<const kimera::NodePosition::T&, const kimera::NodePosition::T&, const kimera::NodePosition::T&>())
        .def("__repr__",
            [](const kimera::NodePosition &pos) {
                return "<kimerasg_python.NodePosition> x, y, z: "
                    + std::to_string(pos.x) + ","
                    + std::to_string(pos.y) + ","
                    + std::to_string(pos.z);
            })
				.def_readwrite("x", &kimera::NodePosition::x)
				.def_readwrite("y", &kimera::NodePosition::y)
				.def_readwrite("z", &kimera::NodePosition::z);
    py::class_<kimera::NodeColor>(m, "NodeColor")
        .def(py::init<>())
        .def(py::init<const kimera::NodeColor::T&, const kimera::NodeColor::T&, const kimera::NodeColor::T&>())
        .def("__repr__",
            [](const kimera::NodeColor &color) {
                return "<kimerasg_python.NodeColor> r, g, b: "
                    + std::to_string(color.r) + ","
                    + std::to_string(color.g) + ","
                    + std::to_string(color.b);
            });
    py::class_<kimera::NodeAttributes>(m, "NodeAttributes")
        .def(py::init<>())
        .def("__repr__",
          [](const kimera::NodeAttributes &attrs) {
            std::stringstream out;
            out << "<kimerasg_python.NodeAttributes>: \n"
            << "timestamp_: " << std::to_string(attrs.timestamp_) << ",\n"
            << "position_: " << attrs.position_ << ",\n"
//            << "color_: " << attrs.color_ << ",\n"; //color isn't working because voxblox::Color missing stream operator
            << "semantic_label_: " << std::to_string(attrs.semantic_label_) << ",\n"
            << "name_: " << attrs.name_ << ",\n"
            << "bounding_box_: " << attrs.bounding_box_;
            return out.str();
        })
        .def_readwrite("timestamp_", &kimera::NodeAttributes::timestamp_)
        .def_readwrite("position_", &kimera::NodeAttributes::position_)
        .def_readwrite("color_", &kimera::NodeAttributes::color_)
        .def_readwrite("semantic_label_", &kimera::NodeAttributes::semantic_label_)
        .def_readwrite("name_", &kimera::NodeAttributes::name_)
        //.def_readwrite("pcl_", &kimera::NodeAttributes::pcl_) // No PCL serialization
        .def_readwrite("bounding_box_", &kimera::NodeAttributes::bounding_box_);

    //BoundingBox-related bindings
    //BoundingBox is a template class. In implementation only uses typedef pcl::PointXYZRGB ColorPoint
    py::class_<kimera::ColorPoint>(m, "ColorPoint")
        .def(py::init<>())
        .def_readwrite("x", &kimera::ColorPoint::x)
        .def_readwrite("y", &kimera::ColorPoint::y)
        .def_readwrite("z", &kimera::ColorPoint::z)
        .def_readwrite("r", &kimera::ColorPoint::r)
        .def_readwrite("g", &kimera::ColorPoint::g)
        .def_readwrite("b", &kimera::ColorPoint::b)
        .def("__repr__",
          [](const kimera::ColorPoint &point) {
            return "<kimerasg_python.ColorPoint> (x, y, z): ("
            + std::to_string(point.x) + ","
            + std::to_string(point.y) + ","
            + std::to_string(point.z) + ")";
        });
    py::class_<kimera::BoundingBox<kimera::ColorPoint>>(m, "BoundingBox")
        .def(py::init<>())
        .def_readwrite("type_", &kimera::BoundingBox<kimera::ColorPoint>::type_)
        .def_readwrite("max_", &kimera::BoundingBox<kimera::ColorPoint>::max_)
        .def_readwrite("min_", &kimera::BoundingBox<kimera::ColorPoint>::min_)
        .def_readwrite("position_", &kimera::BoundingBox<kimera::ColorPoint>::position_)
        .def("__repr__",
          [](const kimera::BoundingBox<kimera::ColorPoint>  &bounding_box) {
            std::stringstream out;
            out << "<kimerasg_python.BoundingBox> max_, min_, position_: " << ",\n"
              << bounding_box.max_ << ",\n"
              << bounding_box.min_ << ",\n"
              << bounding_box.position_ << ",\n";
            return out.str();
        });
    py::enum_<kimera::BoundingBoxType>(m, "BoundingBoxType")
        .value("kAABB", kimera::BoundingBoxType::kAABB)
        .value("kOBB", kimera::BoundingBoxType::kOBB);

    //SceneGraphNode binding
    py::class_<kimera::SceneGraphNode>(m, "SceneGraphNode")
        .def(py::init<>())
        .def("hasParent", &kimera::SceneGraphNode::hasParent)
        .def("hasSiblings", &kimera::SceneGraphNode::hasSiblings)
        .def("hasSiblingEdge", &kimera::SceneGraphNode::hasSiblingEdge)
        .def("checkSiblingEdgeMap", &kimera::SceneGraphNode::checkSiblingEdgeMap)
        .def_readwrite("attributes_", &kimera::SceneGraphNode::attributes_)
        .def_readwrite("node_id_", &kimera::SceneGraphNode::node_id_)
        .def_readwrite("layer_id_", &kimera::SceneGraphNode::layer_id_)
        .def_readwrite("siblings_edge_map_", &kimera::SceneGraphNode::siblings_edge_map_)
        .def_readwrite("parent_edge_", &kimera::SceneGraphNode::parent_edge_)
        .def_readwrite("children_edge_map_", &kimera::SceneGraphNode::children_edge_map_);

    //SceneGraphEdge binding
    py::class_<kimera::SceneGraphEdge>(m, "SceneGraphEdge")
        .def(py::init<>())
        .def_readwrite("edge_id_", &kimera::SceneGraphEdge::edge_id_)
        .def_readwrite("start_layer_id_", &kimera::SceneGraphEdge::start_layer_id_)
        .def_readwrite("start_node_id_", &kimera::SceneGraphEdge::start_node_id_)
        .def_readwrite("end_layer_id_", &kimera::SceneGraphEdge::end_layer_id_)
        .def_readwrite("end_node_id_", &kimera::SceneGraphEdge::end_node_id_)
        .def("isInterLayerEdge", &kimera::SceneGraphEdge::isInterLayerEdge)
        .def("isEdgeValid", &kimera::SceneGraphEdge::isEdgeValid)
        .def("isSelfEdge", &kimera::SceneGraphEdge::isSelfEdge)
        .def("swapDirection", &kimera::SceneGraphEdge::swapDirection)
        .def("__eq__",
            [](const kimera::SceneGraphEdge &self, const kimera::SceneGraphEdge &other) {
                return self.edge_id_ == other.edge_id_ &&
                    self.start_layer_id_ == other.start_layer_id_ &&
                    self.start_node_id_ == other.start_node_id_ &&
                    self.end_layer_id_ == other.end_layer_id_ &&
                    self.end_node_id_ == other.end_node_id_;
            }, py::is_operator())
        .def("__repr__",
            [](const kimera::SceneGraphEdge &edge) {
                return "<kimerasg_python.SceneGraphEdge>: \n EdgeId: "
                    + std::to_string(edge.edge_id_)
                    + ", Start Layer Id: " + std::to_string(static_cast<typename std::underlying_type<kimera::LayerId>::type>(edge.start_layer_id_)) //bind to_underlying typename
                    + ", Start Node Id: " + std::to_string(edge.start_node_id_)
                    + ", End Layer Id: " + std::to_string(static_cast<typename std::underlying_type<kimera::LayerId>::type>(edge.end_layer_id_))
                    + ", End Node Id: " + std::to_string(edge.end_node_id_);
            });

    m.def("getEdgeIdsInEdgeIdMap", &kimera::getEdgeIdsInEdgeIdMap);

    //LayerId strongly typed enum binding
    py::enum_<kimera::LayerId>(m, "LayerId")
        .value("kInvalidLayerId", kimera::LayerId::kInvalidLayerId)
        .value("kObjectsLayerId", kimera::LayerId::kObjectsLayerId)
        .value("kAgentsLayerId", kimera::LayerId::kAgentsLayerId)
        .value("kPlacesLayerId", kimera::LayerId::kPlacesLayerId)
        .value("kRoomsLayerId", kimera::LayerId::kRoomsLayerId)
        .value("kBuildingsLayerId", kimera::LayerId::kBuildingsLayerId);
    m.def("getStringFromLayerId", &kimera::getStringFromLayerId);

    //SceneGraphLayer binding
    py::class_<kimera::SceneGraphLayer>(m, "SceneGraphLayer")
      .def(py::init<>())
      .def(py::init<const kimera::LayerId&>())
      //attributes to remain private, ensure getters have return_value_policy::reference IFF the no bindings for returned pointer's type
//        .def_readwrite("layer_id_", &kimera::SceneGraphLayer::layer_id_)
//        .def_readwrite("node_map_", &kimera::SceneGraphLayer::node_map_)
//        .def_readwrite("next_intra_layer_edge_id_", &kimera::SceneGraphLayer::next_intra_layer_edge_id_)
//        .def_readwrite("intra_layer_edge_map_", &kimera::SceneGraphLayer::intra_layer_edge_map_)
//        //above attributes should remain private
      .def("getLayerId", &kimera::SceneGraphLayer::getLayerId)
      .def("getNodeIdMap", &kimera::SceneGraphLayer::getNodeIdMap)
      .def("getEdgeIdMap", &kimera::SceneGraphLayer::getEdgeIdMap)
      .def("getNodeIdMapMutable", &kimera::SceneGraphLayer::getNodeIdMapMutable)
      .def("getEdgeIdMapMutable", &kimera::SceneGraphLayer::getEdgeIdMapMutable)
      .def("getNode", &kimera::SceneGraphLayer::getNode)
      .def("getIntraLayerEdge", &kimera::SceneGraphLayer::getIntraLayerEdge)
      .def("getNodeMutable", &kimera::SceneGraphLayer::getNodeMutable)
      .def("getIntraLayerEdgeMutable", &kimera::SceneGraphLayer::getIntraLayerEdgeMutable)
      .def("getNumberOfNodes", &kimera::SceneGraphLayer::getNumberOfNodes)
      .def("getNumberOfEdges", &kimera::SceneGraphLayer::getNumberOfEdges)
      .def("setLayerId", &kimera::SceneGraphLayer::setLayerId)
      .def("hasNode", &kimera::SceneGraphLayer::hasNode)
      .def("hasEdge", &kimera::SceneGraphLayer::hasEdge);
      //.def("convertLayerToPcl", &kimera::SceneGraphLayer::convertLayerToPcl); //ColorPointCloud::Ptr
      //protected
//        .def("addNode", &kimera::SceneGraphLayer::addNode)
//        .def("addIntraLayerEdge", &kimera::SceneGraphLayer::addIntraLayerEdge);

		//SceneGraph binding
    py::class_<kimera::SceneGraph>(m, "SceneGraph")
        .def(py::init<>())
        // protected
//        .def_readwrite("database_", &kimera::SceneGraph::database_)
//        .def_readwrite("next_inter_layer_edge_id_", &kimera::SceneGraph::next_inter_layer_edge_id_)
//        .def_readwrite("inter_layer_edge_map_", &kimera::SceneGraph::inter_layer_edge_map_)
        .def("getNode", &kimera::SceneGraph::getNode)
        .def("getAllSceneNodes", &kimera::SceneGraph::getAllSceneNodes)
        .def("getInterLayerEdgesMap", &kimera::SceneGraph::getInterLayerEdgesMap)
        .def("getDatabase", &kimera::SceneGraph::getDatabase)
        .def("getInterLayerEdge", &kimera::SceneGraph::getInterLayerEdge)
        .def("getInterLayerEdgeMutable", &kimera::SceneGraph::getInterLayerEdgeMutable)
        .def("getLayer", &kimera::SceneGraph::getLayer)
        .def("getLayerSafe", &kimera::SceneGraph::getLayerSafe)
        .def("getLayerMutable", &kimera::SceneGraph::getLayerMutable)
        .def("getNumberOfUniqueSceneNodes", &kimera::SceneGraph::getNumberOfUniqueSceneNodes)
        .def("addSceneNode", &kimera::SceneGraph::addSceneNode)
        .def("addEdge", &kimera::SceneGraph::addEdge)
        .def("hasLayer", &kimera::SceneGraph::hasLayer)
        .def("hasNode", &kimera::SceneGraph::hasNode)
        .def("hasInterLayerEdge", &kimera::SceneGraph::hasInterLayerEdge)
        .def("clear", &kimera::SceneGraph::clear);
        // protected
//        .def("addInterLayerEdge", &kimera::SceneGraph::addInterLayerEdge)
//        .def("isParent", &kimera::SceneGraph::isParent)
//        .def("isChild", &kimera::SceneGraph::isChild)
//        .def("isSibling", &kimera::SceneGraph::isSibling);
}
