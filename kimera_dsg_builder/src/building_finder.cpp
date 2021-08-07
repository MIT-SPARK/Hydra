#include "kimera_dsg_builder/building_finder.h"
#include "kimera_dsg_builder/common.h"
#include "kimera_dsg_builder/pcl_types.h"

#include <kimera_dsg/node_attributes.h>

namespace kimera {

//  TODO(Toni): assumes we only have one building, ideally get the sub-graph
// spanned by the inter-class room traversability edges.
void findBuildings(SceneGraph* scene_graph, const NodeColor& color) {
  CHECK_NOTNULL(scene_graph);

  // TODO(nathan) consider unique class for buildings
  RoomNodeAttributes::Ptr building_attrs =
      std::make_unique<RoomNodeAttributes>();
  building_attrs->semantic_label = kBuildingSemanticLabel;
  building_attrs->color = color;

  NodeSymbol building_id('B', 1u);
  building_attrs->name = std::to_string(building_id);

  CHECK(scene_graph->hasLayer(to_underlying(KimeraDsgLayers::ROOMS)));
  const SceneGraphLayer& room_layer =
      *scene_graph->getLayer(to_underlying(KimeraDsgLayers::ROOMS));

  // Calculate the building's centroid: centroid of rooms' centroids.
  Centroid building_centroid;
  for (const auto& id_node_pair : room_layer.nodes()) {
    Eigen::Vector3d room_pos =
        id_node_pair.second->attributes<RoomNodeAttributes>().position;
    building_centroid.add(pcl::PointXYZ(room_pos(0), room_pos(1), room_pos(2)));
  }

  pcl::PointXYZ centroid;
  building_centroid.get(centroid);
  building_attrs->position << centroid.x, centroid.y, centroid.z;

  scene_graph->emplaceNode(to_underlying(KimeraDsgLayers::BUILDINGS),
                           building_id,
                           std::move(building_attrs));
  for (const auto& id_node_pair : room_layer.nodes()) {
    scene_graph->insertEdge(building_id, id_node_pair.first);
  }
}

}  // namespace kimera
