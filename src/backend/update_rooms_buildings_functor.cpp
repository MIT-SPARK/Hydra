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
#include "hydra/backend/update_rooms_buildings_functor.h"

#include <glog/logging.h>

#include "hydra/utils/timing_utilities.h"

namespace hydra {

using timing::ScopedTimer;
using SemanticLabel = SemanticNodeAttributes::Label;

UpdateRoomsFunctor::UpdateRoomsFunctor(const RoomFinderConfig& config)
    : room_finder(new RoomFinder(config)) {}

void UpdateRoomsFunctor::rewriteRooms(const SceneGraphLayer* new_rooms,
                                      DynamicSceneGraph& graph) const {
  std::vector<NodeId> to_remove;
  const auto& prev_rooms = graph.getLayer(DsgLayers::ROOMS);
  for (const auto& id_node_pair : prev_rooms.nodes()) {
    to_remove.push_back(id_node_pair.first);
  }

  for (const auto node_id : to_remove) {
    graph.removeNode(node_id);
  }

  if (!new_rooms) {
    return;
  }

  for (auto&& [id, node] : new_rooms->nodes()) {
    graph.emplaceNode(DsgLayers::ROOMS, id, node->attributes().clone());
  }

  for (const auto& id_edge_pair : new_rooms->edges()) {
    const auto& edge = id_edge_pair.second;
    graph.insertEdge(edge.source, edge.target, edge.info->clone());
  }
}

MergeList UpdateRoomsFunctor::call(const DynamicSceneGraph&,
                                   SharedDsgInfo& dsg,
                                   const UpdateInfo::ConstPtr& info) const {
  if (!room_finder) {
    return {};
  }

  ScopedTimer timer("backend/room_detection", info->timestamp_ns, true, 1, false);
  auto places_clone =
      dsg.graph->getLayer(DsgLayers::PLACES).clone([](const auto& node) {
        return NodeSymbol(node.id).category() == 'p';
      });

  // TODO(nathan) pass in timestamp?
  auto rooms = room_finder->findRooms(*places_clone);
  rewriteRooms(rooms.get(), *dsg.graph);
  room_finder->addRoomPlaceEdges(*dsg.graph);
  return {};
}

UpdateBuildingsFunctor::UpdateBuildingsFunctor(const Color& color, SemanticLabel label)
    : building_color(color), building_semantic_label(label) {}

MergeList UpdateBuildingsFunctor::call(const DynamicSceneGraph&,
                                       SharedDsgInfo& dsg,
                                       const UpdateInfo::ConstPtr& info) const {
  ScopedTimer timer("backend/building_detection", info->timestamp_ns, true, 1, false);

  const NodeSymbol building_id('B', 0);
  const auto& rooms = dsg.graph->getLayer(DsgLayers::ROOMS);

  if (!rooms.numNodes()) {
    if (dsg.graph->hasNode(building_id)) {
      dsg.graph->removeNode(building_id);
    }

    return {};
  }

  Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
  for (const auto& id_node_pair : rooms.nodes()) {
    centroid += id_node_pair.second->attributes().position;
  }
  centroid /= rooms.numNodes();

  if (!dsg.graph->hasNode(building_id)) {
    SemanticNodeAttributes::Ptr attrs(new SemanticNodeAttributes());
    attrs->position = centroid;
    attrs->color = building_color;
    attrs->semantic_label = building_semantic_label;
    attrs->name = building_id.getLabel();
    dsg.graph->emplaceNode(DsgLayers::BUILDINGS, building_id, std::move(attrs));
  } else {
    dsg.graph->getNode(building_id).attributes().position = centroid;
  }

  for (const auto& id_node_pair : rooms.nodes()) {
    dsg.graph->insertParentEdge(building_id, id_node_pair.first);
  }

  return {};
}

}  // namespace hydra
