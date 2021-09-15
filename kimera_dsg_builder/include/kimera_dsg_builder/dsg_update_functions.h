#pragma once
#include <gtsam/nonlinear/Values.h>
#include <kimera_dsg/dynamic_scene_graph.h>

namespace kimera {

using LayerUpdateFunc = std::function<
    void(const DynamicSceneGraph&, const gtsam::Values&, const gtsam::Values&)>;

namespace dsg_updates {

void updateObjects(const DynamicSceneGraph& graph,
                   const gtsam::Values& places_values,
                   const gtsam::Values& pgmo_values);

void updatePlaces(const DynamicSceneGraph& graph,
                  const gtsam::Values& places_values,
                  const gtsam::Values& pgmo_values);

void updateRooms(const DynamicSceneGraph& graph,
                 const gtsam::Values& places_values,
                 const gtsam::Values& pgmo_values);

void updateBuildings(const DynamicSceneGraph& graph,
                     const gtsam::Values& places_values,
                     const gtsam::Values& pgmo_values);

void updateAgents(const DynamicSceneGraph& graph,
                  const gtsam::Values& places_values,
                  const gtsam::Values& pgmo_values);

}  // namespace dsg_updates

}  // namespace kimera
