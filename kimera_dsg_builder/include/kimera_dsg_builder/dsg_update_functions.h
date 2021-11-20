#pragma once
#include <gtsam/nonlinear/Values.h>
#include <kimera_dsg/dynamic_scene_graph.h>

namespace kimera {

using LayerUpdateFunc = std::function<
    void(DynamicSceneGraph&, const gtsam::Values&, const gtsam::Values&, bool)>;

namespace dsg_updates {

void updateObjects(DynamicSceneGraph& graph,
                   const gtsam::Values& places_values,
                   const gtsam::Values& pgmo_values,
                   bool allow_node_merging);

void updatePlaces(DynamicSceneGraph& graph,
                  const gtsam::Values& places_values,
                  const gtsam::Values& pgmo_values,
                  bool allow_node_merging,
                  double pos_threshold_m,
                  double distance_tolerance_m);

void updateRooms(DynamicSceneGraph& graph,
                 const gtsam::Values& places_values,
                 const gtsam::Values& pgmo_values,
                 bool allow_node_merging);

void updateBuildings(DynamicSceneGraph& graph,
                     const gtsam::Values& places_values,
                     const gtsam::Values& pgmo_values,
                     bool allow_node_merging);

void updateAgents(DynamicSceneGraph& graph,
                  const gtsam::Values& places_values,
                  const gtsam::Values& pgmo_values,
                  bool allow_node_merging);

}  // namespace dsg_updates

}  // namespace kimera
