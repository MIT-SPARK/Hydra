#pragma once
#include <hydra_utils/ColormapConfig.h>
#include <hydra_utils/DynamicLayerVisualizerConfig.h>
#include <hydra_utils/LayerVisualizerConfig.h>
#include <hydra_utils/VisualizerConfig.h>

#include <kimera_dsg/dynamic_scene_graph.h>
#include <kimera_dsg/node_attributes.h>

namespace hydra {

using LayerConfig = hydra_utils::LayerVisualizerConfig;
using DynamicLayerConfig = hydra_utils::DynamicLayerVisualizerConfig;
using VisualizerConfig = hydra_utils::VisualizerConfig;
using ColormapConfig = hydra_utils::ColormapConfig;
using NodeColor = kimera::SemanticNodeAttributes::ColorVector;

using kimera::BoundingBox;
using kimera::DynamicSceneGraph;
using kimera::DynamicSceneGraphLayer;
using kimera::KimeraDsgLayers;
using kimera::LayerId;
using kimera::NodeId;
using kimera::NodeSymbol;
using kimera::ObjectNodeAttributes;
using kimera::PlaceNodeAttributes;
using kimera::SceneGraphEdge;
using kimera::SceneGraphLayer;
using kimera::SceneGraphNode;
using kimera::SemanticNodeAttributes;

inline double getZOffset(double z_offset_scale,
                         const VisualizerConfig& visualizer_config) {
  if (visualizer_config.collapse_layers) {
    return 0.0;
  }

  return z_offset_scale * visualizer_config.layer_z_step;
}

inline double getZOffset(const LayerConfig& config,
                         const VisualizerConfig& visualizer_config) {
  return getZOffset(config.z_offset_scale, visualizer_config);
}

}  // namespace hydra
