#pragma once
#include "kimera_topology/graph_extractor.h"
#include "kimera_topology/gvd_voxel.h"
#include "kimera_topology/voxblox_types.h"

#include <kimera_dsg_visualizer/visualizer_types.h>
#include <kimera_topology/GvdVisualizerConfig.h>
#include <visualization_msgs/Marker.h>

namespace kimera {
namespace topology {

using kimera_topology::GvdVisualizerConfig;

enum class GvdVisualizationMode : int {
  DEFAULT = kimera_topology::GvdVisualizer_DEFAULT,
  DISTANCE = kimera_topology::GvdVisualizer_DISTANCE,
  BASIS_POINTS = kimera_topology::GvdVisualizer_BASIS_POINTS,
};

enum class VisualizationType : int {
  NONE = kimera_topology::GvdVisualizer_NONE,
  ESDF_WITH_SLICE = kimera_topology::GvdVisualizer_ESDF_SLICE,
  GVD = kimera_topology::GvdVisualizer_GVD,
};

GvdVisualizationMode getModeFromString(const std::string& mode);

visualization_msgs::Marker makeGvdMarker(const GvdVisualizerConfig& config,
                                         const ColormapConfig& colors,
                                         const Layer<GvdVoxel>& layer);

visualization_msgs::Marker makeEsdfMarker(const GvdVisualizerConfig& config,
                                          const ColormapConfig& colors,
                                          const Layer<GvdVoxel>& layer);

visualization_msgs::Marker makeGvdEdgeMarker(
    const Layer<GvdVoxel>& layer,
    const GraphExtractor::EdgeInfoMap& edge_info_map,
    const GraphExtractor::NodeIdRootMap& id_root_index_map);

visualization_msgs::Marker makeBlocksMarker(const Layer<TsdfVoxel>& layer, double scale);

visualization_msgs::Marker makeBlocksMarker(const Layer<GvdVoxel>& layer, double scale);

}  // namespace topology
}  // namespace kimera
