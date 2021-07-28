#pragma once
#include "kimera_topology/graph_extractor.h"
#include "kimera_topology/gvd_voxel.h"
#include "kimera_topology/voxblox_types.h"

#include <kimera_dsg_visualizer/colormap_utils.h>
#include <visualization_msgs/Marker.h>

namespace kimera {
namespace topology {

enum class GvdVisualizationMode : int {
  DEFAULT = 0,
  DISTANCE = 1,
  BASIS_POINTS = 2,
};

struct GvdVisualizationConfig {
  GvdVisualizationMode mode = GvdVisualizationMode::DEFAULT;
  double min_distance = 0.2;
  double max_distance = 5.0;
  int basis_threshold = 1;
  int min_num_basis = 1;
  int max_num_basis = 10;
  double alpha = 0.8;
};

struct EsdfVisualizationConfig {
  double slice_height = 0.5;
  double min_distance = 0.2;
  double max_distance = 5.0;
  double alpha = 0.8;
};

GvdVisualizationMode getModeFromString(const std::string& mode);

visualization_msgs::Marker makeGvdMarker(
    const Layer<GvdVoxel>& layer,
    const dsg_utils::HlsColorMapConfig& color_config,
    const GvdVisualizationConfig& gvd_config);

visualization_msgs::Marker makeEsdfMarker(
    const Layer<GvdVoxel>& layer,
    const dsg_utils::HlsColorMapConfig& color_config,
    const EsdfVisualizationConfig& esdf_config);

visualization_msgs::Marker makeGvdEdgeMarker(const Layer<GvdVoxel>& layer,
                                             const EdgeInfoMap& edge_info_map,
                                             const IdIndexMap& id_root_index_map);

}  // namespace topology
}  // namespace kimera
