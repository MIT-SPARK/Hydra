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
#pragma once
#include "hydra_topology/graph_extractor.h"
#include "hydra_topology/gvd_voxel.h"
#include "hydra_topology/voxblox_types.h"

#include <hydra_topology/GvdVisualizerConfig.h>
#include <hydra_utils/visualizer_types.h>
#include <visualization_msgs/Marker.h>

namespace hydra {
namespace topology {

using hydra_topology::GvdVisualizerConfig;

enum class GvdVisualizationMode : int {
  DEFAULT = hydra_topology::GvdVisualizer_DEFAULT,
  DISTANCE = hydra_topology::GvdVisualizer_DISTANCE,
  BASIS_POINTS = hydra_topology::GvdVisualizer_BASIS_POINTS,
};

enum class VisualizationType : int {
  NONE = hydra_topology::GvdVisualizer_NONE,
  ESDF_WITH_SLICE = hydra_topology::GvdVisualizer_ESDF_SLICE,
  GVD = hydra_topology::GvdVisualizer_GVD,
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

visualization_msgs::Marker makeBlocksMarker(const Layer<TsdfVoxel>& layer,
                                            double scale);

visualization_msgs::Marker makeBlocksMarker(const Layer<GvdVoxel>& layer, double scale);

}  // namespace topology
}  // namespace hydra
