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
#include <hydra/places/gvd_places/graph_extractor.h>
#include <hydra/places/gvd_places/gvd_graph.h>
#include <hydra/places/gvd_places/gvd_voxel.h>
#include <hydra_visualizer/color/colormap_utilities.h>

#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace hydra {

using ClusterRemapping = std::unordered_map<uint64_t, uint64_t>;
using MarkerMsg = visualization_msgs::msg::Marker;
using MarkerArrayMsg = visualization_msgs::msg::MarkerArray;

enum class GvdVisualizationMode : int {
  DEFAULT,
  DISTANCE,
  BASIS_POINTS,
};

struct GvdVisualizerConfig {
  //! @brief scale for block outlines
  double block_outline_scale = 0.02;
  //! @brief alpha of the GVD
  double gvd_alpha = 0.6;
  //! @brief distance colormap min
  double gvd_min_distance = 0.2;
  //! @brief distance colormap max
  double gvd_max_distance = 3.0;
  //! @brief basis threshold for GVD inclusion
  int basis_threshold = 2;
  //! @brief basis colormap min
  int min_num_basis = 1;
  //! @brief basis colormap max
  int max_num_basis = 10;
  //! @brief visualization mode
  GvdVisualizationMode gvd_mode = GvdVisualizationMode::DEFAULT;
  //! @brief scale for wireframe
  double gvd_graph_scale = 0.005;
  //! @brief alpha of the ESDF
  double esdf_alpha = 0.6;
  //! @brief height of ESDf slice
  double slice_height = 0.0;
  //! @brief distance colormap scale
  double esdf_distance = 2.0;
};

void declare_config(GvdVisualizerConfig& config);

MarkerMsg drawEsdf(const GvdVisualizerConfig& config,
                   const visualizer::RangeColormap& colormap,
                   const Eigen::Isometry3d& pose,
                   const places::GvdLayer& layer,
                   const std::string& ns);

MarkerMsg drawGvd(const GvdVisualizerConfig& config,
                  const visualizer::RangeColormap& colormap,
                  const places::GvdLayer& layer,
                  const std::string& ns);

MarkerMsg drawGvdSurface(const GvdVisualizerConfig& config,
                         const visualizer::RangeColormap& colormap,
                         const places::GvdLayer& layer,
                         const std::string& ns);

MarkerArrayMsg drawGvdWireframe(const places::GvdGraph& graph,
                                const GvdVisualizerConfig& config,
                                const visualizer::RangeColormap& cmap,
                                const std::string& ns,
                                size_t marker_id = 0);

MarkerArrayMsg drawGvdActive(const places::GvdGraph& graph,
                             const GvdVisualizerConfig& config,
                             const std::string& ns,
                             size_t marker_id = 0);

MarkerArrayMsg drawGvdActiveCluster(const places::GvdGraph& graph,
                                    const GvdVisualizerConfig& config,
                                    const std::string& ns,
                                    size_t marker_id = 0);

MarkerArrayMsg drawGvdClusters(const places::GvdGraph& graph,
                               const GvdVisualizerConfig& config,
                               const std::string& ns,
                               const visualizer::DiscreteColormap& cmap = {},
                               size_t marker_id = 0);

MarkerArrayMsg drawCompressedGraph(const places::GraphExtractor::LocalGraph& graph,
                                   const GvdVisualizerConfig& config,
                                   const visualizer::RangeColormap& cmap,
                                   const std::string& ns,
                                   double node_scale = 0.2,
                                   double edge_scale = 0.01,
                                   double alpha = 1.0,
                                   size_t marker_id = 0);

}  // namespace hydra
