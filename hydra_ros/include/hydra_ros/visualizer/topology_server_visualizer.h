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
#include <dynamic_reconfigure/server.h>
#include <hydra/config/config.h>
#include <hydra/places/compression_graph_extractor.h>
#include <hydra/reconstruction/configs.h>
#include <hydra_ros/GvdVisualizerConfig.h>

#include "hydra_ros/visualizer/gvd_visualization_utilities.h"
#include "hydra_ros/visualizer/visualizer_types.h"
#include "hydra_ros/visualizer/visualizer_utilities.h"

namespace hydra {

using hydra_ros::GvdVisualizerConfig;
using RqtMutexPtr = std::unique_ptr<boost::recursive_mutex>;

struct TopologyVisualizerConfig {
  std::string world_frame = "world";
  std::string topology_marker_ns = "topology_graph";
  bool show_block_outlines = false;
  bool use_gvd_block_outlines = false;
  double outline_scale = 0.01;

  ColormapConfig colormap;
  GvdVisualizerConfig gvd;
  VisualizerConfig graph;
  LayerConfig graph_layer;
};

template <typename Visitor>
void visit_config(const Visitor& v, TopologyVisualizerConfig& config) {
  v.visit("world_frame", config.world_frame);
  v.visit("topology_marker_ns", config.topology_marker_ns);
  v.visit("show_block_outlines", config.show_block_outlines);
  v.visit("use_gvd_block_outlines", config.use_gvd_block_outlines);
  v.visit("outline_scale", config.outline_scale);
}

class TopologyServerVisualizer {
 public:
  explicit TopologyServerVisualizer(const std::string& ns);

  virtual ~TopologyServerVisualizer() = default;

  void visualize(const SceneGraphLayer& graph,
                 const places::GvdGraph& gvd_graph,
                 const voxblox::Layer<places::GvdVoxel>& gvd,
                 const voxblox::Layer<voxblox::TsdfVoxel>& tsdf,
                 uint64_t timestamp_ns,
                 const voxblox::MeshLayer* mesh = nullptr);

  void visualizeError(const voxblox::Layer<places::GvdVoxel>& lhs,
                      const voxblox::Layer<places::GvdVoxel>& rhs,
                      double threshold,
                      uint64_t timestamp_ns);

  void visualizeExtractor(uint64_t timestamp_ns,
                          const places::CompressionGraphExtractor& extractor);

 private:
  void visualizeGraph(const std_msgs::Header& header, const SceneGraphLayer& graph);

  void visualizeGvd(const std_msgs::Header& header,
                    const voxblox::Layer<places::GvdVoxel>& gvd) const;

  void visualizeGvdGraph(const std_msgs::Header& header,
                         const places::GvdGraph& gvd_graph) const;

  void visualizeBlocks(const std_msgs::Header& header,
                       const voxblox::Layer<places::GvdVoxel>& gvd,
                       const voxblox::Layer<voxblox::TsdfVoxel>& tsdf,
                       const voxblox::MeshLayer* mesh) const;

  void publishGraphLabels(const std_msgs::Header& header, const SceneGraphLayer& graph);

  void publishFreespace(const std_msgs::Header& header, const SceneGraphLayer& graph);

  void gvdConfigCb(GvdVisualizerConfig& config, uint32_t level);

  void graphConfigCb(LayerConfig& config, uint32_t level);

  void colormapCb(ColormapConfig& config, uint32_t level);

  void setupConfigServers();

  template <typename Config, typename Callback>
  void startRqtServer(const std::string& config_ns,
                      std::unique_ptr<dynamic_reconfigure::Server<Config>>& server,
                      const Callback& callback) {
    ros::NodeHandle config_nh(nh_, config_ns);
    server.reset(new dynamic_reconfigure::Server<Config>(config_nh));
    server->setCallback(boost::bind(callback, this, _1, _2));
  }

 private:
  ros::NodeHandle nh_;
  std::unique_ptr<MarkerGroupPub> pubs_;

  TopologyVisualizerConfig config_;
  std::set<int> previous_labels_;
  size_t previous_spheres_;

  mutable bool published_gvd_graph_;
  mutable bool published_gvd_clusters_;

  std::unique_ptr<dynamic_reconfigure::Server<GvdVisualizerConfig>> gvd_config_server_;
  std::unique_ptr<dynamic_reconfigure::Server<LayerConfig>> graph_config_server_;
  std::unique_ptr<dynamic_reconfigure::Server<ColormapConfig>> colormap_server_;
};

}  // namespace hydra

DECLARE_CONFIG_OSTREAM_OPERATOR(hydra, TopologyVisualizerConfig)
