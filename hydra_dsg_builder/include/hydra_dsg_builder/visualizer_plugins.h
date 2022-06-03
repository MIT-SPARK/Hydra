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
#include "hydra_dsg_builder/minimum_spanning_tree.h"

#include <hydra_utils/dsg_visualizer_plugin.h>
#include <hydra_utils/visualizer_types.h>
#include <kimera_pgmo/DeformationGraph.h>

#include <visualization_msgs/Marker.h>

namespace hydra {

struct PMGraphPluginConfig {
  explicit PMGraphPluginConfig(const ros::NodeHandle& nh);

  double mesh_edge_scale = 0.005;
  double mesh_edge_alpha = 0.8;
  double mesh_marker_scale = 0.1;
  double mesh_marker_alpha = 0.8;
  NodeColor leaf_color;
  NodeColor interior_color;
  NodeColor invalid_color;
  LayerConfig layer_config;
};

class MeshPlaceConnectionsPlugin : public DsgVisualizerPlugin {
 public:
  MeshPlaceConnectionsPlugin(const ros::NodeHandle& nh, const std::string& name);

  virtual ~MeshPlaceConnectionsPlugin() = default;

  void draw(const std_msgs::Header& header, const DynamicSceneGraph& graph) override;

  void reset(const std_msgs::Header& header, const DynamicSceneGraph& graph) override;

 protected:
  ros::Publisher marker_pub_;
  PMGraphPluginConfig config_;
  bool published_nodes_;
  bool published_edges_;
};

class PlacesFactorGraphViz {
 public:
  using Ptr = std::shared_ptr<PlacesFactorGraphViz>;

  explicit PlacesFactorGraphViz(const ros::NodeHandle& nh);

  virtual ~PlacesFactorGraphViz() = default;

  void draw(char vertex_prefix,
            const SceneGraphLayer& places,
            const MinimumSpanningTreeInfo& mst_info,
            const kimera_pgmo::DeformationGraph& deformations);

 protected:
  ros::NodeHandle nh_;
  ros::Publisher marker_pub_;
  PMGraphPluginConfig config_;
};

class PlaceParentsPlugin : public DsgVisualizerPlugin {
 public:
  PlaceParentsPlugin(const ros::NodeHandle& nh, const std::string& name);

  virtual ~PlaceParentsPlugin() = default;

  void draw(const std_msgs::Header& header, const DynamicSceneGraph& graph) override;

  void reset(const std_msgs::Header& header, const DynamicSceneGraph& graph) override;

 protected:
  ros::Publisher marker_pub_;
  PMGraphPluginConfig config_;
  bool published_nodes_;
  bool published_edges_;
};

}  // namespace hydra
