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

#include <config_utilities/dynamic_config.h>
#include <ianvs/node_handle.h>

#include <rclcpp/publisher.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "hydra_visualizer/color/colormap_utilities.h"
#include "hydra_visualizer/layer_info.h"
#include "hydra_visualizer/plugins/visualizer_plugin.h"
#include "hydra_visualizer/utils/marker_tracker.h"

namespace hydra {

class BasisPointPlugin : public VisualizerPlugin {
 public:
  struct Config {
    bool show_voxblox_connections = false;
    bool draw_basis_points = true;
    bool draw_layer_edges = false;
    double places_edge_scale = 0.05;
    double places_edge_alpha = 0.5;
    double basis_point_scale = 0.1;
    double basis_point_alpha = 0.8;
    visualizer::CategoricalColormap::Config colormap;
    visualizer::LayerConfig graph;
  };

  BasisPointPlugin(const Config& config, ianvs::NodeHandle nh, const std::string& name);

  virtual ~BasisPointPlugin() = default;

  void draw(const std_msgs::msg::Header& header,
            const spark_dsg::DynamicSceneGraph& graph) override;

  void reset(const std_msgs::msg::Header& header) override;

  YAML::Node dumpConfig() const override;

 protected:
  void fillMarkers(const std_msgs::msg::Header& header,
                   const spark_dsg::DynamicSceneGraph& graph,
                   visualization_msgs::msg::MarkerArray& msg) const;

  void drawEdges(const Config& config,
                 const visualizer::CategoricalColormap& colormap,
                 const std_msgs::msg::Header& header,
                 const spark_dsg::DynamicSceneGraph& graph,
                 visualization_msgs::msg::MarkerArray& msg) const;

  void drawBasisPoints(const Config& config,
                       const visualizer::CategoricalColormap& colormap,
                       const std_msgs::msg::Header& header,
                       const spark_dsg::DynamicSceneGraph& graph,
                       visualization_msgs::msg::MarkerArray& msg) const;

  mutable MarkerTracker tracker_;
  config::DynamicConfig<Config> config_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_;
};

void declare_config(BasisPointPlugin::Config& config);

}  // namespace hydra
