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
#include <spark_dsg/dynamic_scene_graph.h>

#include <visualization_msgs/msg/marker_array.hpp>

#include "hydra_visualizer/layer_info.h"
#include "hydra_visualizer/plugins/layer_plugin.h"
#include "hydra_visualizer/utils/layer_key_selector.h"
#include "hydra_visualizer/utils/marker_tracker.h"

namespace hydra {

// NOTE(nathan) separate to make config wrapper easier to use
struct GraphRenderConfig {
  //! @brief Unit amount of distance between layers
  double layer_z_step = 5.0;
  //! @brief Whether or not to separate layers by adding z offsets
  bool collapse_layers = false;
};

struct InterlayerEdgeConfig : visualizer::LayerConfig::Edges {
  InterlayerEdgeConfig();
  //! Use the child node to select the edge color instead of the parent
  bool use_child_color = false;
};

void declare_config(GraphRenderConfig& config);
void declare_config(InterlayerEdgeConfig& config);

class SceneGraphRenderer {
 public:
  using Ptr = std::shared_ptr<SceneGraphRenderer>;
  using LayerConfigWrapper = config::DynamicConfig<visualizer::LayerConfig>;
  using EdgeConfigWrapper = config::DynamicConfig<InterlayerEdgeConfig>;

  struct LayerPluginsConfig {
    spark_dsg::LayerKey layer;
    std::vector<config::VirtualConfig<LayerPlugin, true>> plugins;
  };

  struct Config {
    //! @brief Overall graph config
    GraphRenderConfig graph;
    //! @brief Configuration for each layer
    std::map<LayerKeySelector, visualizer::LayerConfig> layers;

    struct InterlayerEdges {
      LayerKeySelector from;
      LayerKeySelector to;
      // NOTE(nathan) this is awkward, but we don't want the key selectors to be
      // visibile in the dynamic config so we need to split the structs
      InterlayerEdgeConfig config;
    };
    //! @brief Configuration for interlayer edges
    std::vector<InterlayerEdges> interlayer_edges;
    //! @brief Extra per-layer plugins
    std::vector<LayerPluginsConfig> layer_plugins;
  };

  explicit SceneGraphRenderer(const Config& config, ianvs::NodeHandle nh);

  virtual ~SceneGraphRenderer() = default;

  virtual void reset(const std_msgs::msg::Header& header);

  virtual void draw(const std_msgs::msg::Header& header,
                    const spark_dsg::DynamicSceneGraph& graph) const;

  virtual bool hasChange() const;

  virtual void clearChangeFlag();

  YAML::Node dumpConfig() const;

 protected:
  virtual void setConfigs(const spark_dsg::DynamicSceneGraph& graph) const;

  virtual void drawInterlayerEdges(const std_msgs::msg::Header& header,
                                   const spark_dsg::DynamicSceneGraph& graph,
                                   visualization_msgs::msg::MarkerArray& msg) const;

  virtual void drawLayer(const std_msgs::msg::Header& header,
                         const visualizer::LayerInfo& info,
                         const spark_dsg::SceneGraphLayer& layer,
                         const spark_dsg::Mesh* mesh,
                         visualization_msgs::msg::MarkerArray& msg) const;

  const visualizer::LayerInfo& getLayerInfo(spark_dsg::LayerKey layer) const;

  visualizer::LayerConfig getLayerConfig(spark_dsg::LayerKey key) const;

  InterlayerEdgeConfig getInterlayerEdgeConfig(spark_dsg::LayerKey l1,
                                               spark_dsg::LayerKey l2) const;

 protected:
  const Config init_config_;

  ianvs::NodeHandle nh_;
  config::DynamicConfig<GraphRenderConfig> graph_config_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_;
  std::map<spark_dsg::LayerKey, std::list<LayerPlugin::Ptr>> layer_plugins_;

  mutable MarkerTracker tracker_;
  mutable std::atomic<bool> has_change_;
  mutable std::map<spark_dsg::LayerKey, visualizer::LayerInfo> layer_infos_;
  mutable std::map<spark_dsg::LayerKey, std::unique_ptr<LayerConfigWrapper>> layers_;

  struct EdgeConfigInfo {
    spark_dsg::LayerKey parent;
    spark_dsg::LayerKey child;
    std::unique_ptr<EdgeConfigWrapper> config;
  };
  mutable std::map<std::string, EdgeConfigInfo> interlayer_edges_;
};

void declare_config(SceneGraphRenderer::LayerPluginsConfig& config);
void declare_config(SceneGraphRenderer::Config& config);

}  // namespace hydra
