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
#include "hydra_utils/dynamic_scene_graph_visualizer.h"

namespace hydra {

inline double getDynamicHue(const DynamicLayerConfig& config, char prefix) {
  // distance is measured from first relatively readable character prefix
  int color_num = (std::abs((prefix - '0')) + config.color_offset) % config.num_colors;
  return static_cast<double>(color_num) / static_cast<double>(config.num_colors);
}

NodeColor getNodeColor(const DynamicLayerConfig& config, char prefix) {
  const double hue = getDynamicHue(config, prefix);
  return dsg_utils::getRgbFromHls(hue, config.luminance, config.saturation);
}

NodeColor getEdgeColor(const DynamicLayerConfig& config, char prefix) {
  const double hue = getDynamicHue(config, prefix);
  const double saturation = config.saturation * config.edge_sl_ratio;
  const double luminance = config.luminance * config.edge_sl_ratio;
  return dsg_utils::getRgbFromHls(hue, saturation, luminance);
}

void DynamicSceneGraphVisualizer::drawDynamicLayer(const std_msgs::Header& header,
                                                   const DynamicSceneGraphLayer& layer,
                                                   const DynamicLayerConfig& config,
                                                   const VisualizerConfig& viz_config,
                                                   size_t viz_idx,
                                                   MarkerArray& msg) {
  const std::string node_ns = getDynamicNodeNamespace(layer.prefix);
  Marker nodes = makeDynamicCentroidMarkers(header,
                                            config,
                                            layer,
                                            viz_config,
                                            getNodeColor(config, layer.prefix),
                                            node_ns,
                                            viz_idx);
  addMultiMarkerIfValid(nodes, msg);

  const std::string edge_ns = getDynamicEdgeNamespace(layer.prefix);
  Marker edges = makeDynamicEdgeMarkers(header,
                                        config,
                                        layer,
                                        viz_config,
                                        getEdgeColor(config, layer.prefix),
                                        edge_ns,
                                        viz_idx);
  addMultiMarkerIfValid(edges, msg);

  if (layer.numNodes() == 0) {
    deleteLabel(header, layer.prefix, msg);
    return;
  }

  const std::string label_ns = getDynamicLabelNamespace(layer.prefix);
  Marker label =
      makeDynamicLabelMarker(header, config, layer, viz_config, label_ns, viz_idx);
  msg.markers.push_back(label);
  published_dynamic_labels_.insert(label_ns);
}

void DynamicSceneGraphVisualizer::drawDynamicLayers(const std_msgs::Header& header,
                                                    MarkerArray& msg) {
  const VisualizerConfig& viz_config = visualizer_config_->get();

  for (const auto& id_layer_map_pair : scene_graph_->dynamicLayers()) {
    const LayerId layer_id = id_layer_map_pair.first;
    if (!layer_configs_.count(layer_id)) {
      continue;
    }

    const DynamicLayerConfig& config = getConfig(layer_id);

    size_t viz_layer_idx = 0;
    for (const auto& prefix_layer_pair : id_layer_map_pair.second) {
      if (!config.visualize) {
        deleteDynamicLayer(header, prefix_layer_pair.first, msg);
        continue;
      }

      const DynamicSceneGraphLayer& layer = *prefix_layer_pair.second;
      drawDynamicLayer(header, layer, config, viz_config, viz_layer_idx, msg);
      viz_layer_idx++;
    }
  }
}

}  // namespace hydra
