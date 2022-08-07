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
#include "hydra_dsg_builder/dsg_lcd_detector.h"

#include <hydra_utils/dynamic_scene_graph_visualizer.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

namespace hydra {
namespace lcd {

using visualization_msgs::Marker;
using visualization_msgs::MarkerArray;

class LcdVisualizer : public DynamicSceneGraphVisualizer {
 public:
  using DynamicLayerConfigManager = ConfigManager<DynamicLayerConfig>;

  static DynamicSceneGraph::LayerIds getDefaultLayerIds() {
    return {DsgLayers::OBJECTS, DsgLayers::PLACES};
  }

  LcdVisualizer(const ros::NodeHandle& nh, double radius);

  void setLcdDetector(DsgLcdDetector* detector);

 protected:
  virtual void redrawImpl(const std_msgs::Header& header, MarkerArray& msg) override;

  virtual void drawLayer(const std_msgs::Header& header,
                         const SceneGraphLayer& layer,
                         const LayerConfig& config,
                         MarkerArray& msg) override;

  virtual void drawLayerMeshEdges(const std_msgs::Header& header,
                                  LayerId layer_id,
                                  const std::string& ns,
                                  MarkerArray& msg) override;

  void drawAgent(const std_msgs::Header& header, MarkerArray& msg);

  std::optional<NodeId> getQueryNode() const;

  std::vector<NodeId> getMatchRoots(LayerId layer) const;

  std::set<NodeId> getMatchedNodes(LayerId layer) const;

  std::set<NodeId> getValidNodes(LayerId layer) const;

 private:
  DsgLcdDetector* lcd_detector_;

  double radius_;

  DynamicLayerConfigManager::Ptr agent_config_;
  NodeColor invalid_match_color_;
  NodeColor valid_match_color_;
  NodeColor default_graph_color_;
  NodeColor query_color_;
};

}  // namespace lcd
}  // namespace hydra
