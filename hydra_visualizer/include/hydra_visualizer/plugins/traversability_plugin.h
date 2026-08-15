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
#include <spark_dsg/traversability_boundary.h>

#include <visualization_msgs/msg/marker_array.hpp>

#include "hydra_visualizer/color/colormap_utilities.h"
#include "hydra_visualizer/plugins/visualizer_plugin.h"
#include "hydra_visualizer/utils/marker_tracker.h"

namespace hydra {

class TraversabilityPlugin : public VisualizerPlugin {
 public:
  struct Config {
    //! Layer to visualize
    std::string layer = spark_dsg::DsgLayers::TRAVERSABILITY;

    //! Colors representing each traversability state.
    std::vector<spark_dsg::Color> colors{spark_dsg::Color::gray(),    // Unknown
                                         spark_dsg::Color::blue(),    // Traversable
                                         spark_dsg::Color::red(),     // Intraversable
                                         spark_dsg::Color::green()};  // Traversed
    //! Height of the slice to visualize in meters.
    float slice_height = 2.0f;

    //! line width of the boundary markers
    float line_width = 0.07f;
  };

  TraversabilityPlugin(const Config& config,
                       ianvs::NodeHandle nh,
                       const std::string& name);

  virtual ~TraversabilityPlugin() = default;

  void draw(const std_msgs::msg::Header& header,
            const spark_dsg::DynamicSceneGraph& graph) override;

  void reset(const std_msgs::msg::Header& header) override;

  YAML::Node dumpConfig() const override;

 protected:
  void fillMarkers(const std_msgs::msg::Header& header,
                   const spark_dsg::DynamicSceneGraph& graph,
                   visualization_msgs::msg::MarkerArray& msg) const;

  void drawBoundaries(const Config& config,
                      const std_msgs::msg::Header& header,
                      const spark_dsg::SceneGraphLayer& layer,
                      visualization_msgs::msg::MarkerArray& msg) const;

  void drawBlockBoundary(const Config& config,
                         const spark_dsg::TraversabilityNodeAttributes& attrs,
                         visualization_msgs::msg::Marker& marker) const;

  void drawRegionBoundary(const Config& config,
                          const spark_dsg::TravNodeAttributes& attrs,
                          visualization_msgs::msg::Marker& marker) const;

  void addBoundaryPoint(const Config& config,
                        visualization_msgs::msg::Marker& marker,
                        const Eigen::Vector3d& point,
                        spark_dsg::TraversabilityState state,
                        bool last_point = false) const;

  config::DynamicConfig<Config> config_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_;
  mutable MarkerTracker tracker_;

  // Start and stop indices for the corner points of the boundary along the state
  // direction.
  inline static const std::array<std::pair<size_t, size_t>, 4> state_pairs_ = {
      std::make_pair(1, 0),
      std::make_pair(1, 2),
      std::make_pair(2, 3),
      std::make_pair(0, 3)};
};

void declare_config(TraversabilityPlugin::Config& config);

}  // namespace hydra
