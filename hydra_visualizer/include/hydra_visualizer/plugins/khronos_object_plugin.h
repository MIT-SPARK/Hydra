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

// Adapted from Khronos, original notice replicated below:
/** -----------------------------------------------------------------------------
 * Copyright (c) 2024 Massachusetts Institute of Technology.
 * All Rights Reserved.
 *
 * AUTHORS:      Lukas Schmid <lschmid@mit.edu>, Marcus Abate <mabate@mit.edu>,
 *               Yun Chang <yunchang@mit.edu>, Luca Carlone <lcarlone@mit.edu>
 * AFFILIATION:  MIT SPARK Lab, Massachusetts Institute of Technology
 * YEAR:         2024
 * SOURCE:       https://github.com/MIT-SPARK/Khronos
 * LICENSE:      BSD 3-Clause
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * -------------------------------------------------------------------------- */

#pragma once

#include <config_utilities/dynamic_config.h>
#include <ianvs/node_handle.h>
#include <spark_dsg/color.h>
#include <spark_dsg/node_attributes.h>
#include <tf2_ros/transform_broadcaster.h>

#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <kimera_pgmo_msgs/msg/mesh.hpp>
#include <rclcpp/publisher.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "hydra_visualizer/plugins/visualizer_plugin.h"
#include "hydra_visualizer/utils/marker_tracker.h"

namespace hydra {

/**
 * @brief Plugin to visualize KhronosObjectAttributes, i.e. meshes of static object
 * fragments and trajectories of dynamic object fragments.
 */
class KhronosObjectPlugin : public VisualizerPlugin {
 public:
  // Config.
  struct Config {
    //! Queue size for publishing.
    int queue_size = 100;

    //! How to color the trajectories of dynamic objects.
    enum class DynamicColorMode {
      ID,
      SEMANTIC,
      CONSTANT
    } dynamic_color_mode = DynamicColorMode::ID;

    //! Constant color for dynamic objects.
    spark_dsg::Color dynamic_color = spark_dsg::Color::pink();

    //! Width of the dynamic bounding box lines in meters.
    float dynamic_bbox_scale = 0.05f;

    //! Number of color revolutions used to recolor integer IDs.
    int id_color_revolutions = 10;

    //! Layer to draw objects for
    std::string layer = spark_dsg::DsgLayers::OBJECTS;
  };

  // Construction.
  KhronosObjectPlugin(const Config& config,
                      ianvs::NodeHandle nh,
                      const std::string& name);

  // Implement visualization interfaces.
  void draw(const std_msgs::msg::Header& header,
            const spark_dsg::DynamicSceneGraph& graph) override;
  void reset(const std_msgs::msg::Header& header) override;

  YAML::Node dumpConfig() const override;

 protected:
  // Helper functions.
  void drawDynamicObjects(const Config& config,
                          const std_msgs::msg::Header& header,
                          const spark_dsg::DynamicSceneGraph& graph);
  void drawStaticObjects(const Config& config,
                         const std_msgs::msg::Header& header,
                         const spark_dsg::DynamicSceneGraph& graph);

  // Mesh namespace for the visualizer plugin.
  static std::string getNamespace(const uint64_t id);

  // TF frame name.
  static std::string getFrameName(const uint64_t id);

  // Publish Tf transform for a static object.
  void publishTransform(const std_msgs::msg::Header& header,
                        const spark_dsg::KhronosObjectAttributes& attrs,
                        const uint64_t id);

  // Get the color of a dynamic object.
  spark_dsg::Color getDynamicColor(const Config& config,
                                   const spark_dsg::KhronosObjectAttributes& attrs,
                                   const uint64_t id) const;

  // Clear static object visuliazation
  void resetObject(const std_msgs::msg::Header& header, const uint64_t id);

 private:
  const config::DynamicConfig<Config> config_;
  // ROS.
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr dynamic_pub_;
  rclcpp::Publisher<kimera_pgmo_msgs::msg::Mesh>::SharedPtr static_pub_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;

  // Marker tracking.
  MarkerTracker tracker_;                          // Dynamic objects.
  std::unordered_set<uint64_t> previous_objects_;  // Static objects.
};

void declare_config(KhronosObjectPlugin::Config& config);

}  // namespace hydra
