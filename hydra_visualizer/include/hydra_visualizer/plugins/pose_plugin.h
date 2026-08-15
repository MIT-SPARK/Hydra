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
#include <ianvs/node_handle.h>

#include <geometry_msgs/msg/pose_array.hpp>
#include <rclcpp/publisher.hpp>

#include "hydra_visualizer/plugins/visualizer_plugin.h"

namespace hydra {

class PosePlugin : public VisualizerPlugin {
 public:
  struct Config {
    size_t num_to_skip = 0;
    std::string layer = spark_dsg::DsgLayers::AGENTS;
    spark_dsg::PartitionId partition = 'a';
    size_t num_received_before_warn = 500;
  } const config;

  PosePlugin(const Config& config, ianvs::NodeHandle nh, const std::string& name);

  virtual ~PosePlugin() = default;

  void draw(const std_msgs::msg::Header& header,
            const spark_dsg::DynamicSceneGraph& graph) override;

  void reset(const std_msgs::msg::Header& header) override;

  YAML::Node dumpConfig() const override;

 protected:
  size_t num_received_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pub_;
};

void declare_config(PosePlugin::Config& config);

}  // namespace hydra
