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
#include <hydra/active_window/active_window_module.h>
#include <hydra/input/sensor_map.h>
#include <hydra_visualizer/adapters/mesh_color.h>
#include <hydra_visualizer/color/colormap_utilities.h>
#include <hydra_visualizer/utils/marker_group_pub.h>
#include <ianvs/lazy_publisher_group.h>
#include <ianvs/node_handle.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <kimera_pgmo_msgs/msg/mesh.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "hydra_ros/utils/input_data_to_messages.h"

namespace hydra {

class ReconstructionVisualizer : public ActiveWindowModule::Sink {
 public:
  struct SensorDisplay {
    using Config = DisplayConfig;
    const Config config;
    explicit SensorDisplay(const Config& config) : config(config) {}
  };

  struct Config {
    std::string ns = "~/reconstruction";
    double min_weight = 0.0;
    double max_weight = 10.0;
    double marker_alpha = 0.5;
    bool use_relative_height = true;
    double slice_height = 0.0;
    double min_observation_weight = 1.0e-5;
    double tsdf_block_scale = 0.02;
    double tsdf_block_alpha = 1.0;
    spark_dsg::Color tsdf_block_color = spark_dsg::Color::green();
    double mesh_block_scale = 0.02;
    double mesh_block_alpha = 1.0;
    spark_dsg::Color mesh_block_color = spark_dsg::Color::red();
    double point_size = 0.04;
    bool filter_points_by_range = true;
    visualizer::RangeColormap::Config colormap;
    visualizer::CategoricalColormap::Config label_colormap;
    config::VirtualConfig<MeshColoring> mesh_coloring;
    SensorMap<SensorDisplay>::Config sensor_displays;
  } const config;

  explicit ReconstructionVisualizer(const Config& config);

  virtual ~ReconstructionVisualizer();

  std::string printInfo() const override;

  void call(uint64_t timestamp_ns,
            const VolumetricMap& tsdf,
            const ActiveWindowOutput& msg) const override;

 protected:
  void publishMesh(const ActiveWindowOutput& output) const;

  ianvs::NodeHandle nh_;
  MarkerGroupPub pubs_;
  rclcpp::Publisher<kimera_pgmo_msgs::msg::Mesh>::SharedPtr active_mesh_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  SensorMap<SensorDisplay> sensor_displays_;
  ianvs::RosPublisherGroup<sensor_msgs::msg::Image> image_pubs_;
  ianvs::RosPublisherGroup<sensor_msgs::msg::PointCloud2> cloud_pubs_;
  const visualizer::RangeColormap colormap_;
  const visualizer::CategoricalColormap label_colormap_;
  std::shared_ptr<MeshColoring> mesh_coloring_;

 private:
  inline static const auto registration_ =
      config::RegistrationWithConfig<ActiveWindowModule::Sink,
                                     ReconstructionVisualizer,
                                     Config>("ReconstructionVisualizer");
};

void declare_config(ReconstructionVisualizer::Config& config);

}  // namespace hydra
