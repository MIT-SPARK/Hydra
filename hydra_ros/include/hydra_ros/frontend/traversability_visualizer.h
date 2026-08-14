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
#include <hydra/frontend/traversability_place_extractor.h>
#include <hydra_visualizer/color/colormap_utilities.h>
#include <ianvs/node_handle.h>

#include <mutex>

#include <visualization_msgs/msg/marker.hpp>

namespace hydra {

class TraversabilityVisualizer : public places::TraversabilityPlaceExtractor::Sink {
 public:
  struct Config {
    //! Namespace to use for the visualizer node.
    std::string ns = "~/traversability";
    //! Colormap to use for traversability values.
    visualizer::RangeColormap::Config traversability_colormap{
        createSpectrumColormap({spark_dsg::Color::red(),
                                spark_dsg::Color::yellow(),
                                spark_dsg::Color::green()})};
    //! Colormap to use for confidence values.
    visualizer::RangeColormap::Config confidence_colormap{
        createSpectrumColormap({spark_dsg::Color::black(),
                                spark_dsg::Color::blue(),
                                spark_dsg::Color::cyan()})};
    //! Colors to use for the different traversability states (unknown, traversable,
    //! intraversable, traversed).
    std::vector<spark_dsg::Color> state_colors{spark_dsg::Color::black(),
                                               spark_dsg::Color::blue(),
                                               spark_dsg::Color::red(),
                                               spark_dsg::Color::green()};
    //! Height where the slice is visualized [m].
    float drawing_offset_z = 0.0f;
    //! True: Use height relative to the robot, false: use absolute height.
    bool use_relative_offset = true;
    //! Alpha value to use for the voxel colors.
    double alpha = 0.3;
  };

  explicit TraversabilityVisualizer(const Config& config);

  virtual ~TraversabilityVisualizer() = default;

  std::string printInfo() const override;

  void call(uint64_t timestamp_ns,
            const Eigen::Vector3d& world_t_body,
            const places::TraversabilityLayer& layer) const override;

 protected:
  config::DynamicConfig<Config> config_;
  Config active_config_;  // Current config to use for a call.
  mutable std::mutex config_mutex_;
  ianvs::NodeHandle nh_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr layer_pub_;
  std::unique_ptr<visualizer::RangeColormap> traversability_colormap_;
  std::unique_ptr<visualizer::RangeColormap> confidence_colormap_;

  void visualizeLayer(const std_msgs::msg::Header& header,
                      const Eigen::Vector3d& world_t_body,
                      const places::TraversabilityLayer& layer) const;

  static visualizer::RangeColormap::Config createSpectrumColormap(
      std::vector<spark_dsg::Color> colors);

  spark_dsg::Color debugColor(float value) const;

  void onConfigUpdate();

 private:
  inline static const auto registration_ =
      config::RegistrationWithConfig<places::TraversabilityPlaceExtractor::Sink,
                                     TraversabilityVisualizer,
                                     Config>("TraversabilityVisualizer");
};

void declare_config(TraversabilityVisualizer::Config& config);

}  // namespace hydra
