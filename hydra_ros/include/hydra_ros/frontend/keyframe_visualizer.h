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
#include <hydra/frontend/keyframe_selector.h>
#include <hydra_visualizer/utils/marker_group_pub.h>
#include <ianvs/node_handle.h>

namespace hydra {

class KeyframeVisualizer : public KeyframeSelector::Sink {
 public:
  struct Config {
    //! ROS namespace for sink
    std::string ns = "~/keyframes";
    //! Distance from position that the image plane is draw
    double far_distance = 1.0;
    //! Marker line width
    double line_width = 0.01;
    //! Color to use
    spark_dsg::Color color = spark_dsg::Color::blue();
    //! Alpha for image plane
    double image_plane_alpha = 0.1;
    //! Draw image plane for both viewing directions
    bool draw_both_image_plane_sides = true;
  } const config;

  KeyframeVisualizer(const Config& config);

  void call(uint64_t timestamp_ns,
            const KeyframeSelector::Keyframes& frames) const override;

 private:
  ianvs::NodeHandle nh_;
  MarkerGroupPub pubs_;
  config::DynamicConfig<Config> config_;
};

void declare_config(KeyframeVisualizer::Config& config);

}  // namespace hydra
