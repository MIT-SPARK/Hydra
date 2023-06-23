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
#include "hydra/config/config.h"

namespace hydra {

enum class RoomClusterMode { MODULARITY, MODULARITY_DISTANCE, NEIGHBORS, NONE };

enum class DilationThresholdMode {
  REPEATED,
  LONGEST_LIFETIME,
  PLATEAU,
  PLATEAU_THRESHOLD
};

}  // namespace hydra

DECLARE_CONFIG_ENUM(hydra,
                    RoomClusterMode,
                    {RoomClusterMode::MODULARITY, "MODULARITY"},
                    {RoomClusterMode::MODULARITY, "MODULARITY_DISTANCE"},
                    {RoomClusterMode::NEIGHBORS, "NEIGHBORS"},
                    {RoomClusterMode::NONE, "NONE"})

DECLARE_CONFIG_ENUM(hydra,
                    DilationThresholdMode,
                    {DilationThresholdMode::REPEATED, "REPEATED"},
                    {DilationThresholdMode::LONGEST_LIFETIME, "LONGEST_LIFETIME"},
                    {DilationThresholdMode::PLATEAU, "PLATEAU"},
                    {DilationThresholdMode::PLATEAU_THRESHOLD, "PLATEAU_THRESHOLD"})

namespace hydra {

struct RoomFinderConfig {
  char room_prefix = 'R';
  double min_dilation_m = 0.1;
  double max_dilation_m = 0.7;
  double min_window_size = 0.2;
  bool clip_dilation_window_to_max = false;
  size_t min_component_size = 15;
  size_t min_room_size = 10;
  DilationThresholdMode dilation_threshold_mode = DilationThresholdMode::REPEATED;
  double min_lifetime_length_m = 0.1;
  double plateau_ratio = 0.5;
  RoomClusterMode clustering_mode = RoomClusterMode::NEIGHBORS;
  double max_modularity_iters = 5;
  double modularity_gamma = 1.0;
  double dilation_diff_threshold_m = 1.0e-4;
  bool log_filtrations = false;
  bool log_place_graphs = false;
};

template <typename Visitor>
void visit_config(const Visitor& v, RoomFinderConfig& config) {
  v.visit("min_dilation_m", config.min_dilation_m);
  v.visit("max_dilation_m", config.max_dilation_m);
  v.visit("min_window_size", config.min_window_size);
  v.visit("clip_dilation_window_to_max", config.clip_dilation_window_to_max);
  v.visit("min_component_size", config.min_component_size);
  v.visit("min_room_size", config.min_room_size);
  v.visit("dilation_threshold_mode", config.dilation_threshold_mode);
  v.visit("min_lifetime_length_m", config.min_lifetime_length_m);
  v.visit("plateau_ratio", config.plateau_ratio);
  v.visit("max_modularity_iters", config.max_modularity_iters);
  v.visit("modularity_gamma", config.modularity_gamma);
  v.visit("clustering_mode", config.clustering_mode);
  v.visit("dilation_diff_threshold_m", config.dilation_diff_threshold_m);
  v.visit("log_filtrations", config.log_filtrations);
  v.visit("log_place_graphs", config.log_place_graphs);

  std::string prefix_string;
  if (!config_parser::is_parser<Visitor>()) {
    prefix_string.push_back(config.room_prefix);
  }

  v.visit("room_prefix", prefix_string);
  if (config_parser::is_parser<Visitor>()) {
    config.room_prefix = prefix_string[0];
  }
}

}  // namespace hydra

DECLARE_CONFIG_OSTREAM_OPERATOR(hydra, RoomFinderConfig)
