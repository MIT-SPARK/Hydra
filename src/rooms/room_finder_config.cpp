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
#include "hydra/rooms/room_finder_config.h"

#include <config_utilities/config.h>
#include <config_utilities/types/conversions.h>
#include <config_utilities/types/enum.h>

namespace hydra {

void declare_config(RoomFinderConfig& conf) {
  using namespace config;
  name("RoomFinderConfig");
  field<CharConversion>(conf.room_prefix, "prefix");
  field(conf.min_dilation_m, "min_dilation_m", "m");
  field(conf.max_dilation_m, "max_dilation_m", "m");
  field(conf.min_window_size, "min_window_size");
  field(conf.clip_dilation_window_to_max, "clip_dilation_window_to_max");
  field(conf.min_component_size, "min_component_size");
  field(conf.min_room_size, "min_room_size");
  enum_field(conf.dilation_threshold_mode,
             "dilation_threshold_mode",
             {{DilationThresholdMode::REPEATED, "REPEATED"},
              {DilationThresholdMode::LONGEST_LIFETIME, "LONGEST_LIFETIME"},
              {DilationThresholdMode::PLATEAU, "PLATEAU"},
              {DilationThresholdMode::PLATEAU_THRESHOLD, "PLATEAU_THRESHOLD"}});
  field(conf.min_lifetime_length_m, "min_lifetime_length_m");
  field(conf.plateau_ratio, "plateau_ratio");
  field(conf.max_modularity_iters, "max_modularity_iters");
  field(conf.modularity_gamma, "modularity_gamma");
  enum_field(conf.clustering_mode,
             "clustering_mode",
             {{RoomClusterMode::MODULARITY, "MODULARITY"},
              {RoomClusterMode::MODULARITY, "MODULARITY_DISTANCE"},
              {RoomClusterMode::NEIGHBORS, "NEIGHBORS"},
              {RoomClusterMode::NONE, "NONE"}});
  field(conf.dilation_diff_threshold_m, "dilation_diff_threshold_m", "m");
  field(conf.log_filtrations, "log_filtrations");
  field(conf.log_place_graphs, "log_place_graphs");
  // TODO(nathan) checks
}

}  // namespace hydra
