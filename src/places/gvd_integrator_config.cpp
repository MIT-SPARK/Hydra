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
#include "hydra/places/gvd_integrator_config.h"

#include <config_utilities/config.h>
#include <config_utilities/types/enum.h>

namespace hydra::places {

void declare_config(VoronoiCheckConfig& config) {
  using namespace config;
  name("VoronoiCheckConfig");
  enum_field(config.mode,
             "mode",
             {{ParentUniquenessMode::ANGLE, "ANGLE"},
              {ParentUniquenessMode::L1_DISTANCE, "L1_DISTANCE"},
              {ParentUniquenessMode::L1_THEN_ANGLE, "L1_THEN_ANGLE"}});
  field(config.min_distance_m, "min_distance_m");
  field(config.parent_l1_separation, "parent_l1_separation");
  field(config.parent_cos_angle_separation, "parent_cos_angle_separation");
}

void declare_config(GvdIntegratorConfig& config) {
  using namespace config;
  name("GvdIntegratorConfig");
  field(config.max_distance_m, "max_distance_m");
  field(config.min_distance_m, "min_distance_m");
  field(config.min_diff_m, "min_diff_m");
  field(config.min_weight, "min_weight");
  field(config.num_buckets, "num_buckets");
  field(config.multi_queue, "multi_queue");
  field(config.refine_voxel_pos, "refine_voxel_pos");
  field(config.positive_distance_only, "positive_distance_only");
  field(config.min_basis_for_extraction, "min_basis_for_extraction");
  field(config.voronoi_config, "voronoi_config");
}

}  // namespace hydra::places
