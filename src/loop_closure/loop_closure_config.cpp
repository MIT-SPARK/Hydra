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
#include "hydra/loop_closure/loop_closure_config.h"

#include <config_utilities/config.h>
#include <config_utilities/types/eigen_matrix.h>
#include <config_utilities/types/enum.h>

namespace hydra {

void declare_config(SubgraphConfig& conf) {
  using namespace config;
  name("SubgraphConfig");
  field(conf.fixed_radius, "fixed_radius");
  field(conf.max_radius_m, "max_radius_m");
  if (!conf.fixed_radius) {
    field(conf.min_radius_m, "min_radius_m");
    field(conf.min_nodes, "min_nodes");
  }
}

namespace lcd {

void declare_config(LayerRegistrationConfig& conf) {
  using namespace config;
  name("LayerRegistrationConfig");
  field(conf.min_correspondences, "min_correspondences");
  field(conf.min_inliers, "min_inliers");
  field(conf.max_same_nodes, "max_same_nodes");
  field(conf.log_registration_problem, "log_registration_problem");
  field(conf.registration_output_path, "registration_output_path");
  field(conf.recreate_subgraph, "recreate_subgraph");
  if (conf.recreate_subgraph) {
    field(conf.subgraph_extraction, "subgraph_extraction");
  }
}

void declare_config(DescriptorMatchConfig& conf) {
  using namespace config;
  name("DescriptorMatchConfig");
  field(conf.min_score, "min_score");
  field(conf.min_registration_score, "min_registration_score");
  field(conf.min_time_separation_s, "min_time_separation_s");
  field(conf.max_registration_matches, "max_registration_matches");
  field(conf.min_score_ratio, "min_score_ratio");
  field(conf.min_match_separation_m, "min_match_separation_m");
  enum_field(
      conf.type,
      "type",
      {{DescriptorScoreType::COSINE, "COSINE"}, {DescriptorScoreType::L1, "L1"}});
}

}  // namespace lcd
}  // namespace hydra
