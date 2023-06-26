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
#include "hydra/places/graph_extractor_config.h"

namespace hydra {
namespace places {

enum class ParentUniquenessMode {
  ANGLE,
  L1_DISTANCE,
  L1_THEN_ANGLE,
};

}  // namespace places
}  // namespace hydra

namespace hydra {
namespace places {

struct VoronoiCheckConfig {
  ParentUniquenessMode mode = ParentUniquenessMode::L1_THEN_ANGLE;
  double min_distance_m = 0.2;
  double parent_l1_separation = 3.0;
  double parent_cos_angle_separation = 0.5;
};

struct GvdIntegratorConfig {
  float max_distance_m = 2.0f;
  float min_distance_m = 0.2f;
  float min_diff_m = 1.0e-3f;
  float min_weight = 1.0e-6f;
  int num_buckets = 20;
  bool multi_queue = false;
  bool positive_distance_only = true;
  uint8_t min_basis_for_extraction = 3;
  VoronoiCheckConfig voronoi_config;
  bool extract_graph = true;
  GraphExtractorConfig graph_extractor;
};

template <typename Visitor>
void visit_config(const Visitor& v, VoronoiCheckConfig& config) {
  v.visit("mode", config.mode);
  v.visit("min_distance_m", config.min_distance_m);
  v.visit("parent_l1_separation", config.parent_l1_separation);
  v.visit("parent_cos_angle_separation", config.parent_cos_angle_separation);
}

template <typename Visitor>
void visit_config(const Visitor& v, GvdIntegratorConfig& config) {
  v.visit("max_distance_m", config.max_distance_m);
  v.visit("min_distance_m", config.min_distance_m);
  v.visit("min_diff_m", config.min_diff_m);
  v.visit("min_weight", config.min_weight);
  v.visit("num_buckets", config.num_buckets);
  v.visit("multi_queue", config.multi_queue);
  v.visit("positive_distance_only", config.positive_distance_only);
  v.visit("min_basis_for_extraction", config.min_basis_for_extraction);
  v.visit("voronoi_config", config.voronoi_config);
  v.visit("extract_graph", config.extract_graph);
  v.visit("graph_extractor", config.graph_extractor);
}

}  // namespace places
}  // namespace hydra
