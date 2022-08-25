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
#include "hydra_dsg_builder/config_utils.h"
#include "hydra_dsg_builder/incremental_mesh_segmenter.h"

#include <kimera_pgmo/MeshFrontendInterface.h>

namespace spark_dsg {

using BoundingBoxType = BoundingBox::Type;

}  // namespace spark_dsg

DECLARE_CONFIG_ENUM(spark_dsg,
                    BoundingBoxType,
                    {BoundingBoxType::INVALID, "INVALID"},
                    {BoundingBoxType::AABB, "AABB"},
                    {BoundingBoxType::OBB, "OBB"},
                    {BoundingBoxType::RAABB, "RAABB"});

namespace kimera_pgmo {

template <typename Visitor>
void visit_config(const Visitor& v, kimera_pgmo::MeshFrontendConfig& config) {
  v.visit("robot_id", config.robot_id);
  v.visit("horizon", config.time_horizon);
  v.visit("track_mesh_graph_mapping", config.b_track_mesh_graph_mapping);
  v.visit("log_path", config.log_path);
  v.visit("should_log", config.log_output);
  v.visit("full_compression_method", config.full_compression_method);
  v.visit("graph_compression_method", config.graph_compression_method);
  v.visit("d_graph_resolution", config.d_graph_resolution);
  v.visit("output_mesh_resolution", config.mesh_resolution);
}

}  // namespace kimera_pgmo

namespace hydra {
namespace incremental {

struct DsgFrontendConfig {
  // TODO(nathan) consider unifying log path with backend
  bool should_log = true;
  std::string log_path;
  size_t min_object_vertices = 20;
  bool prune_mesh_indices = false;
  std::string semantic_label_file;
  kimera_pgmo::MeshFrontendConfig pgmo_config;
  MeshSegmenterConfig object_config;
};

template <typename Visitor>
void visit_config(const Visitor& v, MeshSegmenterConfig& config) {
  std::string prefix_string;
  if (!config_parser::is_parser<Visitor>()) {
    prefix_string.push_back(config.prefix);
  }

  v.visit("prefix", prefix_string);
  if (config_parser::is_parser<Visitor>()) {
    config.prefix = prefix_string.at(0);
  }

  v.visit("active_horizon_s", config.active_horizon_s);
  v.visit("active_index_horizon_m", config.active_index_horizon_m);
  v.visit("cluster_tolerance", config.cluster_tolerance);
  v.visit("min_cluster_size", config.min_cluster_size);
  v.visit("max_cluster_size", config.max_cluster_size);
  v.visit("bounding_box_type", config.bounding_box_type);
  v.visit("labels", config.labels);
}

template <typename Visitor>
void visit_config(const Visitor& v, DsgFrontendConfig& config) {
  // TODO(nathan) replace with single param (derive should_log from log_path)
  v.visit("should_log", config.should_log);
  v.visit("log_path", config.log_path);
  v.visit("min_object_vertices", config.min_object_vertices);
  v.visit("prune_mesh_indices", config.prune_mesh_indices);
  v.visit("semantic_label_file", config.semantic_label_file);
  v.visit("pgmo", config.pgmo_config);
  v.visit("objects", config.object_config);
}

}  // namespace incremental
}  // namespace hydra

DECLARE_CONFIG_OSTREAM_OPERATOR(kimera_pgmo, MeshFrontendConfig)
DECLARE_CONFIG_OSTREAM_OPERATOR(hydra::incremental, MeshSegmenterConfig)
DECLARE_CONFIG_OSTREAM_OPERATOR(hydra::incremental, DsgFrontendConfig)
