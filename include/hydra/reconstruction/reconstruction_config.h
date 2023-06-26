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
#include <kimera_semantics/semantic_integrator_base.h>
#include <voxblox/integrator/tsdf_integrator.h>

#include "hydra/config/config.h"
#include "hydra/config/eigen_config_types.h"
#include "hydra/reconstruction/configs.h"

DECLARE_CONFIG_ENUM(kimera,
                    ColorMode,
                    {ColorMode::kColor, "color"},
                    {ColorMode::kSemantic, "semantic"},
                    {ColorMode::kSemanticProbability, "semantic_probability"});

namespace kimera {

template <typename T>
struct AlignedConverter {
  voxblox::AlignedVector<T> to(const std::vector<T>& other) const {
    return voxblox::AlignedVector<T>(other.begin(), other.end());
  }

  std::vector<T> from(const voxblox::AlignedVector<T>& other) const {
    return std::vector<T>(other.begin(), other.end());
  }
};

template <typename Visitor>
void visit_config(const Visitor& v, SemanticIntegratorBase::SemanticConfig& config) {
  v.visit("semantic_measurement_probability", config.semantic_measurement_probability_);
  v.visit("color_mode", config.color_mode);
  v.visit("dynamic_labels", config.dynamic_labels_, AlignedConverter<SemanticLabel>());
}

}  // namespace kimera

namespace voxblox {

template <typename Visitor>
void visit_config(const Visitor& v, TsdfIntegratorBase::Config& config) {
  v.visit("default_truncation_distance", config.default_truncation_distance);
  v.visit("max_weight", config.max_weight);
  v.visit("voxel_carving_enabled", config.voxel_carving_enabled);
  v.visit("min_ray_length_m", config.min_ray_length_m);
  v.visit("max_ray_length_m", config.max_ray_length_m);
  v.visit("use_const_weight", config.use_const_weight);
  v.visit("allow_clear", config.allow_clear);
  v.visit("use_weight_dropoff", config.use_weight_dropoff);
  v.visit("use_sparsity_compensation_factor", config.use_sparsity_compensation_factor);
  v.visit("integrator_threads", config.integrator_threads, ThreadNumConverter());
  v.visit("integration_order_mode", config.integration_order_mode);
  v.visit("enable_anti_grazing", config.enable_anti_grazing);
  v.visit("start_voxel_subsampling_factor", config.start_voxel_subsampling_factor);
  v.visit("max_consecutive_ray_collisions", config.max_consecutive_ray_collisions);
  v.visit("clear_checks_every_n_frames", config.clear_checks_every_n_frames);
}

}  // namespace voxblox

namespace hydra {

struct ReconstructionConfig {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ReconstructionConfig()
      : body_R_camera(Eigen::Quaterniond::Identity()),
        body_t_camera(Eigen::Vector3d::Zero()) {}

  std::string semantic_label_file;
  float voxel_size = 0.1;
  int voxels_per_side = 16;
  bool show_stats = true;
  bool clear_distant_blocks = true;
  double dense_representation_radius_m = 5.0;
  std::string world_frame = "world";
  std::string robot_frame = "base_link";
  size_t num_poses_per_update = 1;
  size_t max_input_queue_size = 0;
  bool make_pose_graph = false;
  places::GvdIntegratorConfig gvd;
  voxblox::TsdfIntegratorBase::Config tsdf;
  kimera::SemanticIntegratorBase::SemanticConfig semantics;
  voxblox::MeshIntegratorConfig mesh;
  Eigen::Quaterniond body_R_camera;
  Eigen::Vector3d body_t_camera;
};

struct QuaternionConverter {
  QuaternionConverter() = default;

  Eigen::Quaterniond to(const std::map<std::string, double>& other) const;

  std::map<std::string, double> from(const Eigen::Quaterniond& other) const;
};

template <typename Visitor>
void visit_config(const Visitor& v, ReconstructionConfig& config) {
  v.visit("semantic_label_file", config.semantic_label_file);
  v.visit("voxel_size", config.voxel_size);
  v.visit("voxels_per_side", config.voxels_per_side);
  v.visit("show_stats", config.show_stats);
  v.visit("clear_distant_blocks", config.clear_distant_blocks);
  v.visit("dense_representation_radius_m", config.dense_representation_radius_m);
  v.visit("world_frame", config.world_frame);
  v.visit("robot_frame", config.robot_frame);
  v.visit("num_poses_per_update", config.num_poses_per_update);
  v.visit("max_input_queue_size", config.max_input_queue_size);
  v.visit("make_pose_graph", config.make_pose_graph);
  v.visit("gvd", config.gvd);
  v.visit("tsdf", config.tsdf);
  v.visit("semantics", config.semantics);
  v.visit("mesh", config.mesh);
  v.visit("body_R_camera", config.body_R_camera, QuaternionConverter());
  v.visit("body_t_camera", config.body_t_camera);
}

}  // namespace hydra

DECLARE_CONFIG_OSTREAM_OPERATOR(voxblox, TsdfIntegratorBase::Config)
DECLARE_CONFIG_OSTREAM_OPERATOR(kimera, SemanticIntegratorBase::SemanticConfig)
DECLARE_CONFIG_OSTREAM_OPERATOR(hydra, ReconstructionConfig)
