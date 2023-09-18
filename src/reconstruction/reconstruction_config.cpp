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
#include "hydra/reconstruction/reconstruction_config.h"

#include <config_utilities/config.h>
#include <config_utilities/types/conversions.h>
#include <config_utilities/types/eigen_matrix.h>

#include "hydra/common/config_utilities.h"

namespace hydra {

void declare_config(ReconstructionConfig& conf) {
  using namespace config;
  name("ReconstructionConfig");
  field(conf.voxel_size, "voxel_size");
  field(conf.voxels_per_side, "voxels_per_side");
  field(conf.show_stats, "show_stats");
  field(conf.clear_distant_blocks, "clear_distant_blocks");
  field(conf.dense_representation_radius_m, "dense_representation_radius_m");
  field(conf.odom_frame, "odom_frame");
  field(conf.robot_frame, "robot_frame");
  field(conf.num_poses_per_update, "num_poses_per_update");
  field(conf.max_input_queue_size, "max_input_queue_size");
  field(conf.make_pose_graph, "make_pose_graph");
  field(conf.semantic_measurement_probability, "semantic_measurement_probability");
  field(conf.copy_dense_representations, "copy_dense_representations");

  field(conf.tsdf, "tsdf");
  field(conf.mesh, "mesh");

  field<QuaternionConverter>(conf.body_R_camera, "body_R_camera");
  field(conf.body_t_camera, "body_t_camera");
}

}  // namespace hydra

namespace voxblox {

void declare_config(TsdfIntegratorBase::Config& conf) {
  using namespace config;
  name("TsdfIntegratorBase::Config");
  field(conf.default_truncation_distance, "default_truncation_distance");
  field(conf.max_weight, "max_weight");
  field(conf.voxel_carving_enabled, "voxel_carving_enabled");
  field(conf.min_ray_length_m, "min_ray_length_m");
  field(conf.max_ray_length_m, "max_ray_length_m");
  field(conf.use_const_weight, "use_const_weight");
  field(conf.allow_clear, "allow_clear");
  field(conf.use_weight_dropoff, "use_weight_dropoff");
  field(conf.use_sparsity_compensation_factor, "use_sparsity_compensation_factor");
  field<ThreadNumConversion>(conf.integrator_threads, "integrator_threads");
  field(conf.integration_order_mode, "integration_order_mode");
  field(conf.enable_anti_grazing, "enable_anti_grazing");
  field(conf.start_voxel_subsampling_factor, "start_voxel_subsampling_factor");
  field(conf.max_consecutive_ray_collisions, "max_consecutive_ray_collisions");
  field(conf.clear_checks_every_n_frames, "clear_checks_every_n_frames");
}

}  // namespace voxblox
