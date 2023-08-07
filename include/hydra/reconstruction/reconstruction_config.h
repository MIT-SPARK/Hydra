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
#include <config_utilities/virtual_config.h>
#include <voxblox/integrator/tsdf_integrator.h>

#include "hydra/places/graph_extractor_interface.h"
#include "hydra/places/gvd_integrator_config.h"
#include "hydra/reconstruction/mesh_integrator_config.h"

namespace hydra {

struct ReconstructionConfig {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ReconstructionConfig()
      : body_R_camera(Eigen::Quaterniond::Identity()),
        body_t_camera(Eigen::Vector3d::Zero()) {}

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
  float semantic_measurement_probability = 0.9;

  places::GvdIntegratorConfig gvd;
  config::VirtualConfig<places::GraphExtractorInterface> graph_extractor;
  voxblox::TsdfIntegratorBase::Config tsdf;
  MeshIntegratorConfig mesh;
  Eigen::Quaterniond body_R_camera;
  Eigen::Vector3d body_t_camera;
};

void declare_config(ReconstructionConfig& conf);

}  // namespace hydra

namespace voxblox {

void declare_config(TsdfIntegratorBase::Config& conf);

}  // namespace voxblox
