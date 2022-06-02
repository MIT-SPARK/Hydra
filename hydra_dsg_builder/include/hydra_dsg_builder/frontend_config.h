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

namespace hydra {
namespace incremental {

struct DsgFrontendConfig {
  // TODO(nathan) consider unifying log path with backend
  bool should_log = true;
  std::string log_path;
  size_t mesh_queue_size = 10;
  size_t min_object_vertices = 20;
  bool prune_mesh_indices = false;
  std::string sensor_frame = "base_link";
  std::string mesh_ns = "";
};

template <typename Visitor>
void visit_config(const Visitor& v, DsgFrontendConfig& config) {
  // TODO(nathan) replace with single param (derive should_log from log_path)
  v.visit("should_log", config.should_log);
  v.visit("log_path", config.log_path);
  v.visit("mesh_queue_size", config.mesh_queue_size);
  v.visit("min_object_vertices", config.min_object_vertices);
  v.visit("prune_mesh_indices", config.prune_mesh_indices);
  v.visit("sensor_frame", config.sensor_frame);
  v.visit("mesh_ns", config.mesh_ns);
}

}  // namespace incremental
}  // namespace hydra

DECLARE_CONFIG_OSTREAM_OPERATOR(hydra::incremental, DsgFrontendConfig)
