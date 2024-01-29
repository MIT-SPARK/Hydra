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
#include "hydra/frontend/frontend_config.h"

#include <config_utilities/config.h>

namespace kimera_pgmo {

void declare_config(kimera_pgmo::MeshFrontendConfig& conf) {
  using namespace config;
  name("MeshFrontendConfig");
  field(conf.time_horizon, "horizon");
  field(conf.b_track_mesh_graph_mapping, "track_mesh_graph_mapping");
  field(conf.full_compression_method, "full_compression_method");
  field(conf.graph_compression_method, "graph_compression_method");
  field(conf.d_graph_resolution, "d_graph_resolution");
  field(conf.mesh_resolution, "output_mesh_resolution");
}

}  // namespace kimera_pgmo

namespace hydra {

void declare_config(FrontendConfig& conf) {
  using namespace config;
  name("FrontendConfig");
  field(conf.min_object_vertices, "min_object_vertices");
  field(conf.prune_mesh_indices, "prune_mesh_indices");
  field(conf.lcd_use_bow_vectors, "lcd_use_bow_vectors");
  field(conf.pgmo_config, "pgmo");
  field(conf.object_config, "objects");
  field(conf.object_config.angle_step, "angle_step");
  field(conf.validate_vertices, "validate_vertices");
  field(conf.filter_places, "filter_places");
  field(conf.min_places_component_size, "min_places_component_size");

  field(conf.gvd, "gvd");
  conf.graph_extractor.setOptional();
  field(conf.graph_extractor, "graph_extractor");
}

}  // namespace hydra
