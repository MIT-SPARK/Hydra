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
#include "hydra/backend/backend_config.h"

#include <config_utilities/config.h>
#include <config_utilities/types/eigen_matrix.h>
#include <config_utilities/types/enum.h>

#include "hydra/common/config_utilities.h"

namespace kimera_pgmo {

void declare_config(KimeraPgmoConfig& config) {
  using namespace config;
  name("KimeraPgmoConfig");
  // TODO(nathan) handle this better
  enum_field(config.mode,
             "run_mode",
             {{RunMode::FULL, "0"}, {RunMode::MESH, "1"}, {RunMode::DPGMO, "2"}});
  field(config.embed_delta_t, "embed_trajectory_delta_t");
  field(config.num_interp_pts, "num_interp_pts");
  field(config.interp_horizon, "interp_horizon");
  field(config.b_add_initial_prior, "add_initial_prior");
  field(config.log_path, "output_prefix");

  {  // config namespace covariance
    NameSpace ns("covariance");
    field(config.odom_variance, "odom");
    field(config.lc_variance, "loop_close");
    field(config.prior_variance, "prior");
    field(config.mesh_edge_variance, "mesh_mesh");
    field(config.pose_mesh_variance, "pose_mesh");
  }  // config namespace covariance

  {  // config namespace rpgo
    NameSpace ns("rpgo");
    field(config.odom_trans_threshold, "odom_trans_threshold");
    field(config.odom_rot_threshold, "odom_rot_threshold");
    field(config.pcm_trans_threshold, "pcm_trans_threshold");
    field(config.pcm_rot_threshold, "pcm_rot_threshold");
    field(config.gnc_alpha, "gnc_alpha");
    field(config.gnc_max_it, "gnc_max_iterations");
    field(config.gnc_mu_step, "gnc_mu_step");
    field(config.gnc_cost_tol, "gnc_cost_tolerance");
    field(config.gnc_weight_tol, "gnc_weight_tolerance");
    field(config.gnc_fix_prev_inliers, "gnc_fix_prev_inliers");
    field(config.lm_diagonal_damping, "lm_diagonal_damping");
  }  // config namespace rpgo

  // TODO(nathan) handle valid flag
}

}  // namespace kimera_pgmo

namespace hydra {

void declare_config(HydraPgmoConfig& config) {
  using namespace config;
  name("HydraPgmoConfig");
  base<kimera_pgmo::KimeraPgmoConfig>(config);
  {  // config namespace covariance
    NameSpace ns("covariance");
    field(config.place_mesh_variance, "place_mesh");
    field(config.place_edge_variance, "place_edge");
    field(config.place_merge_variance, "place_merge");
    field(config.object_merge_variance, "object_merge");
    field(config.sg_loop_closure_variance, "sg_loop_close");
  }  // config namespace covariance

  {  // config namespace rpgo
    NameSpace ns("rpgo");
    field(config.gnc_fix_prev_inliers, "gnc_fix_prev_inliers");
    enum_field(config.rpgo_verbosity,
               "verbosity",
               {{KimeraRPGO::Verbosity::UPDATE, "UPDATE"},
                {KimeraRPGO::Verbosity::QUIET, "QUIET"},
                {KimeraRPGO::Verbosity::VERBOSE, "VERBOSE"}});
    enum_field(config.rpgo_solver,
               "solver",
               {{KimeraRPGO::Solver::LM, "LM"}, {KimeraRPGO::Solver::GN, "GN"}});
  }  // config namespace rpgo
}

void declare_config(BackendConfig& config) {
  using namespace config;
  name("BackendConfig");
  field(config.angle_step, "angle_step");
  field(config.visualize_place_factors, "visualize_place_factors");
  field(config.building_color, "building_color");
  field(config.building_semantic_label, "building_semantic_label");
  field(config.enable_rooms, "enable_rooms");
  field(config.room_finder, "room_finder");
  field(config.pgmo, "pgmo");

  enter_namespace("dsg");
  field(config.add_places_to_deformation_graph, "add_places_to_deformation_graph");
  field(config.optimize_on_lc, "optimize_on_lc");
  field(config.enable_node_merging, "enable_node_merging");
  field<LayerMapConversion<bool>>(config.merge_update_map, "merge_update_map");
  field(config.merge_update_dynamic, "merge_update_dynamic");
  field(config.places_merge_pos_threshold_m, "places_merge_pos_threshold_m");
  field(config.places_merge_distance_tolerance_m, "places_merge_distance_tolerance_m");
  field(config.use_mesh_subscribers, "use_mesh_subscribers");
  field(config.enable_merge_undos, "enable_merge_undos");
  field(config.use_active_flag_for_updates, "use_active_flag_for_updates");
  field(config.num_neighbors_to_find_for_merge, "num_neighbors_to_find_for_merge");
  field(config.zmq_send_url, "zmq_send_url");
  field(config.zmq_recv_url, "zmq_recv_url");
  field(config.use_zmq_interface, "use_zmq_interface");
  field(config.zmq_num_threads, "zmq_num_threads");
  field(config.zmq_poll_time_ms, "zmq_poll_time_ms");
}

}  // namespace hydra
