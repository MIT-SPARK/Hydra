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
#include <KimeraRPGO/SolverParams.h>
#include <kimera_pgmo/KimeraPgmoInterface.h>

#include "hydra/common/dsg_types.h"
#include "hydra/config/config.h"
#include "hydra/config/eigen_config_types.h"
#include "hydra/rooms/room_finder_config.h"

DECLARE_CONFIG_ENUM(KimeraRPGO,
                    Verbosity,
                    {Verbosity::UPDATE, "UPDATE"},
                    {Verbosity::QUIET, "QUIET"},
                    {Verbosity::VERBOSE, "VERBOSE"})

DECLARE_CONFIG_ENUM(kimera_pgmo,
                    RunMode,
                    {RunMode::FULL, "FULL"},
                    {RunMode::MESH, "MESH"},
                    {RunMode::DPGMO, "DPGMO"})

DECLARE_CONFIG_ENUM(KimeraRPGO, Solver, {Solver::LM, "LM"}, {Solver::GN, "GN"})

namespace kimera_pgmo {

template <typename Visitor>
void visit_config(const Visitor& v, KimeraPgmoConfig& config) {
  if (!config_parser::is_parser<Visitor>()) {
    v.visit("run_mode", config.mode);
  } else {
    int mode = 0;
    v.visit("run_mode", mode);
    config.mode = static_cast<RunMode>(mode);
  }

  v.visit("embed_trajectory_delta_t", config.embed_delta_t);
  v.visit("num_interp_pts", config.num_interp_pts);
  v.visit("interp_horizon", config.interp_horizon);
  v.visit("add_initial_prior", config.b_add_initial_prior);
  if (!config_parser::is_parser<Visitor>()) {
    // this gets populated elsewhere
    v.visit("output_prefix", config.log_path);
  }

  auto covar_handle = v["covariance"];
  covar_handle.visit("odom", config.odom_variance);
  covar_handle.visit("loop_close", config.lc_variance);
  covar_handle.visit("prior", config.prior_variance);
  covar_handle.visit("mesh_mesh", config.mesh_edge_variance);
  covar_handle.visit("pose_mesh", config.pose_mesh_variance);

  auto rpgo_handle = v["rpgo"];
  rpgo_handle.visit("odom_trans_threshold", config.odom_trans_threshold);
  rpgo_handle.visit("odom_rot_threshold", config.odom_rot_threshold);
  rpgo_handle.visit("pcm_trans_threshold", config.pcm_trans_threshold);
  rpgo_handle.visit("pcm_rot_threshold", config.pcm_rot_threshold);
  rpgo_handle.visit("gnc_alpha", config.gnc_alpha);
  rpgo_handle.visit("gnc_max_iterations", config.gnc_max_it);
  rpgo_handle.visit("gnc_mu_step", config.gnc_mu_step);
  rpgo_handle.visit("gnc_cost_tolerance", config.gnc_cost_tol);
  rpgo_handle.visit("gnc_weight_tolerance", config.gnc_weight_tol);
  rpgo_handle.visit("gnc_fix_prev_inliers", config.gnc_fix_prev_inliers);
  rpgo_handle.visit("lm_diagonal_damping", config.lm_diagonal_damping);

  if (!config_parser::is_parser<Visitor>()) {
    v.visit("valid", config.valid);
  } else {
    // TODO(nathan) might want to track actual validity
    config.valid = true;
  }
}

}  // namespace kimera_pgmo

namespace hydra {

struct BackendConfig {
  bool should_log = true;
  std::string log_path;

  bool visualize_place_factors = true;
  SemanticNodeAttributes::ColorVector building_color{169, 8, 194};  // purple
  SemanticNodeAttributes::Label building_semantic_label = 22u;

  bool enable_rooms = true;
  RoomFinderConfig room_finder;

  struct PgmoConfig {
    bool should_log = true;
    std::string log_path;
    // covariance
    double place_mesh_variance;
    double place_edge_variance;
    double place_merge_variance;
    double object_merge_variance;
    double sg_loop_closure_variance;
    // rpgo
    bool gnc_fix_prev_inliers = true;
    KimeraRPGO::Verbosity rpgo_verbosity = KimeraRPGO::Verbosity::UPDATE;
    KimeraRPGO::Solver rpgo_solver = KimeraRPGO::Solver::LM;
  } pgmo;

  // dsg
  bool add_places_to_deformation_graph = true;
  bool optimize_on_lc = true;
  bool enable_node_merging = true;
  bool use_mesh_subscribers = false;
  std::map<LayerId, bool> merge_update_map{{DsgLayers::OBJECTS, false},
                                           {DsgLayers::PLACES, true},
                                           {DsgLayers::ROOMS, false},
                                           {DsgLayers::BUILDINGS, false}};
  bool merge_update_dynamic = true;
  float angle_step = 10.0f;
  double places_merge_pos_threshold_m = 0.4;
  double places_merge_distance_tolerance_m = 0.3;
  bool enable_merge_undos = false;
  bool use_active_flag_for_updates = true;
  size_t num_neighbors_to_find_for_merge = 1;
  std::string zmq_send_url = "tcp://127.0.0.1:8001";
  std::string zmq_recv_url = "tcp://127.0.0.1:8002";
  bool use_zmq_interface = false;
  size_t zmq_num_threads = 2;
  size_t zmq_poll_time_ms = 10;
};

struct EnableMapConverter {
  using TargetMap = std::map<std::string, bool>;
  using SourceMap = std::map<LayerId, bool>;

  EnableMapConverter() = default;

  inline TargetMap from(const SourceMap& other) const {
    TargetMap to_return;
    for (const auto& kv_pair : other) {
      to_return[DsgLayers::LayerIdToString(kv_pair.first)] = kv_pair.second;
    }

    return to_return;
  }

  inline SourceMap to(const TargetMap& other) const {
    SourceMap to_return;
    for (const auto& kv_pair : other) {
      to_return[DsgLayers::StringToLayerId(kv_pair.first)] = kv_pair.second;
    }

    return to_return;
  }
};

template <typename Visitor>
void visit_config(const Visitor& v, BackendConfig& config) {
  // TODO(nathan) replace with single param (derive should_log from log_path)
  v.visit("should_log", config.should_log);
  if (config.should_log) {
    v.visit("log_path", config.log_path);
  }
  v.visit("visualize_place_factors", config.visualize_place_factors);
  v.visit("building_color", config.building_color);
  v.visit("building_semantic_label", config.building_semantic_label);
  v.visit("enable_rooms", config.enable_rooms);
  v.visit("angle_step", config.angle_step);
  v.visit("room_finder", config.room_finder);

  v.visit("pgmo", config.pgmo);

  auto dsg_handle = v["dsg"];
  dsg_handle.visit("add_places_to_deformation_graph",
                   config.add_places_to_deformation_graph);
  dsg_handle.visit("optimize_on_lc", config.optimize_on_lc);
  dsg_handle.visit("enable_node_merging", config.enable_node_merging);
  dsg_handle.visit("merge_update_map", config.merge_update_map, EnableMapConverter());
  dsg_handle.visit("merge_update_dynamic", config.merge_update_dynamic);
  dsg_handle.visit("places_merge_pos_threshold_m", config.places_merge_pos_threshold_m);
  dsg_handle.visit("places_merge_distance_tolerance_m",
                   config.places_merge_distance_tolerance_m);
  dsg_handle.visit("use_mesh_subscribers", config.use_mesh_subscribers);
  dsg_handle.visit("enable_merge_undos", config.enable_merge_undos);
  dsg_handle.visit("use_active_flag_for_updates", config.use_active_flag_for_updates);
  dsg_handle.visit("num_neighbors_to_find_for_merge",
                   config.num_neighbors_to_find_for_merge);
  dsg_handle.visit("zmq_send_url", config.zmq_send_url);
  dsg_handle.visit("zmq_recv_url", config.zmq_recv_url);
  dsg_handle.visit("use_zmq_interface", config.use_zmq_interface);
  dsg_handle.visit("zmq_num_threads", config.zmq_num_threads);
  dsg_handle.visit("zmq_poll_time_ms", config.zmq_poll_time_ms);
}

template <typename Visitor>
void visit_config(const Visitor& v, BackendConfig::PgmoConfig& config) {
  // TODO(nathan) replace with single param (derive should_log from log_path)
  v.visit("should_log", config.should_log);
  if (config.should_log) {
    v.visit("log_path", config.log_path);
  }
  auto covar_handle = v["covariance"];
  covar_handle.visit("place_mesh", config.place_mesh_variance);
  covar_handle.visit("place_edge", config.place_edge_variance);
  covar_handle.visit("place_merge", config.place_merge_variance);
  covar_handle.visit("object_merge", config.object_merge_variance);
  covar_handle.visit("sg_loop_close", config.sg_loop_closure_variance);
  auto rpgo_handle = v["rpgo"];
  rpgo_handle.visit("gnc_fix_prev_inliers", config.gnc_fix_prev_inliers);
  rpgo_handle.visit("verbosity", config.rpgo_verbosity);
  rpgo_handle.visit("solver", config.rpgo_solver);
}

}  // namespace hydra

DECLARE_CONFIG_OSTREAM_OPERATOR(hydra, BackendConfig)
DECLARE_CONFIG_OSTREAM_OPERATOR(kimera_pgmo, KimeraPgmoConfig)
