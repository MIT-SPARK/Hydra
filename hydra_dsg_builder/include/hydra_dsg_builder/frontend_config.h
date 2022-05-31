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
