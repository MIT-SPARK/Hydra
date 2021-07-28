#pragma once
#include "kimera_topology/gvd_integrator.h"
#include <voxblox/integrator/esdf_integrator.h>

namespace kimera {
namespace topology {

#define COPY_FIELD(dest, src, field) dest.field = src.field;

inline GvdIntegratorConfig gvdConfigFromEsdfConfig(const voxblox::EsdfIntegrator::Config& esdf) {
  GvdIntegratorConfig config;
  COPY_FIELD(config, esdf, max_distance_m);
  COPY_FIELD(config, esdf, min_distance_m);
  COPY_FIELD(config, esdf, min_diff_m);
  COPY_FIELD(config, esdf, min_weight);
  COPY_FIELD(config, esdf, num_buckets);
  COPY_FIELD(config, esdf, multi_queue);
  return config;
}

#undef COPY_FIELD

}
}
