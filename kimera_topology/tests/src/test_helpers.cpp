#include <kimera_topology_test/test_helpers.h>

namespace kimera {
namespace topology {
namespace test_helpers {

#define COPY_FIELD(dest, src, field) dest.field = src.field;

GvdIntegratorConfig gvdConfigFromEsdfConfig(const EsdfIntegrator::Config& esdf) {
  GvdIntegratorConfig config;
  COPY_FIELD(config, esdf, full_euclidean_distance);
  COPY_FIELD(config, esdf, max_distance_m);
  COPY_FIELD(config, esdf, min_distance_m);
  COPY_FIELD(config, esdf, min_diff_m);
  COPY_FIELD(config, esdf, min_weight);
  COPY_FIELD(config, esdf, num_buckets);
  COPY_FIELD(config, esdf, multi_queue);
  return config;
}

#undef COPY_FIELD

}  // namespace test_helpers
}  // namespace topology
}  // namespace kimera
