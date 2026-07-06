#include <gtest/gtest.h>
#include <spark_dsg/node_attributes.h>

#include "hydra/backend/deformation_interpolator.h"

namespace hydra {

using spark_dsg::KhronosObjectAttributes;

// Khronos objects never populate last_update_time_ns (it stays 0), so NodeCache
// must fall back to last_observed_ns. Otherwise the cached entry is stamped 0
// and the deformation solver matches it against the earliest control points
// instead of the ones near the object's observation time.
TEST(NodeCache, KhronosFallbackTimestampOnInsert) {
  constexpr uint64_t observed_ns = 1782847154557208075ULL;
  KhronosObjectAttributes attrs;
  attrs.last_update_time_ns = 0;
  attrs.last_observed_ns = {observed_ns};
  attrs.is_active = true;

  NodeCache cache;
  const NodeId node_id = 42;

  auto* entry = cache.add(node_id, attrs);
  ASSERT_NE(entry, nullptr);
  EXPECT_EQ(entry->timestamp, observed_ns);

  // re-adding an active node must keep the fallback stamp, not reset it to 0
  entry = cache.add(node_id, attrs);
  ASSERT_NE(entry, nullptr);
  EXPECT_EQ(entry->timestamp, observed_ns);
}

}  // namespace hydra
