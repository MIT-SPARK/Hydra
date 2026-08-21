#include <gtest/gtest.h>
#include <hydra/common/node_matchers.h>

using namespace spark_dsg;
namespace hydra {

TEST(NodeMatchers, BBoxIntersectionCorrect) {
  BBoxIntersectionMatcher matcher(BBoxIntersectionMatcher::Config{});

  KhronosObjectAttributes attrs1;
  attrs1.bounding_box =
      BoundingBox(Eigen::Vector3f::Constant(1.0), Eigen::Vector3f::Constant(1.5));

  KhronosObjectAttributes attrs2;
  attrs2.bounding_box =
      BoundingBox(Eigen::Vector3f::Constant(1.0), Eigen::Vector3f::Constant(3.5));

  EXPECT_FALSE(matcher.match(attrs1, attrs2));

  // make sure the two bounding boxes overlap
  attrs1.bounding_box =
      BoundingBox(Eigen::Vector3f::Constant(1.0), Eigen::Vector3f::Constant(3));

  EXPECT_TRUE(matcher.match(attrs1, attrs2));
}

}  // namespace hydra
