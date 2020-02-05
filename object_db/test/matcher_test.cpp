#include <gtest/gtest.h>
#include <object_db/matcher.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Core>

/**
 * Test brute force matching
 */
TEST(MatcherTest, BruteForce) {
  object_registration::KeypointsMatcher matcher;

  // Prepare two fake point clouds
  size_t src_size = 5;
  size_t dst_size = 4;
  pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);
  for (size_t i = 0; i < src_size; ++i) {
    src_cloud->push_back({i, i, i});
  }
  pcl::PointCloud<pcl::PointXYZ>::Ptr dst_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);
  for (size_t i = 0; i < dst_size; ++i) {
    dst_cloud->push_back({static_cast<float>(i + 5), static_cast<float>(i + 5), static_cast<float>(i + 5)});
  }

  Eigen::Matrix<double, 3, Eigen::Dynamic> src(3, src_size * dst_size);
  Eigen::Matrix<double, 3, Eigen::Dynamic> dst(3, src_size * dst_size);

  matcher.generateCorrespondences(src_cloud, dst_cloud, &src, &dst);

  ASSERT_EQ(src.cols(), src_size*dst_size);
  ASSERT_EQ(dst.cols(), src_size*dst_size);
  std::cout << "SRC correspondences:" << std::endl;
  std::cout << src << std::endl;
  std::cout << "DST correspondences:" << std::endl;
  std::cout << dst << std::endl;
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}