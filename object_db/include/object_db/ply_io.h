#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace object_registration {

// Internal datatypes for storing ply vertices
struct float3 {
  float x, y, z;
};
struct double3 {
  double x, y, z;
};

/**
 * @brief A class for reading PLY files.
 */
class PLYReader {
 public:
  PLYReader() = default;
  ~PLYReader() = default;

  /**
   * @brief A wrapper function for reading ply files into PointCloud
   */
  int read(const std::string& file_name,
           pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
};

}  // namespace object_registration
