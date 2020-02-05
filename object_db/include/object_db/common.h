#pragma once

#include <pcl/io/ply_io.h>

namespace object_registration {

/**
 * Struct to hold an object
 */
struct Object {
  std::string semantic_label;

  // Paths to various object files
  std::string mtl_file_path;
  std::string obj_file_path;
  std::string ply_file_path;

  // Pointer to a pcl point cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud;
};

}

