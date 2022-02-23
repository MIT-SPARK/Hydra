#pragma once
#include <pcl/common/centroid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace kimera {

using ColorPoint = pcl::PointXYZRGB;
using ColorPointCloud = pcl::PointCloud<ColorPoint>;

using IntensityPoint = pcl::PointXYZI;
using IntensityPointCloud = pcl::PointCloud<IntensityPoint>;

using Centroid = pcl::CentroidPoint<pcl::PointXYZ>;
using Centroids = std::vector<Centroid>;

}  // namespace kimera
