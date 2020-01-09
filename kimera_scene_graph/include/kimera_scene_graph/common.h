#pragma once

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

// Kimera-X typedefs
typedef pcl::PointXYZ Point;
typedef pcl::PointXYZRGB ColorPoint;
typedef pcl::PointCloud<Point> PointCloud;
typedef pcl::PointCloud<ColorPoint> ColoredPointCloud;

typedef uint64_t Timestamp;
typedef std::string NodeId;
typedef pcl::PointXYZ NodePosition;
typedef pcl::PointNormal NodeOrientation;
