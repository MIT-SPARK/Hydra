#pragma once

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

// Kimera-X typedefs
typedef pcl::PointXYZ Point;
typedef pcl::PointXYZRGB ColorPoint;
typedef pcl::PointCloud<Point> PointCloud;
typedef pcl::PointCloud<ColorPoint> ColoredPointCloud;
typedef pcl::PointXYZI IntensityPoint;
typedef pcl::PointCloud<IntensityPoint> IntensityPointCloud;

typedef uint64_t Timestamp;
typedef std::string NodeId;
typedef pcl::PointXYZ NodePosition;
typedef pcl::PointNormal NodeOrientation;

// TODO(Toni): create a structure to hold both Centroids and ObjectPointClouds
// associated to the centroids in the same object...
typedef pcl::CentroidPoint<Point> Centroid;
typedef std::vector<Centroid> Centroids;
typedef std::vector<ColoredPointCloud::Ptr> ObjectPointClouds;


// Hardcoded for now
static constexpr int kRoomSemanticLabel = 21u;

