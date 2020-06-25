#pragma once

#include <vector>

#include <pcl_ros/point_cloud.h>

#include <pcl/common/centroid.h>

#include <Eigen/Core>

#include <kimera_semantics/common.h>
#include <voxblox/core/color.h> // just for getroomcolor

namespace kimera {

// Kimera-X typedefs
typedef pcl::PointXYZ Point;
typedef pcl::PointXYZRGB ColorPoint;
typedef pcl::PointCloud<Point> PointCloud;
typedef pcl::PointCloud<ColorPoint> ColorPointCloud;
typedef pcl::PointXYZI IntensityPoint;
typedef pcl::PointCloud<IntensityPoint> IntensityPointCloud;

typedef uint64_t Timestamp;
typedef std::string NodeId;
typedef int64_t InstanceId;
typedef Point NodePosition;
typedef pcl::PointNormal NodeOrientation;
typedef Eigen::Vector3i NodeColor;

// TODO(Toni): create a structure to hold both Centroids and ObjectPointClouds
// associated to the centroids in the same object...
typedef pcl::CentroidPoint<Point> Centroid;
typedef std::vector<Centroid> Centroids;
typedef std::vector<ColorPointCloud::Ptr> ObjectPointClouds;

// Hardcoded for now
static constexpr int kRoomSemanticLabel = 21u;
static constexpr int kBuildingSemanticLabel = 22u;

static const Eigen::Vector3i kRoomColor(0u, 255u, 0u);
static const Eigen::Vector3i kBuildingColor(255u, 0u, 0u);

static constexpr float kEsdfTruncation = 0.3;

// inline Eigen::Vector3i getRandomColor() {
//  Eigen::Vector3i random_color;
// random_color.x =    static_cast<unsigned char>(rand() % 256),
// random_color.x =    static_cast<unsigned char>(rand() % 256),
// random_color.x =    static_cast<unsigned char>(rand() % 256));
// return random_color;
//}

enum class BoundingBoxType { kAABB = 0, kOBB = 1 };

template <class PointT>
struct BoundingBox {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  BoundingBoxType type_;
  PointT max_;
  PointT min_;
  // Position and rotation is only used for BB type of bb box.
  PointT position_;
  Eigen::Matrix3f orientation_matrix;
};

inline vxb::Color getRoomColor(int room_id) {
  return vxb::rainbowColorMap(static_cast<double>(room_id % 20) / 20.0);
}

}  // namespace kimera
