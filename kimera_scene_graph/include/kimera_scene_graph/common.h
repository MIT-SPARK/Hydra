#pragma once

#include <vector>

#include <pcl_ros/point_cloud.h>

#include <pcl/common/centroid.h>

#include <Eigen/Core>

#include <kimera_semantics/common.h>
#include <voxblox/core/color.h>  // just for getroomcolor

namespace kimera {

// Kimera-X typedefs
typedef pcl::PointXYZ Point;
typedef pcl::PointXYZRGB ColorPoint;
typedef pcl::PointCloud<Point> PointCloud;
typedef pcl::PointCloud<ColorPoint> ColorPointCloud;
typedef pcl::PointXYZI IntensityPoint;
typedef pcl::PointCloud<IntensityPoint> IntensityPointCloud;

typedef uint64_t Timestamp;
//! Specifies the id of a node in a layer
typedef int64_t NodeId;
//! Specifies the id of a layer in the graph
typedef int64_t EdgeId;
typedef std::string NodeName;

// TODO(Toni): create a structure to hold both Centroids and ObjectPointClouds
// associated to the centroids in the same object...
typedef pcl::CentroidPoint<Point> Centroid;
typedef std::vector<Centroid> Centroids;
typedef std::vector<ColorPointCloud::Ptr> ObjectPointClouds;

// Hardcoded for now
static constexpr int kRoomSemanticLabel = 21u;
static constexpr int kBuildingSemanticLabel = 22u;

/**
 * @brief The LayerId enum The numbering is important, as it determines the
 * parentesco: a higher number corresponds to parents.
 */
enum class LayerId {
  kInvalidLayerId = 0,
  kObjectsLayerId = 1,
  kAgentsLayerId = 2,
  kPlacesLayerId = 3,
  kRoomsLayerId = 4,
  kBuildingsLayerId = 5
};

inline std::string getStringFromLayerId(const LayerId& layer_id) {
  switch (layer_id) {
    case LayerId::kBuildingsLayerId:
      return "B";
    case LayerId::kRoomsLayerId:
      return "R";
    case LayerId::kPlacesLayerId:
      return "P";
    case LayerId::kObjectsLayerId:
      return "O";
    default:
      return "NA";
  }
}

static constexpr float kEsdfTruncation = 0.3;

enum class BoundingBoxType { kAABB = 0, kOBB = 1 };

template <class PointT>
struct BoundingBox {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  BoundingBoxType type_ = BoundingBoxType::kAABB;
  PointT max_ = PointT();
  PointT min_ = PointT();
  // Position and rotation is only used for BB type of bb box.
  PointT position_ = PointT();
  Eigen::Matrix3f orientation_matrix = Eigen::Matrix3f();
};

inline vxb::Color getRoomColor(const NodeId& room_id) {
  return vxb::rainbowColorMap(static_cast<double>(room_id % 20) / 20.0);
}

// Add way of printing strongly typed enums (enum class).
template <typename E>
constexpr typename std::underlying_type<E>::type to_underlying(E e) noexcept {
  return static_cast<typename std::underlying_type<E>::type>(e);
}

}  // namespace kimera
