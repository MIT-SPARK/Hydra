#pragma once

#include <string>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>

// Region growing pcl
#include <pcl/features/normal_3d.h>
#include <pcl/filters/passthrough.h>
#include <pcl/search/search.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/visualization/cloud_viewer.h>

#include <kimera_semantics/common.h>

#include "kimera_scene_graph/common.h"
#include "kimera_scene_graph/object_finder.h"  // for params definitions

namespace kimera {

template <class T>
class WallFinder : protected ObjectFinder<T> {
 public:
  typedef pcl::PointCloud<T> PointCloudT;
  typedef std::vector<typename PointCloudT::Ptr> ObjectPointClouds;

  WallFinder(const std::string& world_frame)
      : ObjectFinder<T>(world_frame, ObjectFinderType::kRegionGrowing) {}

  ~WallFinder() = default;

  /**
   * @brief findWalls in a given pointcloud by using region growing techniques
   * We assume this pointcloud has already been Semantically segmented (aka
   * it only contains one semantic label)
   * @param pointcloud
   * @return colored pointcloud for cluster visualization...
   */
  ColorPointCloud::Ptr findWalls(const typename PointCloudT::Ptr& pointcloud,
                                 Centroids* centroids,
                                 ObjectPointClouds* object_pcls) {
    CHECK_NOTNULL(centroids);
    CHECK_NOTNULL(object_pcls);
    return this->findObjects(pointcloud, centroids, object_pcls);
  }

 private:
};

}  // namespace kimera
