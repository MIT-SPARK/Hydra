#include "kimera_scene_graph/room_finder.h"

#include <glog/logging.h>

#include <voxblox/core/layer.h>
#include <voxblox/core/voxel.h>
#include <voxblox_ros/ptcloud_vis.h>

#include <pcl/common/transforms.h>

#include "kimera_scene_graph/common.h"
#include "kimera_scene_graph/object_finder.h"
#include "kimera_scene_graph/scene_graph.h"
#include "kimera_scene_graph/scene_graph_edge.h"
#include "kimera_scene_graph/scene_graph_layer.h"
#include "kimera_scene_graph/scene_graph_node.h"

namespace kimera {

RoomFinder::RoomFinder(const ros::NodeHandle& nh_private,
                       const std::string& world_frame,
                       const vxb::FloatingPoint& esdf_slice_level,
                       const float& skeleton_z_level,
                       const bool& visualize)
    : nh_private_(nh_private),
      pcl_pub_(),
      world_frame_(world_frame),
      esdf_slice_level_(esdf_slice_level),
      skeleton_z_level_(skeleton_z_level),
      visualize_(visualize) {
  pcl_pub_ = nh_private_.advertise<ColorPointCloud>("room_clusters", 1, true);
  esdf_truncated_pub_ =
      nh_private_.advertise<IntensityPointCloud>("esdf_truncated", 1, true);
}

/**
 * @brief RoomFinder::findRooms
 * Uses Semantic ESDF to find the rooms
 * @param cloud
 * @param room_centroids
 * @return
 */
IntensityPointCloud::Ptr RoomFinder::findRooms(
    const vxb::Layer<vxb::EsdfVoxel>& esdf_layer,
    SceneGraph* scene_graph) {
  CHECK_NOTNULL(scene_graph);

  // Create pcl from ESDF
  IntensityPointCloud::Ptr esdf_pcl(new IntensityPointCloud);
  vxb::createDistancePointcloudFromEsdfLayerSlice(
      esdf_layer, 2, esdf_slice_level_, &*esdf_pcl);
  if (esdf_pcl->empty()) {
    LOG(ERROR) << "Pointcloud of ESDF slice is empty! Modify the esdf slice "
                  "height to another value... \n Current value: "
               << std::to_string(esdf_slice_level_);
    return nullptr;
  }
  if (visualize_) publishTruncatedEsdf(esdf_pcl);

  // Downsample pcl
  IntensityPointCloud::Ptr downsampled_esdf_pcl(new IntensityPointCloud());
  downsampled_esdf_pcl = downsamplePcl<IntensityPoint>(esdf_pcl, 0.05f);

  // Pass through values below the given esdf truncation
  // We use -1.0 instead of 0.0 bcs we also want to filter out 0.
  downsampled_esdf_pcl = passThroughFilter1D<IntensityPoint>(
      downsampled_esdf_pcl, "intensity", -1.0, kEsdfTruncation, true);

  // Create region growing clustering algorithm
  ObjectFinder<IntensityPoint> object_finder(world_frame_,
                                             ObjectFinderType::kEuclidean);
  // Update the euclidean cluster estimator parameters
  EuclideanClusterEstimatorParams params;
  params.min_cluster_size_ = 300;
  object_finder.updateEuclideanClusterParams(params);

  // Get room clusters by clustering the downsampled and truncated ESDF
  Centroids room_centroids;
  std::vector<IntensityPointCloud::Ptr> room_pcls_noncolored;
  ObjectFinder<IntensityPoint>::BoundingBoxes bounding_boxes;
  ColorPointCloud::Ptr room_pcl_colored =
      object_finder.findObjects(downsampled_esdf_pcl,
                                &room_centroids,
                                &room_pcls_noncolored,
                                &bounding_boxes);
  CHECK_EQ(room_centroids.size(), room_pcls_noncolored.size());

  // Generate colored point cloud out of the noncolored rooms pcl.
  // Repaint room_pcls to uniform color for better visualization of edges.
  std::vector<ColorPointCloud::Ptr> room_pcls;
  room_pcls.resize(room_pcls_noncolored.size());
  static constexpr int32_t rgb =
      (static_cast<uint32_t>(0u) << 16 | static_cast<uint32_t>(255u) << 8 |
       static_cast<uint32_t>(0u));
  for (size_t i = 0u; i < room_pcls_noncolored.size(); ++i) {
    auto& room_pcl_ptr = room_pcls.at(i);
    room_pcl_ptr.reset(new ColorPointCloud);
    pcl::copyPointCloud(*room_pcls_noncolored.at(i), *room_pcl_ptr);
    for (auto& room_pcl_point : room_pcl_ptr->points) room_pcl_point.rgb = rgb;
  }

  room_pcl_colored->header.frame_id = world_frame_;
  for (auto& it : room_pcl_colored->points) {
    // Move all points to level 5
    it.z = 10;
  }
  pcl_pub_.publish(*room_pcl_colored);

  // Finally, update scene graphs with estimated rooms.
  updateSceneGraph(room_centroids, room_pcls, scene_graph);

  return downsampled_esdf_pcl;
}

void RoomFinder::updateSceneGraph(
    const Centroids& room_centroids,
    const std::vector<ColorPointCloud::Ptr>& room_pcls,
    SceneGraph* scene_graph) {
  CHECK_NOTNULL(scene_graph);
  CHECK_EQ(room_centroids.size(), room_pcls.size());
  // Update SceneGraph with rooms
  for (size_t idx = 0u; idx < room_centroids.size(); ++idx) {
    // Create SceneNode out of centroids
    SceneGraphNode room_instance;
    room_instance.attributes_.semantic_label_ = kRoomSemanticLabel;
    room_instance.attributes_.name_ = std::to_string(next_room_id_);
    const vxb::Color& room_color = getRoomColor(next_room_id_);
    room_instance.attributes_.color_ =
        NodeColor(room_color.r, room_color.g, room_color.b);
    room_instance.node_id_ = next_room_id_;
    room_instance.layer_id_ = LayerId::kRoomsLayerId;
    // TODO(Toni): project the centroid to the interior of the room.
    // otherwise the centroid might be outside of the room...
    // Ideally, this centroid should match the position of a 3D place
    pcl::PointXYZ centroid_point;
    room_centroids.at(idx).get(centroid_point);
    room_instance.attributes_.position_ =
        NodePosition(centroid_point.x, centroid_point.y, centroid_point.z);
    // This assumes the room_pcls vector has the same order as the centroids...
    // aka, the pointcloud and the centroid correspond to the same room.
    ColorPointCloud::Ptr room_pcl = room_pcls.at(idx);
    CHECK(room_pcl);
    // This pcl will be further used later to infer the places-room connections
    room_instance.attributes_.pcl_ = room_pcl;

    // Add the room to the database
    scene_graph->addSceneNode(room_instance);
    next_room_id_++;
  }
}

void RoomFinder::publishTruncatedEsdf(
    const IntensityPointCloud::Ptr& esdf_pcl) {
  // Publish truncated ESDF to see wall layout:
  IntensityPointCloud::Ptr esdf_truncated(new IntensityPointCloud);
  esdf_truncated = passThroughFilter1D<IntensityPoint>(
      esdf_pcl, "intensity", -1.0, kEsdfTruncation, false);
  esdf_truncated->header.frame_id = world_frame_;
  IntensityPointCloud::Ptr esdf_truncated_z_shift(new IntensityPointCloud);
  Eigen::Affine3f z_esdf = Eigen::Affine3f::Identity();
  z_esdf.translation() << 0.0, 0.0, skeleton_z_level_;
  pcl::transformPointCloud(*esdf_truncated, *esdf_truncated_z_shift, z_esdf);
  esdf_truncated_pub_.publish(*esdf_truncated_z_shift);
}

}  // namespace kimera
