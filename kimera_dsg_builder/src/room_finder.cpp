#include "kimera_dsg_builder/room_finder.h"
#include "kimera_dsg_builder/pcl_types.h"

#include <glog/logging.h>

#include <kimera_dsg/node_attributes.h>
#include <voxblox/core/layer.h>
#include <voxblox/core/voxel.h>
#include <voxblox_ros/ptcloud_vis.h>

#include <pcl/common/transforms.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/surface/concave_hull.h>
#include <pcl_ros/point_cloud.h>

namespace kimera {

struct RoomPclClusters {
  using CloudCentroidPair = std::pair<ColorPointCloud::Ptr, Centroid>;
  IntensityPointCloud::Ptr colored_room_cloud;
  std::vector<CloudCentroidPair> room_info;
};

RoomFinder::RoomFinder(const ros::NodeHandle& nh_private,
                       const std::string& world_frame,
                       vxb::FloatingPoint esdf_slice_level,
                       bool visualize)
    : nh_private_(nh_private),
      pcl_pub_(),
      world_frame_(world_frame),
      esdf_slice_level_(esdf_slice_level),
      next_room_id_('R', 0),
      visualize_(visualize) {
  pcl_pub_ = nh_private_.advertise<ColorPointCloud>("room_clusters", 1, true);
  esdf_truncated_pub_ =
      nh_private_.advertise<IntensityPointCloud>("esdf_truncated", 1, true);
}

template <class T>
typename pcl::PointCloud<T>::Ptr passThroughFilter1D(
    const typename pcl::PointCloud<T>::Ptr& input,
    const std::string& dimension,
    float min,
    float max,
    bool negative_limits = false) {
  typename pcl::PointCloud<T>::Ptr cloud_filtered(new pcl::PointCloud<T>);
  // Create the filtering object
  pcl::PassThrough<T> pass;
  pass.setInputCloud(input);
  pass.setFilterFieldName(dimension);
  pass.setFilterLimits(min, max);
  pass.setFilterLimitsNegative(negative_limits);
  pass.filter(*cloud_filtered);
  return cloud_filtered;
}

template <class T>
typename pcl::PointCloud<T>::Ptr downsamplePcl(
    const typename pcl::PointCloud<T>::Ptr& input,
    float leaf_size = 0.05f,
    bool approx_downsampling = false) {
  typename pcl::PointCloud<T>::Ptr downsampled_pcl(new pcl::PointCloud<T>);
  if (approx_downsampling) {
    pcl::ApproximateVoxelGrid<T> downsampler;
    downsampler.setLeafSize(leaf_size, leaf_size, leaf_size);
    downsampler.setInputCloud(input);
    downsampler.filter(*downsampled_pcl);
  } else {
    pcl::VoxelGrid<T> downsampler;
    downsampler.setInputCloud(input);
    downsampler.setLeafSize(leaf_size, leaf_size, leaf_size);
    downsampler.filter(*downsampled_pcl);
  }
  return downsampled_pcl;
}

RoomPclClusters getRoomClusters(const IntensityPointCloud::Ptr& input,
                                double cluster_tolerance = 0.25,
                                size_t min_cluster_size = 300,
                                size_t max_cluster_size = 1000000) {
  pcl::search::KdTree<IntensityPoint>::Ptr tree(
      new pcl::search::KdTree<IntensityPoint>());
  tree->setInputCloud(input);

  pcl::EuclideanClusterExtraction<IntensityPoint> estimator;
  estimator.setSearchMethod(tree);
  estimator.setInputCloud(input);
  estimator.setClusterTolerance(cluster_tolerance);
  estimator.setMinClusterSize(min_cluster_size);
  estimator.setMaxClusterSize(max_cluster_size);

  std::vector<pcl::PointIndices> cluster_indices;
  estimator.extract(cluster_indices);

  RoomPclClusters clusters;
  clusters.room_info.reserve(cluster_indices.size());
  for (const auto& indices : cluster_indices) {
    IntensityPointCloud room_cloud;
    pcl::copyPointCloud(*input, indices.indices, room_cloud);
    ColorPointCloud::Ptr colored_room_cloud(new ColorPointCloud());
    pcl::copyPointCloud(room_cloud, *colored_room_cloud);

    // color point cloud and get centroid
    Centroid centroid;
    for (auto& point : colored_room_cloud->points) {
      centroid.add(pcl::PointXYZ(point.x, point.y, point.z));
    }

    clusters.room_info.push_back(std::make_pair(colored_room_cloud, centroid));
  }

  return clusters;
}

// TODO(nathan) this is hardcoded for 2 dimensions
RoomHull makeRoomHull(const ColorPointCloud::Ptr& room_pcl) {
  pcl::ConcaveHull<ColorPoint> concave_hull_adapter;
  static constexpr double kAlpha = 0.30;
  concave_hull_adapter.setAlpha(kAlpha);
  concave_hull_adapter.setKeepInformation(true);
  concave_hull_adapter.setDimension(2);
  concave_hull_adapter.setInputCloud(room_pcl);

  std::vector<pcl::Vertices> concave_polygon;
  ColorPointCloud::Ptr concave_hull_pcl(new ColorPointCloud());
  concave_hull_adapter.reconstruct(*concave_hull_pcl, concave_polygon);

  RoomHull hull;
  hull.hull_kdtree.reset(new pcl::KdTreeFLANN<ColorPoint>());
  hull.hull_kdtree->setInputCloud(concave_hull_pcl);

  hull.cropper.reset(new pcl::CropHull<ColorPoint>());
  hull.cropper->setHullIndices(concave_polygon);
  hull.cropper->setHullCloud(concave_hull_pcl);
  hull.cropper->setDim(2);
  hull.cropper->setCropOutside(true);

  return hull;
}

RoomHullMap updateSceneGraph(const RoomPclClusters& room_clusters,
                             SceneGraph* scene_graph,
                             NodeSymbol next_room_id) {
  CHECK_NOTNULL(scene_graph);

  RoomHullMap room_hull_map;
  for (const auto& cloud_centroid_pair : room_clusters.room_info) {
    RoomNodeAttributes::Ptr room_attrs = std::make_unique<RoomNodeAttributes>();

    room_attrs->semantic_label = kRoomSemanticLabel;
    room_attrs->name = next_room_id.getLabel();
    // TODO(nathan) fix this when we make this conversion more explicit
    const voxblox::Color& room_color = getRoomColor(next_room_id.categoryId());
    room_attrs->color << room_color.r, room_color.g, room_color.b;

    // TODO(Toni): project the centroid to the interior of the room.
    pcl::PointXYZ centroid;
    cloud_centroid_pair.second.get(centroid);
    room_attrs->position << centroid.x, centroid.y, centroid.z;

    scene_graph->emplaceNode(KimeraDsgLayers::ROOMS,
                             next_room_id,
                             std::move(room_attrs));

    room_hull_map[next_room_id] = makeRoomHull(cloud_centroid_pair.first);

    ++next_room_id;
  }

  return room_hull_map;
}

RoomHullMap RoomFinder::findRooms(const vxb::Layer<vxb::EsdfVoxel>& esdf,
                                  SceneGraph* scene_graph) {
  CHECK_NOTNULL(scene_graph);

  IntensityPointCloud::Ptr esdf_pcl(new IntensityPointCloud);
  vxb::createDistancePointcloudFromEsdfLayerSlice(
      esdf, 2, esdf_slice_level_, esdf_pcl.get());
  if (esdf_pcl->empty()) {
    LOG(ERROR) << "Pointcloud of ESDF slice is empty! Modify the esdf slice "
                  "height to another value... \n Current value: "
               << std::to_string(esdf_slice_level_);
    return RoomHullMap();
  }

  IntensityPointCloud::Ptr downsampled_esdf_pcl(new IntensityPointCloud());
  downsampled_esdf_pcl = downsamplePcl<IntensityPoint>(esdf_pcl, 0.05f);

  // Pass through values below the given esdf truncation
  // We use -1.0 instead of 0.0 bcs we also want to filter out 0.
  downsampled_esdf_pcl = passThroughFilter1D<IntensityPoint>(
      downsampled_esdf_pcl, "intensity", -1.0, kEsdfTruncation, true);

  RoomPclClusters clusters = getRoomClusters(downsampled_esdf_pcl);

  // TODO(nathan) was this needed? maybe cache
  // return downsampled_esdf_pcl;

  return updateSceneGraph(clusters, scene_graph, next_room_id_);
}

// TODO(nathan) consider moving this to visualization
/*void RoomFinder::publishTruncatedEsdf(*/
// const IntensityPointCloud::Ptr& esdf_pcl) {
//// Publish truncated ESDF to see wall layout:
// IntensityPointCloud::Ptr esdf_truncated(new IntensityPointCloud);
// esdf_truncated = passThroughFilter1D<IntensityPoint>(
// esdf_pcl, "intensity", -1.0, kEsdfTruncation, false);
// esdf_truncated->header.frame_id = world_frame_;
// IntensityPointCloud::Ptr esdf_truncated_z_shift(new IntensityPointCloud);
// Eigen::Affine3f z_esdf = Eigen::Affine3f::Identity();
//// TODO(nathan) this used to be configurable
// z_esdf.translation() << 0.0, 0.0, 30.0;
// pcl::transformPointCloud(*esdf_truncated, *esdf_truncated_z_shift, z_esdf);
// esdf_truncated_pub_.publish(*esdf_truncated_z_shift);
//}

}  // namespace kimera
