#pragma once

#include <glog/logging.h>

#include <ros/ros.h>

/// FOR SLIC
#include <opencv2/core/utility.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/ximgproc.hpp>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// Outlier Removal
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>

// for subscribers
#include <pcl_ros/point_cloud.h>

#include <pcl/features/integral_image_normal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/edge_aware_plane_comparator.h>
#include <pcl/segmentation/organized_connected_component_segmentation.h>
#include <pcl/segmentation/organized_multi_plane_segmentation.h>

#include <pcl/segmentation/edge_aware_plane_comparator.h>
#include <pcl/segmentation/euclidean_cluster_comparator.h>
#include <pcl/segmentation/euclidean_plane_coefficient_comparator.h>
#include <pcl/segmentation/plane_coefficient_comparator.h>
#include <pcl/segmentation/rgb_plane_coefficient_comparator.h>

// FOR SUPERVOXELS
#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/supervoxel_clustering.h>
#include <pcl/visualization/pcl_visualizer.h>

// FOR POISSON
#include <pcl/common/common.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/poisson.h>

// For Marching Cubes
#include <pcl/PCLPointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/surface/marching_cubes_hoppe.h>
#include <pcl/surface/marching_cubes_rbf.h>

// FOR FAST TRIANGULATION
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>
#include <pcl/surface/gp3.h>

// For SAC segmentation
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/concave_hull.h>

#include <voxblox/mesh/mesh.h>

#include <kimera_semantics/common.h>

#include "kimera_scene_graph/common.h"

namespace kimera {

struct MarchingCubesParams {
  float iso_level_ = 0.0f;
  float extend_percentage = 0.0f;
  int grid_res = 50;
  float off_surface_displacement = 0.01f;
  bool hoppe_or_rbf = 0;
};

struct OutlierFilterParams {
  float radius_search = 0.11;
  float min_neighbors_in_radius = 12;
};

template <class T>
typename pcl::PointCloud<T>::Ptr passThroughFilter1D(
    const typename pcl::PointCloud<T>::Ptr& input,
    const std::string& dimension,
    const float& min,
    const float& max,
    const bool& negative_limits = false) {
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

template<class T>
typename pcl::PointCloud<T>::Ptr downsamplePcl(
    const typename pcl::PointCloud<T>::Ptr& input,
    const float& leaf_size = 0.05f,
    const bool& approx_downsampling = false) {
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

class RoomFinder {
 public:
  RoomFinder(const ros::NodeHandle& nh_private, const std::string& world_frame);
  ~RoomFinder() = default;

  void findRooms(const vxb::Mesh& mesh, Centroids* room_centroids);

  /**
   * @brief findRooms
   * Uses Semantic ESDF
   * @param cloud
   * @param room_centroids
   * @param room_pcls
   * @return
   */
  IntensityPointCloud::Ptr findRooms(
      const IntensityPointCloud::Ptr& cloud,
      Centroids* room_centroids,
      std::vector<ColorPointCloud::Ptr>* room_pcls);

  /**
   * @brief findRooms
   * Uses dilation/erosion
   * @param cloud
   * @param room_centroids
   * @param room_pcls
   * @return
   */
  IntensityPointCloud::Ptr findRooms(
      const ColorPointCloud::Ptr& cloud,
      Centroids* room_centroids,
      std::vector<ColorPointCloud::Ptr>* room_pcls);

  inline void updateMarchingCubesParams(const MarchingCubesParams& mc_params) {
    mc_params_ = mc_params;
  }

  inline void updateOutlierFilterParams(const OutlierFilterParams& of_params) {
    of_params_ = of_params;
  }

 protected:
  void poissonReconstruction(const PointCloud::Ptr& cloud,
                             pcl::PolygonMesh* mesh);

  void marchingCubes(const PointCloud::Ptr& cloud, pcl::PolygonMesh* mesh);

  bool fastTriangulation(const PointCloud::Ptr& cloud, pcl::PolygonMesh* mesh);

  static void projectPointcloudToPlane(const ColorPointCloud::Ptr& cloud,
                                       ColorPointCloud* cloud_projected);

  // Types
  using PointT = pcl::PointXYZRGB;
  using PlanarRegions =
      std::vector<pcl::PlanarRegion<PointT>,
                  Eigen::aligned_allocator<pcl::PlanarRegion<PointT> > >;
  typedef pcl::PointCloud<PointT> PointCloudT;
  typedef pcl::PointNormal PointNT;
  typedef pcl::PointCloud<PointNT> PointNCloudT;
  typedef pcl::PointXYZL PointLT;
  typedef pcl::PointCloud<PointLT> PointLCloudT;

  void superVoxelize(const PointCloudT::Ptr& cloud,
                     PlanarRegions* planar_regions);
  void multiPlaneSegmenter(const ColorPointCloud::Ptr& cloud,
                           PlanarRegions* planar_regions);

  // Create a 2D mesh from 2D corners in an image
  // Returns the actual keypoints used to perform the triangulation.
  std::vector<cv::Vec6f> createMesh2dImpl(
      const cv::Size& img_size,
      std::vector<cv::Point2f>* keypoints_to_triangulate);

  struct PointIndexIdx {
    unsigned int grid_idx;
    unsigned int cloud_point_index;

    PointIndexIdx(unsigned int idx_, unsigned int cloud_point_index_)
        : grid_idx(idx_), cloud_point_index(cloud_point_index_) {}
    bool operator<(const PointIndexIdx& p) const {
      return (grid_idx < p.grid_idx);
    }
  };

  bool makeCvImgFromPcl(const IntensityPointCloud::Ptr& input,
                        cv::Mat* img,
                        IntensityPointCloud::Ptr* cloud,
                        const double& resolution = 0.1,
                        const bool& visualize = false);

  void triangulatePointcloud(const IntensityPointCloud::Ptr& morphology);

  cv::Mat getMorphologyRoom(const cv::Mat& img);

  void getSlicRoom(const cv::Mat& img);

 private:
  std::string world_frame_;
  ros::NodeHandle nh_private_;
  ros::Publisher pcl_pub_;
  MarchingCubesParams mc_params_;
  OutlierFilterParams of_params_;
};

}  // namespace kimera
