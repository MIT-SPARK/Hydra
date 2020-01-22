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
#include <pcl/filters/morphological_filter.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>

#include <pcl/2d/morphology.h>

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
#include <pcl/point_cloud.h>
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

class RoomFinder {
 private:
 public:
  RoomFinder(const ros::NodeHandle& nh_private, const std::string& world_frame)
      : pcl_pub_(), nh_private_(nh_private), world_frame_(world_frame) {
    pcl_pub_ =
        nh_private_.advertise<ColoredPointCloud>("room_clusters", 1, true);
  };
  ~RoomFinder() = default;

  void findRooms(const vxb::Mesh& mesh, Centroids* room_centroids);
  IntensityPointCloud::Ptr findRooms(
      const ColoredPointCloud::Ptr& cloud,
      Centroids* room_centroids,
      std::vector<IntensityPointCloud::Ptr>* room_pcls);

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

  static void projectPointcloudToPlane(const ColoredPointCloud::Ptr& cloud,
                                       ColoredPointCloud* cloud_projected);

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
  void multiPlaneSegmenter(const ColoredPointCloud::Ptr& cloud,
                           PlanarRegions* planar_regions);

  template <class T>
  typename pcl::PointCloud<T>::Ptr passThroughFilter1D(
      const typename pcl::PointCloud<T>::Ptr& input,
      const std::string& dimension,
      const float& min,
      const float& max) {
    typename pcl::PointCloud<T>::Ptr cloud_filtered(new pcl::PointCloud<T>);
    // Create the filtering object
    pcl::PassThrough<T> pass;
    pass.setInputCloud(input);
    pass.setFilterFieldName(dimension);
    pass.setFilterLimits(min, max);
    // pass.setFilterLimitsNegative (true);
    pass.filter(*cloud_filtered);
    return cloud_filtered;
  }

  /* --------------------------------------------------------------------------
   */
  // Create a 2D mesh from 2D corners in an image
  // Returns the actual keypoints used to perform the triangulation.
  std::vector<cv::Vec6f> createMesh2dImpl(
      const cv::Size& img_size,
      std::vector<cv::Point2f>* keypoints_to_triangulate) {
    CHECK_NOTNULL(keypoints_to_triangulate);
    // Nothing to triangulate.
    if (keypoints_to_triangulate->size() == 0) return std::vector<cv::Vec6f>();

    // Rectangle to be used with Subdiv2D.
    cv::Rect2f rect(0, 0, img_size.width, img_size.height);
    // subdiv has the delaunay triangulation function
    cv::Subdiv2D subdiv(rect);

    // TODO Luca: there are kpts outside image, probably from tracker. This
    // check should be in the tracker.
    // -> Make sure we only pass keypoints inside the image!
    for (auto it = keypoints_to_triangulate->begin();
         it != keypoints_to_triangulate->end();) {
      if (!rect.contains(*it)) {
        VLOG(1) << "createMesh2D - error, keypoint out of image frame.";
        it = keypoints_to_triangulate->erase(it);
        // Go backwards, otherwise it++ will jump one keypoint...
      } else {
        it++;
      }
    }

    // Perform triangulation.
    try {
      subdiv.insert(*keypoints_to_triangulate);
    } catch (...) {
      LOG(FATAL) << "CreateMesh2D: subdiv.insert error (2).\n Keypoints to "
                    "triangulate: "
                 << keypoints_to_triangulate->size();
    }

    // getTriangleList returns some spurious triangle with vertices outside
    // image
    // TODO I think that the spurious triangles are due to ourselves sending
    // keypoints out of the image... Compute actual triangulation.
    std::vector<cv::Vec6f> triangulation2D;
    subdiv.getTriangleList(triangulation2D);

    // Retrieve "good triangles" (all vertices are inside image).
    for (auto it = triangulation2D.begin(); it != triangulation2D.end();) {
      if (!rect.contains(cv::Point2f((*it)[0], (*it)[1])) ||
          !rect.contains(cv::Point2f((*it)[2], (*it)[3])) ||
          !rect.contains(cv::Point2f((*it)[4], (*it)[5]))) {
        it = triangulation2D.erase(it);
        // Go backwards, otherwise it++ will jump one keypoint...
      } else {
        it++;
      }
    }
    return triangulation2D;
  }

  struct PointIndexIdx {
    unsigned int grid_idx;
    unsigned int cloud_point_index;

    PointIndexIdx(unsigned int idx_, unsigned int cloud_point_index_)
        : grid_idx(idx_), cloud_point_index(cloud_point_index_) {}
    bool operator<(const PointIndexIdx& p) const {
      return (grid_idx < p.grid_idx);
    }
  };

  IntensityPointCloud::Ptr makeCvImgFromPcl(const cv::Mat& img,
                                            const double& resolution = 0.1,
                                            const bool& visualize = false) {}

  bool makeCvImgFromPcl(const IntensityPointCloud::Ptr& input,
                        cv::Mat* img,
                        IntensityPointCloud::Ptr* cloud,
                        const double& resolution = 0.1,
                        const bool& visualize = false) {
    CHECK_NOTNULL(img);
    CHECK_NOTNULL(cloud);
    // Get the minimum and maximum dimensions
    Eigen::Vector4f min_p, max_p;
    pcl::getMinMax3D<IntensityPoint>(*input, min_p, max_p);

    // Check that the resolution is not too small, given the size of the data
    double inverse_resolution = 1 / resolution;
    std::int64_t dx =
        static_cast<std::int64_t>((max_p[0] - min_p[0]) * inverse_resolution) +
        1;
    std::int64_t dy =
        static_cast<std::int64_t>((max_p[1] - min_p[1]) * inverse_resolution) +
        1;

    if ((dx * dy) >
        static_cast<std::int64_t>(std::numeric_limits<std::int32_t>::max())) {
      LOG(ERROR) << "Leaf size is too small for the input dataset. Integer "
                    "indices would overflow.";
      return false;
    }

    Eigen::Vector4i min_b, max_b, div_b;

    // Compute the minimum and maximum bounding box values
    min_b[0] = static_cast<int>(std::floor(min_p[0] * inverse_resolution));
    max_b[0] = static_cast<int>(std::floor(max_p[0] * inverse_resolution));
    min_b[1] = static_cast<int>(std::floor(min_p[1] * inverse_resolution));
    max_b[1] = static_cast<int>(std::floor(max_p[1] * inverse_resolution));

    // Compute the number of divisions needed along all axis
    div_b = max_b - min_b + Eigen::Vector4i::Ones();
    div_b[3] = 0;

    std::vector<PointIndexIdx> index_vector;
    index_vector.reserve(input->size());

    // This is our output img
    // width, height
    cv::Size size(div_b[1], div_b[0]);
    *img = cv::Mat::zeros(size, CV_8U);

    // This is our output pointcloud
    (*cloud)->width = size.width;
    (*cloud)->height = size.height;
    (*cloud)->resize(size.area());

    // First pass: go over all points and insert them into the index_vector
    // vector with calculated idx. Points with the same idx value will
    // contribute to the same point of resulting CloudPoint
    size_t ptcloud_idx = 0u;
    for (auto it = input->begin(); it != input->end(); ++it) {
      if (!input->is_dense) {
        // Check if the point is invalid
        if (!std::isfinite(it->x) || !std::isfinite(it->y) ||
            !std::isfinite(it->z)) {
          continue;
        }
      }

      // Compute the grid cell index
      int row = static_cast<int>(std::floor(it->x * inverse_resolution) -
                                 static_cast<float>(min_b[0]));
      int col = static_cast<int>(std::floor(it->y * inverse_resolution) -
                                 static_cast<float>(min_b[1]));

      // Fill image
      // at(row, column)
      img->at<std::int8_t>(row, col) = 255u;

      // Fill pointcloud
      IntensityPoint point;
      // point.x = row;
      // point.y = col;
      // point.z = 0.0;
      // We are just taking the values of the last one voting for this cell
      point.x = it->x;
      point.y = it->y;
      point.z = it->z;
      point.intensity = 1.0;
      // at(column, row)
      CHECK_LT(row, size.height);
      CHECK_LT(col, size.width);
      (*cloud)->at(col, row) = point;

      ++ptcloud_idx;
    }
    CHECK_EQ((*cloud)->size(), size.area());

    if (visualize) {
      cv::imshow("PCL image", *img);
      *img = getMorphologyRoom(getMorphologyRoom(*img));
      // getSlicRoom(*img);
      // cv::waitKey(0);
    }

    // Restore pointcloud of morphology room
    CHECK_EQ(img->rows, size.height);
    CHECK_EQ(img->cols, size.width);
    for (size_t row = 0u; row < img->rows; row++) {
      for (size_t col = 0u; col < img->cols; col++) {
        CHECK_LT(row, size.height);
        CHECK_LT(col, size.width);
        if (img->at<std::int8_t>(row, col) == 0u) {
          // Switch off this point, since it has been eroded in the original
          // image.
          (*cloud)->at(col, row).intensity = 0.0;
        }
      }
    }

    return true;
  }

  void triangulatePointcloud(const IntensityPointCloud::Ptr& morphology) {
    std::vector<cv::Point2f> keypoints_to_triangulate;
    keypoints_to_triangulate.resize(morphology->size());
    // Fill keypoints to triangulate
    size_t i = 0u;
    float x_max = std::numeric_limits<float>::min();
    float x_min = std::numeric_limits<float>::max();
    float y_max = std::numeric_limits<float>::min();
    float y_min = std::numeric_limits<float>::max();
    for (auto it = morphology->begin(); it != morphology->end(); it++) {
      float x_coord = std::round(it->x * 10);
      float y_coord = std::round(it->y * 10);
      keypoints_to_triangulate.at(i).x = x_coord;
      keypoints_to_triangulate.at(i).y = y_coord;

      // Find x max
      if (x_coord > x_max) x_max = x_coord;
      // Find x min
      if (x_coord < x_min) x_min = x_coord;
      // Find y may
      if (y_coord > y_max) y_max = y_coord;
      // Find y min
      if (y_coord < y_min) y_min = y_coord;

      // Increase count
      ++i;
    }
    CHECK_GE(x_max, x_min);
    CHECK_GE(y_max, y_min);
    cv::Size size(std::abs(y_max - y_min), std::abs(x_max - x_min));

    std::vector<cv::Vec6f> triangles =
        createMesh2dImpl(size, &keypoints_to_triangulate);
    std::vector<cv::Point> pt(3);
    cv::Mat img = cv::Mat::zeros(size, CV_8UC3);
    static const cv::Scalar kDelaunayColor(0, 255, 0);    // Green
    static const cv::Scalar kMeshVertexColor(255, 0, 0);  // Blue
    for (const cv::Vec6f& triangle : triangles) {
      // Visualize mesh vertices.
      /// We shift the vertices in order to have its coords all positive.
      pt[0] = cv::Point(cvRound(triangle[0] + std::abs(y_min)),
                        cvRound(triangle[1] + std::abs(x_min)));
      pt[1] = cv::Point(cvRound(triangle[2] + std::abs(y_min)),
                        cvRound(triangle[3] + std::abs(x_min)));
      pt[2] = cv::Point(cvRound(triangle[4] + std::abs(y_min)),
                        cvRound(triangle[5] + std::abs(x_min)));
      cv::circle(img, pt[0], 2, kMeshVertexColor, CV_FILLED, CV_AA, 0);
      cv::circle(img, pt[1], 2, kMeshVertexColor, CV_FILLED, CV_AA, 0);
      cv::circle(img, pt[2], 2, kMeshVertexColor, CV_FILLED, CV_AA, 0);

      // Visualize mesh edges.
      cv::line(img, pt[0], pt[1], kDelaunayColor, 1, CV_AA, 0);
      cv::line(img, pt[1], pt[2], kDelaunayColor, 1, CV_AA, 0);
      cv::line(img, pt[2], pt[0], kDelaunayColor, 1, CV_AA, 0);
    }
    cv::imshow("Triangulation", img);
    cv::waitKey(0);
  }

  cv::Mat getMorphologyRoom(const cv::Mat& img) {
    static constexpr float dilation_size = 3;
    cv::Mat dilation_element = cv::getStructuringElement(
        cv::MORPH_RECT,
        cv::Size(2 * dilation_size + 1, 2 * dilation_size + 1),
        cv::Point(-1, -1));

    static float erosion_size = 3;
    cv::Mat erosion_element = cv::getStructuringElement(
        cv::MORPH_RECT,
        cv::Size(2 * erosion_size + 1, 2 * erosion_size + 1),
        cv::Point(-1, -1));

    // Only eroding now...
    cv::Mat erosion_dst;
    // Do it twice.
    cv::erode(img, erosion_dst, erosion_element);
    cv::erode(erosion_dst, erosion_dst, erosion_element);

    ///// Apply the dilation operation
    cv::dilate(erosion_dst, erosion_dst, dilation_element);

    cv::imshow("After Erosion", erosion_dst);
    return erosion_dst;
  }

  void getSlicRoom(const cv::Mat& img) {
    double t = (double)cv::getTickCount();
    // SLIC
    int region_size = 50;
    int ruler = 30;
    int min_element_size = 50;
    int algorithm = 0;
    int num_iterations = 3;
    cv::Ptr<cv::ximgproc::SuperpixelSLIC> slic =
        cv::ximgproc::createSuperpixelSLIC(
            img, algorithm + cv::ximgproc::SLIC, region_size, float(ruler));
    slic->iterate(num_iterations);
    if (min_element_size > 0) slic->enforceLabelConnectivity(min_element_size);

    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    std::cout << "SLIC" << (algorithm ? 'O' : ' ') << " segmentation took "
              << (int)(t * 1000) << " ms with "
              << slic->getNumberOfSuperpixels() << " superpixels" << endl;

    // get the contours for displaying
    cv::Mat mask;
    slic->getLabelContourMask(mask, true);
    cv::Mat output = img.clone();
    output.setTo(cv::Scalar(0, 0, 255), mask);
    cv::imshow("After SLIC", output);
  }

  template <class T>
  typename pcl::PointCloud<T>::Ptr downsamplePcl(
      const typename pcl::PointCloud<T>::Ptr& input,
      const float& leaf_size = 0.05f) {
    typename pcl::PointCloud<T>::Ptr downsampled_pcl(new pcl::PointCloud<T>);
    static constexpr bool kApproxDownsampler = false;
    if (kApproxDownsampler) {
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

 private:
  std::string world_frame_;
  ros::NodeHandle nh_private_;
  ros::Publisher pcl_pub_;
  MarchingCubesParams mc_params_;
  OutlierFilterParams of_params_;
};

}  // namespace kimera
