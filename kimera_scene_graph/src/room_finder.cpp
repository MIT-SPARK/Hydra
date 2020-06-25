#include "kimera_scene_graph/room_finder.h"

#include <kimera_scene_graph/object_finder.h>

#include <limits>

namespace kimera {

RoomFinder::RoomFinder(const ros::NodeHandle& nh_private,
                       const std::string& world_frame)
    : pcl_pub_(), nh_private_(nh_private), world_frame_(world_frame) {
  pcl_pub_ = nh_private_.advertise<ColorPointCloud>("room_clusters", 1, true);
}

void RoomFinder::findRooms(const vxb::Mesh& mesh, Centroids* room_centroids) {}

/**
 * @brief RoomFinder::findRooms
 * Uses Semantic ESDF to find the rooms
 * @param cloud
 * @param room_centroids
 * @param room_pcls
 * @return
 */
IntensityPointCloud::Ptr RoomFinder::findRooms(
    const IntensityPointCloud::Ptr& cloud,
    Centroids* room_centroids,
    std::vector<ColorPointCloud::Ptr>* room_pcls) {
  LOG(INFO) << "Start Room finding using ESDF";
  // Downsample pcl
  IntensityPointCloud::Ptr downsampled_pcl(new IntensityPointCloud());
  downsampled_pcl = downsamplePcl<IntensityPoint>(cloud, 0.05f);

  // Pass through values below the given esdf truncation
  // We use -1.0 instead of 0.0 bcs we also want to filter out 0.
  downsampled_pcl = passThroughFilter1D<IntensityPoint>(
      downsampled_pcl, "intensity", -1.0, kEsdfTruncation, true);

  // Region growing clustering
  ObjectFinder<IntensityPoint> object_finder(world_frame_,
                                             ObjectFinderType::kEuclidean);
  EuclideanClusterEstimatorParams params;
  params.min_cluster_size_ = 300;
  object_finder.updateEuclideanClusterParams(params);
  std::vector<IntensityPointCloud::Ptr> room_pcls_noncolored;
  ObjectFinder<IntensityPoint>::BoundingBoxes bounding_boxes;
  ColorPointCloud::Ptr room_pcl_colored = object_finder.findObjects(
      downsampled_pcl, room_centroids, &room_pcls_noncolored, &bounding_boxes);
  // Generate colored point cloud out of the noncolored rooms pcl.
  // Repaint room_pcls to uniform color for better visualization of edges.
  room_pcls->resize(room_pcls_noncolored.size());
  static constexpr int32_t rgb =
      (static_cast<uint32_t>(0u) << 16 | static_cast<uint32_t>(255u) << 8 |
       static_cast<uint32_t>(0u));
  for (size_t i = 0u; i < room_pcls_noncolored.size(); ++i) {
    auto& pcl_ptr = room_pcls->at(i);
    pcl_ptr.reset(new ColorPointCloud);
    pcl::copyPointCloud(*room_pcls_noncolored.at(i), *pcl_ptr);
    for (auto& point : pcl_ptr->points) point.rgb = rgb;
  }

  room_pcl_colored->header.frame_id = world_frame_;
  for (auto& it : room_pcl_colored->points) {
    // Move all points to level 5
    it.z = 10;
  }
  pcl_pub_.publish(*room_pcl_colored);
  LOG(INFO) << "Finished Room finding using ESDF";
  return downsampled_pcl;
}

IntensityPointCloud::Ptr RoomFinder::findRooms(
    const ColorPointCloud::Ptr& cloud,
    Centroids* room_centroids,
    std::vector<ColorPointCloud::Ptr>* room_pcls) {
  CHECK_NOTNULL(room_centroids);
  CHECK_NOTNULL(room_pcls);

  // Project pointcloud onto plane
  ColorPointCloud::Ptr projected_pcl(new ColorPointCloud());
  projectPointcloudToPlane(cloud, &(*projected_pcl));

  ColorPointCloud::Ptr downsampled_pcl(new ColorPointCloud());
  downsampled_pcl = downsamplePcl<ColorPoint>(projected_pcl, 0.05f);

  ColorPointCloud::Ptr final_cloud(new ColorPointCloud());
  pcl::RadiusOutlierRemoval<ColorPoint> outrem;
  outrem.setRadiusSearch(of_params_.radius_search);
  outrem.setMinNeighborsInRadius(of_params_.min_neighbors_in_radius);
  outrem.setInputCloud(downsampled_pcl);
  outrem.filter(*final_cloud);

  IntensityPointCloud::Ptr morphology(new IntensityPointCloud());
  pcl::copyPointCloud(*final_cloud, *morphology);

  // Make morphology an organized cloud

  // Triangulate morphology
  // triangulatePointcloud(morphology);
  cv::Mat organized_img;
  IntensityPointCloud::Ptr organized_cloud(new IntensityPointCloud);
  makeCvImgFromPcl(morphology, &organized_img, &organized_cloud, 0.1, true);
  CHECK_EQ(organized_cloud->size(), organized_img.size().area());

  // Pass through intensity, we only want to keep intensity = 1.0 points.
  organized_cloud = passThroughFilter1D<IntensityPoint>(
      organized_cloud, "intensity", 0.5, 1.5);

  // Downsample the cloud like crazy...
  organized_cloud = downsamplePcl<IntensityPoint>(organized_cloud, 0.20f);

  ObjectFinder<IntensityPoint> object_finder(world_frame_,
                                             ObjectFinderType::kEuclidean);
  EuclideanClusterEstimatorParams params;
  params.min_cluster_size_ = 200;
  object_finder.updateEuclideanClusterParams(params);
  std::vector<IntensityPointCloud::Ptr> room_pcls_noncolored;
  ObjectFinder<IntensityPoint>::BoundingBoxes bounding_boxes;
  ColorPointCloud::Ptr pcl = object_finder.findObjects(
      organized_cloud, room_centroids, &room_pcls_noncolored, &bounding_boxes);
  // Generate colored point cloud out of the noncolored rooms pcl.
  // Repaint room_pcls to uniform color for better visualization of edges.
  room_pcls->resize(room_pcls_noncolored.size());
  static constexpr int32_t rgb =
      (static_cast<uint32_t>(0u) << 16 | static_cast<uint32_t>(255u) << 8 |
       static_cast<uint32_t>(0u));
  for (size_t i = 0u; i < room_pcls_noncolored.size(); ++i) {
    auto& pcl_ptr = room_pcls->at(i);
    pcl_ptr.reset(new ColorPointCloud);
    pcl::copyPointCloud(*room_pcls_noncolored.at(i), *pcl_ptr);
    for (auto& point : pcl_ptr->points) point.rgb = rgb;
  }

  pcl->header.frame_id = world_frame_;
  for (auto& it : pcl->points) {
    // Move all points to level 5
    it.z = 10;
  }
  pcl_pub_.publish(*pcl);

  return organized_cloud;
}

void RoomFinder::poissonReconstruction(const PointCloud::Ptr& cloud,
                                       pcl::PolygonMesh* mesh) {
  CHECK_NOTNULL(mesh);

  // cout << "begin moving least squares" << endl;
  // MovingLeastSquares<PointXYZ, PointXYZ> mls;
  // mls.setInputCloud(filtered);
  // mls.setSearchRadius(0.01);
  // mls.setPolynomialFit(true);
  // mls.setPolynomialOrder(2);
  // mls.setUpsamplingMethod(MovingLeastSquares<PointXYZ,
  // PointXYZ>::SAMPLE_LOCAL_PLANE); mls.setUpsamplingRadius(0.005);
  // mls.setUpsamplingStepSize(0.003);

  // PointCloud<PointXYZ>::Ptr cloud_smoothed (new PointCloud<PointXYZ>());
  // mls.process(*cloud_smoothed);
  // cout << "MLS complete" << endl;

  pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
  ne.setNumberOfThreads(8);
  ne.setInputCloud(cloud);
  ne.setRadiusSearch(0.01);
  Eigen::Vector4f centroid;
  compute3DCentroid(*cloud, centroid);
  ne.setViewPoint(centroid[0], centroid[1], centroid[2]);

  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(
      new pcl::PointCloud<pcl::Normal>());
  ne.compute(*cloud_normals);

  for (size_t i = 0; i < cloud_normals->size(); ++i) {
    cloud_normals->points[i].normal_x *= -1;
    cloud_normals->points[i].normal_y *= -1;
    cloud_normals->points[i].normal_z *= -1;
  }

  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_smoothed_normals(
      new pcl::PointCloud<pcl::PointNormal>());
  concatenateFields(*cloud, *cloud_normals, *cloud_smoothed_normals);

  pcl::Poisson<pcl::PointNormal> poisson;
  poisson.setDepth(9);
  poisson.setInputCloud(cloud_smoothed_normals);
  poisson.reconstruct(*mesh);
}

void RoomFinder::marchingCubes(const PointCloud::Ptr& cloud,
                               pcl::PolygonMesh* mesh) {
  CHECK_NOTNULL(mesh);

  pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree1(
      new pcl::search::KdTree<pcl::PointXYZ>);
  tree1->setInputCloud(cloud);
  ne.setInputCloud(cloud);
  ne.setSearchMethod(tree1);
  ne.setKSearch(20);
  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
  ne.compute(*normals);

  // Concatenate the XYZ and normal fields*
  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(
      new pcl::PointCloud<pcl::PointNormal>);
  concatenateFields(*cloud, *normals, *cloud_with_normals);

  pcl::MarchingCubes<pcl::PointNormal>* mc;
  if (mc_params_.hoppe_or_rbf == 0) {
    mc = new pcl::MarchingCubesHoppe<pcl::PointNormal>();
  } else {
    mc = new pcl::MarchingCubesRBF<pcl::PointNormal>();
    (reinterpret_cast<pcl::MarchingCubesRBF<pcl::PointNormal>*>(mc))
        ->setOffSurfaceDisplacement(mc_params_.off_surface_displacement);
  }

  mc->setIsoLevel(mc_params_.iso_level_);
  mc->setGridResolution(
      mc_params_.grid_res, mc_params_.grid_res, mc_params_.grid_res);
  mc->setPercentageExtendGrid(mc_params_.extend_percentage);
  mc->setInputCloud(cloud_with_normals);

  LOG(INFO) << "Computing Marching Cubes";
  mc->reconstruct(*mesh);
  delete mc;
  LOG(INFO) << "Done computing Marching Cubes";
}

void RoomFinder::superVoxelize(const PointCloudT::Ptr& cloud,
                               PlanarRegions* planar_regions) {
  float voxel_resolution = 0.008f;
  float seed_resolution = 0.1f;
  float color_importance = 0.2f;
  float spatial_importance = 0.4f;
  float normal_importance = 1.0f;

  //////////////////////////////  //////////////////////////////
  ////// This is how to use supervoxels
  //////////////////////////////  //////////////////////////////

  pcl::SupervoxelClustering<PointT> super(voxel_resolution, seed_resolution);
  // Our cloud is not organized...
  // bool disable_transform = true;
  // if (disable_transform) super.setUseSingleCameraTransform(false);

  super.setInputCloud(cloud);
  super.setColorImportance(color_importance);
  super.setSpatialImportance(spatial_importance);
  super.setNormalImportance(normal_importance);

  std::map<std::uint32_t, pcl::Supervoxel<PointT>::Ptr> supervoxel_clusters;

  pcl::console::print_highlight("Extracting supervoxels!\n");
  super.extract(supervoxel_clusters);
  LOG(INFO) << "Found " << supervoxel_clusters.size() << " supervoxels.";

  PointLCloudT::Ptr labeled_voxel_cloud = super.getLabeledVoxelCloud();

  PointNCloudT::Ptr sv_normal_cloud =
      super.makeSupervoxelNormalCloud(supervoxel_clusters);
  // We have this disabled so graph is easy to see, uncomment to see
  // supervoxel normals viewer->addPointCloudNormals<PointNormal>
  // (sv_normal_cloud,1,0.05f, "supervoxel_normals");

  pcl::console::print_highlight("Getting supervoxel adjacency\n");
  std::multimap<std::uint32_t, std::uint32_t> supervoxel_adjacency;
  super.getSupervoxelAdjacency(supervoxel_adjacency);
  // To make a graph of the supervoxel adjacency, we need to iterate through
  // the supervoxel adjacency multimap
  for (auto label_itr = supervoxel_adjacency.cbegin();
       label_itr != supervoxel_adjacency.cend();) {
    // First get the label
    std::uint32_t supervoxel_label = label_itr->first;
    // Now get the supervoxel corresponding to the label
    pcl::Supervoxel<PointT>::Ptr supervoxel =
        supervoxel_clusters.at(supervoxel_label);

    // Now we need to iterate through the adjacent supervoxels and make a
    // point cloud of them
    PointCloudT adjacent_supervoxel_centers;
    for (auto adjacent_itr =
             supervoxel_adjacency.equal_range(supervoxel_label).first;
         adjacent_itr !=
         supervoxel_adjacency.equal_range(supervoxel_label).second;
         ++adjacent_itr) {
      pcl::Supervoxel<PointT>::Ptr neighbor_supervoxel =
          supervoxel_clusters.at(adjacent_itr->second);
      // adjacent_supervoxel_centers.push_back(neighbor_supervoxel->centroid_);
    }
    // Now we make a name for this polygon
    std::stringstream ss;
    ss << "supervoxel_" << supervoxel_label;
    // This function is shown below, but is beyond the scope of this tutorial
    // - basically it just generates a "star" polygon mesh from the points
    // given
    // addSupervoxelConnectionsToViewer (supervoxel->centroid_,
    // adjacent_supervoxel_centers, ss.str (), viewer);
    // Move iterator forward to next label
    label_itr = supervoxel_adjacency.upper_bound(supervoxel_label);
  }
}

void RoomFinder::multiPlaneSegmenter(const ColorPointCloud::Ptr& cloud,
                                     PlanarRegions* planar_regions) {
  CHECK_NOTNULL(planar_regions);

  // Estimate Normals
  // typename pcl::search::Search<PointT>::Ptr tree(
  //    new pcl::search::KdTree<PointT>);
  // pcl::PointCloud<pcl::Normal>::Ptr normal_cloud(
  //    new pcl::PointCloud<pcl::Normal>);
  // pcl::NormalEstimation<PointT, pcl::Normal> normal_estimator;
  // normal_estimator.setSearchMethod(tree);
  // normal_estimator.setKSearch(20);

  // normal_estimator.setInputCloud(cloud);
  // normal_estimator.compute(*normal_cloud);
}

bool RoomFinder::fastTriangulation(const PointCloud::Ptr& cloud,
                                   pcl::PolygonMesh* mesh) {
  CHECK_NOTNULL(mesh);
  // Normal estimation*
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
      new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(cloud);
  n.setInputCloud(cloud);
  n.setSearchMethod(tree);
  n.setKSearch(20);
  n.compute(*normals);
  //* normals should not contain the point normals + surface curvatures

  // Concatenate the XYZ and normal fields*
  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(
      new pcl::PointCloud<pcl::PointNormal>);
  pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);
  //* cloud_with_normals = cloud + normals

  // Create search tree*
  pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(
      new pcl::search::KdTree<pcl::PointNormal>);
  tree2->setInputCloud(cloud_with_normals);

  // Initialize objects
  pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;

  // Set the maximum distance between connected points (maximum edge length)
  gp3.setSearchRadius(0.025);

  // Set typical values for the parameters
  gp3.setMu(2.5);
  gp3.setMaximumNearestNeighbors(100);
  gp3.setMaximumSurfaceAngle(M_PI / 4);  // 45 degrees
  gp3.setMinimumAngle(M_PI / 18);        // 10 degrees
  gp3.setMaximumAngle(2 * M_PI / 3);     // 120 degrees
  gp3.setNormalConsistency(false);

  // Get result
  gp3.setInputCloud(cloud_with_normals);
  gp3.setSearchMethod(tree2);
  gp3.reconstruct(*mesh);

  // Additional vertex information
  std::vector<int> parts = gp3.getPartIDs();
  std::vector<int> states = gp3.getPointStates();

  // Finish
  return (0);
}

void RoomFinder::projectPointcloudToPlane(const ColorPointCloud::Ptr& cloud,
                                          ColorPointCloud* cloud_projected) {
  CHECK_NOTNULL(cloud_projected);
  // Create a set of planar coefficients with X=Y=0,Z=1
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
  coefficients->values.resize(4);
  coefficients->values[0] = coefficients->values[1] = 0;
  coefficients->values[2] = 1.0;
  coefficients->values[3] = 0;

  // Project the model inliers
  pcl::ProjectInliers<ColorPoint> proj;
  proj.setModelType(pcl::SACMODEL_PLANE);
  proj.setInputCloud(cloud);
  proj.setModelCoefficients(coefficients);
  proj.filter(*cloud_projected);
}

// Create a 2D mesh from 2D corners in an image
// Returns the actual keypoints used to perform the triangulation.
std::vector<cv::Vec6f> RoomFinder::createMesh2dImpl(
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

bool RoomFinder::makeCvImgFromPcl(const IntensityPointCloud::Ptr& input,
                                  cv::Mat* img,
                                  IntensityPointCloud::Ptr* cloud,
                                  const double& resolution,
                                  const bool& visualize) {
  CHECK_NOTNULL(img);
  CHECK_NOTNULL(cloud);
  // Get the minimum and maximum dimensions
  Eigen::Vector4f min_p, max_p;
  pcl::getMinMax3D<IntensityPoint>(*input, min_p, max_p);

  // Check that the resolution is not too small, given the size of the data
  double inverse_resolution = 1 / resolution;
  std::int64_t dx =
      static_cast<std::int64_t>((max_p[0] - min_p[0]) * inverse_resolution) + 1;
  std::int64_t dy =
      static_cast<std::int64_t>((max_p[1] - min_p[1]) * inverse_resolution) + 1;

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

void RoomFinder::triangulatePointcloud(
    const IntensityPointCloud::Ptr& morphology) {
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

cv::Mat RoomFinder::getMorphologyRoom(const cv::Mat& img) {
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

void RoomFinder::getSlicRoom(const cv::Mat& img) {
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
            << (int)(t * 1000) << " ms with " << slic->getNumberOfSuperpixels()
            << " superpixels" << endl;

  // get the contours for displaying
  cv::Mat mask;
  slic->getLabelContourMask(mask, true);
  cv::Mat output = img.clone();
  output.setTo(cv::Scalar(0, 0, 255), mask);
  cv::imshow("After SLIC", output);
}

}  // namespace kimera
