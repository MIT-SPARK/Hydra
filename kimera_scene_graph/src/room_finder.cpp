#include "kimera_scene_graph/room_finder.h"

#include <kimera_scene_graph/object_finder.h>

#include <limits>

namespace kimera {

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
  ColorPointCloud::Ptr pcl = object_finder.findObjects(
      downsampled_pcl, room_centroids, &room_pcls_noncolored, &bounding_boxes);
  // Generate colored point cloud out of the noncolored rooms pcl.
  // Repaint room_pcls to uniform color for better visualization of edges.
  room_pcls->resize(room_pcls_noncolored.size());
  static constexpr int32_t rgb =
      (static_cast<uint32_t>(0u) << 16 |
       static_cast<uint32_t>(255u) << 8 |
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
  bool disable_transform = true;
  if (disable_transform) super.setUseSingleCameraTransform(false);

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

}  // namespace kimera
