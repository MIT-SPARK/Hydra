#include "kimera_dsg_builder/object_finder.h"
#include "kimera_dsg_builder/common.h"

#include <actionlib/client/terminal_state.h>
#include <kimera_dsg/bounding_box.h>
#include <kimera_semantics/common.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/common/centroid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>

#include <ostream>

namespace kimera {

template <typename PointT>
ClusterIndices estimateClustersRegionGrowing(
    const RegionGrowingClusteringParams& params,
    const typename pcl::PointCloud<PointT>::Ptr& cloud,
    const pcl::IndicesPtr& active_indices = nullptr) {
  CHECK(cloud);

  using namespace pcl::search;
  typename Search<PointT>::Ptr tree(new KdTree<PointT>());
  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>());

  typename pcl::NormalEstimation<PointT, pcl::Normal> normal_estimator;
  normal_estimator.setSearchMethod(tree);
  normal_estimator.setKSearch(params.normal_estimator_neighbour_size);
  normal_estimator.setInputCloud(cloud);
  if (active_indices) {
    normal_estimator.setIndices(active_indices);
  }

  normal_estimator.compute(*normals);

  typename pcl::RegionGrowing<PointT, pcl::Normal> estimator;
  estimator.setMinClusterSize(params.min_cluster_size);
  estimator.setMaxClusterSize(params.max_cluster_size);
  estimator.setNumberOfNeighbours(params.number_of_neighbours);
  estimator.setSmoothnessThreshold(params.smoothness_threshold);
  estimator.setCurvatureThreshold(params.curvature_threshold);
  CHECK(estimator.getCurvatureTestFlag());
  CHECK(estimator.getSmoothModeFlag());

  estimator.setSearchMethod(tree);
  estimator.setInputNormals(normals);
  estimator.setInputCloud(cloud);
  if (active_indices) {
    estimator.setIndices(active_indices);
  }

  ClusterIndices cluster_indices;
  estimator.extract(cluster_indices);
  return cluster_indices;
}

template <typename PointT>
ClusterIndices estimateClustersEuclidean(
    const EuclideanClusteringParams& params,
    const typename pcl::PointCloud<PointT>::Ptr& cloud,
    const pcl::IndicesPtr& active_indices = nullptr) {
  using namespace pcl::search;
  typename KdTree<PointT>::Ptr tree(new KdTree<PointT>());
  tree->setInputCloud(cloud);

  typename pcl::EuclideanClusterExtraction<PointT> estimator;
  estimator.setClusterTolerance(params.cluster_tolerance);
  estimator.setMinClusterSize(params.min_cluster_size);
  estimator.setMaxClusterSize(params.max_cluster_size);
  estimator.setSearchMethod(tree);
  estimator.setInputCloud(cloud);
  if (active_indices) {
    estimator.setIndices(active_indices);
  }

  ClusterIndices cluster_indices;
  estimator.extract(cluster_indices);
  return cluster_indices;
}

// TODO(nathan) consider not making individual pointclouds for each object...
template <typename PointT>
typename Cluster<PointT>::Vector getClusterInfo(
    const typename pcl::PointCloud<PointT>::Ptr& cloud,
    const ClusterIndices& indices) {
  typename Cluster<PointT>::Vector clusters;
  clusters.resize(indices.size());
  for (size_t k = 0; k < indices.size(); ++k) {
    clusters.at(k).cloud.reset(new pcl::PointCloud<PointT>());

    const auto& object_indices = indices.at(k).indices;
    clusters.at(k).cloud->resize(object_indices.size());
    clusters.at(k).indices = indices.at(k);

    for (size_t i = 0; i < object_indices.size(); ++i) {
      const PointT& color_point = cloud->at(object_indices.at(i));
      clusters.at(k).cloud->at(i) = color_point;

      clusters.at(k).centroid.add(
          pcl::PointXYZ(color_point.x, color_point.y, color_point.z));
    }
  }

  return clusters;
}

std::ostream& operator<<(std::ostream& out, const RegionGrowingClusteringParams& p) {
  // clang-format off
  out << "\n================== Region Growing Cluster Config ====================\n";
  out << " - normal_estimator_neighbour_size:  " << p.normal_estimator_neighbour_size << '\n';
  out << " - min_cluster_size:                 " << p.min_cluster_size << '\n';
  out << " - max_cluster_size:                 " << p.max_cluster_size << '\n';
  out << " - number_of_neighbours:             " << p.number_of_neighbours << '\n';
  out << " - smoothness_threshold_:            " << p.smoothness_threshold << '\n';
  out << " - curvature_threshold_:             " << p.curvature_threshold << '\n';
  out << "==============================================================\n";
  // clang-format on
  return out;
}

std::ostream& operator<<(std::ostream& out, const EuclideanClusteringParams& p) {
  std::stringstream ss;
  // clang-format off
  out << "\n================== Euclidean Cluster Config ====================\n";
  out << " - min_cluster_size:                 " << p.min_cluster_size << '\n';
  out << " - max_cluster_size:                 " << p.max_cluster_size << '\n';
  out << " - cluster_tolerance:                " << p.cluster_tolerance << '\n';
  out << "==============================================================\n";
  // clang-format on
  return out;
}

ObjectFinder::ObjectFinder(ObjectFinderType type)
    : type_(type), next_object_id_('O', 0) {}

void ObjectFinder::connectToObjectDb() {
  // Build Object database action client
  ROS_INFO("Creating object database client");
  object_db_client_.reset(new ObjectDBClient("/object_db", true));
  ROS_INFO("Waiting for object database server");
  object_db_client_->waitForServer();
  ROS_INFO("Object database server connected");
}

void ObjectFinder::updateClusterEstimator(ObjectFinderType type) { type_ = type; }

void ObjectFinder::setRegionGrowingParams(const RegionGrowingClusteringParams& params) {
  region_growing_params_ = params;
}

void ObjectFinder::setEuclideanClusterParams(const EuclideanClusteringParams& params) {
  euclidean_params_ = params;
}

std::ostream& operator<<(std::ostream& out, const ObjectFinder& finder) {
  switch (static_cast<ObjectFinderType>(finder.type_)) {
    case ObjectFinderType::kEuclidean:
      out << "Object Finder: Euclidean";
      out << finder.euclidean_params_;
      break;
    case ObjectFinderType::kRegionGrowing:
      out << "Object Finder: Region growing";
      out << finder.region_growing_params_;
      break;
    default:
      out << "Object Finder: Unkown object finder type";
      break;
  }
  return out;
}

void ObjectFinder::addObjectsToGraph(const SubMesh& mesh,
                                     const NodeColor& label_color,
                                     SemanticLabel label,
                                     DynamicSceneGraph* scene_graph) {
  CHECK(scene_graph);
  CHECK(mesh.vertices);

  VLOG(1) << "Extracting objects for label: " << std::to_string(label);
  if (mesh.vertices->empty()) {
    VLOG(1) << "Skipping label " << label << " as input was empty";
    return;
  }

  // TODO(nathan) eventually we'll refactor this to group centroids and stuff
  OfflineObjectClusters clusters;
  findObjectsOffline(mesh.vertices, clusters);

  // TODO(nathan) use the registration for something
  // ObjectPointClouds registered_objects =
  // registerObjects(objects, std::to_string(label));

  size_t num_objects = 0;
  for (const auto& cluster : clusters) {
    ObjectNodeAttributes::Ptr attrs = std::make_unique<ObjectNodeAttributes>();
    attrs->semantic_label = label;
    attrs->color = label_color,
    attrs->name = std::to_string(label) + std::to_string(num_objects);
    attrs->bounding_box = BoundingBox::extract(cluster.cloud, BoundingBox::Type::AABB);

    pcl::PointXYZ centroid;
    cluster.centroid.get(centroid);
    attrs->position << centroid.x, centroid.y, centroid.z;

    scene_graph->emplaceNode(
        KimeraDsgLayers::OBJECTS, next_object_id_, std::move(attrs));

    for (const auto& idx : cluster.indices.indices) {
      scene_graph->insertMeshEdge(next_object_id_, mesh.vertex_map.at(idx));
    }

    ++next_object_id_;
    ++num_objects;
  }
}

ColorPointCloud::Ptr ObjectFinder::findObjectsOffline(const ColorPointCloud::Ptr& cloud,
                                                      OfflineObjectClusters& clusters) {
  CHECK(cloud);

  ClusterIndices cluster_indices;
  switch (type_) {
    case ObjectFinderType::kRegionGrowing:
      VLOG(2) << "Using region growing object finder.";
      cluster_indices = estimateClustersRegionGrowing<pcl::PointXYZRGB>(
          region_growing_params_, cloud);
      break;
    case ObjectFinderType::kEuclidean:
    default:
      VLOG(2) << "Using euclidean object finder.";
      cluster_indices =
          estimateClustersEuclidean<pcl::PointXYZRGB>(euclidean_params_, cloud);
      break;
  }

  clusters = getClusterInfo<pcl::PointXYZRGB>(cloud, cluster_indices);
  return getColoredCloud(cloud, cluster_indices);
}

ObjectClusters ObjectFinder::findObjects(
    const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& cloud,
    const std::vector<size_t>& active_indices) {
  CHECK(cloud);

  // in general, this is unsafe, but PCL doesn't offer us any alternative
  pcl::IndicesPtr cloud_indices(
      new std::vector<int>(active_indices.begin(), active_indices.end()));

  ClusterIndices cluster_indices;
  switch (type_) {
    case ObjectFinderType::kRegionGrowing:
      cluster_indices = estimateClustersRegionGrowing<pcl::PointXYZRGBA>(
          region_growing_params_, cloud, cloud_indices);
      break;
    case ObjectFinderType::kEuclidean:
    default:
      cluster_indices = estimateClustersEuclidean<pcl::PointXYZRGBA>(
          euclidean_params_, cloud, cloud_indices);
      break;
  }

  return getClusterInfo<pcl::PointXYZRGBA>(cloud, cluster_indices);
}

ColorPointCloud::Ptr ObjectFinder::getColoredCloud(const ColorPointCloud::Ptr& input,
                                                   const ClusterIndices& clusters) {
  ColorPointCloud::Ptr colored_cloud(new ColorPointCloud);

  if (!clusters.empty()) {
    srand(static_cast<unsigned int>(time(nullptr)));
    std::vector<unsigned char> colors;
    for (std::size_t i_segment = 0; i_segment < clusters.size(); i_segment++) {
      colors.push_back(static_cast<unsigned char>(rand() % 256));
      colors.push_back(static_cast<unsigned char>(rand() % 256));
      colors.push_back(static_cast<unsigned char>(rand() % 256));
    }

    colored_cloud->width = input->width;
    colored_cloud->height = input->height;
    colored_cloud->is_dense = input->is_dense;
    for (std::size_t i_point = 0; i_point < input->points.size(); i_point++) {
      ColorPoint point;
      point.x = *(input->points[i_point].data);
      point.y = *(input->points[i_point].data + 1);
      point.z = *(input->points[i_point].data + 2);
      point.r = 255;
      point.g = 0;
      point.b = 0;
      colored_cloud->points.push_back(point);
    }

    int next_color = 0;
    for (auto i_segment = clusters.cbegin(); i_segment != clusters.cend();
         i_segment++) {
      for (auto i_point = i_segment->indices.cbegin();
           i_point != i_segment->indices.cend();
           i_point++) {
        int index;
        index = *i_point;
        colored_cloud->points[index].r = colors[3 * next_color];
        colored_cloud->points[index].g = colors[3 * next_color + 1];
        colored_cloud->points[index].b = colors[3 * next_color + 2];
      }
      next_color++;
    }
  }

  return colored_cloud;
}

ObjectPointClouds ObjectFinder::registerObjects(const ObjectPointClouds& object_pcls,
                                                const std::string semantic_label) {
  CHECK(object_db_client_);
  LOG(INFO) << "Sending object point clouds to object database.";
  // For storing all registrated object point clouds
  ObjectPointClouds registrated_object_pcls;

  // Query object database for all point clouds
  // Object pcls is a vector of pointers to pcl::XYZRGB point clouds
  for (const auto& color_pcl : object_pcls) {
    if (color_pcl->points.size() == 0) {
      LOG(INFO) << "Empty point cloud given to object database client.";
      registrated_object_pcls.push_back(color_pcl);
      continue;
    }
    LOG(INFO) << "Object pcl size:" << color_pcl->points.size();

    // Create goal for object db
    object_db::ObjectRegistrationGoal c_goal;
    c_goal.semantic_label = semantic_label;

    // Get current color
    const auto& color_r = color_pcl->points[0].r;
    const auto& color_g = color_pcl->points[0].g;
    const auto& color_b = color_pcl->points[0].b;

    // Convert colored point cloud to sensor msg point cloud
    for (size_t p_idx = 0; p_idx < color_pcl->size(); ++p_idx) {
      geometry_msgs::Point32 c_point;
      c_point.x = color_pcl->points[p_idx].x;
      c_point.y = color_pcl->points[p_idx].y;
      c_point.z = color_pcl->points[p_idx].z;
      c_goal.dst.points.push_back(c_point);
    }

    // Send goal to object databse action server
    object_db_client_->sendGoal(c_goal);

    // Deal with the result
    bool finished = object_db_client_->waitForResult(ros::Duration(30));
    bool aborted =
        object_db_client_->getState() == actionlib::SimpleClientGoalState::ABORTED;
    if (aborted) {
      LOG(INFO) << "Object database aborted.";
      registrated_object_pcls.push_back(color_pcl);
    } else if (!finished) {
      LOG(INFO) << "Object database did not finish before the timeout.";
      registrated_object_pcls.push_back(color_pcl);
    } else {
      auto result = object_db_client_->getResult();
      auto registrated_object = result->aligned_object;

      // Convert sensor msg point cloud type to colored pcl
      ColorPointCloud::Ptr registrated_pcl(new ColorPointCloud);
      for (const auto& o_point : registrated_object.points) {
        ColorPoint c_point;
        c_point.x = o_point.x;
        c_point.y = o_point.y;
        c_point.z = o_point.z;
        c_point.r = color_r;
        c_point.g = color_g;
        c_point.b = color_b;
        registrated_pcl->points.push_back(c_point);
      }
      registrated_object_pcls.push_back(registrated_pcl);
      LOG(INFO) << "Object database query successful.";
      LOG(INFO) << "Registrated object size: " << registrated_pcl->size();
    }
  }

  if (registrated_object_pcls.size() != object_pcls.size()) {
    LOG(INFO) << "Registrated objects size mismatch!";
  }
  return registrated_object_pcls;
}

}  // namespace kimera
