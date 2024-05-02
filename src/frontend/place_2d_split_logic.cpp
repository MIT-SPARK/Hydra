#include "hydra/frontend/place_2d_split_logic.h"

#include <spark_dsg/dynamic_scene_graph.h>
#include <spark_dsg/node_attributes.h>

#include "hydra/utils/place_2d_ellipsoid_math.h"
#include "opencv2/imgproc.hpp"

namespace hydra {

void addRectInfo(const Place2d::CloudT& points,
                 const std::vector<Place2d::Index> mindices,
                 const double connection_ellipse_scale_factor,
                 Eigen::Vector2d& ellipse_centroid,
                 Eigen::Matrix2d& m_expand,
                 Eigen::Matrix2d& m_compress,
                 Eigen::Vector2d& cut_plane) {
  std::vector<cv::Point2f> region;
  for (auto midx : mindices) {
    region.push_back(cv::Point2f(points[midx].x(), points[midx].y()));
  }
  cv::RotatedRect box = cv::minAreaRect(region);

  cv::Point2f box_pts[4];
  box.points(box_pts);

  // Get rays along two sides of bounding box
  cv::Point2f e1_ray = box_pts[1] - box_pts[0];
  cv::Point2f e2_ray = box_pts[3] - box_pts[0];

  cv::Point2f long_ray = cv::norm(e1_ray) > cv::norm(e2_ray) ? e1_ray : e2_ray;

  cv::Point2f box_center = (box_pts[0] + box_pts[1] + box_pts[2] + box_pts[3]) / 4;
  ellipse_centroid(0) = box_center.x;
  ellipse_centroid(1) = box_center.y;

  m_expand(0, 0) = connection_ellipse_scale_factor * sqrt(2) * (e1_ray.x / 2);
  m_expand(1, 0) = connection_ellipse_scale_factor * sqrt(2) * (e1_ray.y / 2);
  m_expand(0, 1) = connection_ellipse_scale_factor * sqrt(2) * (e2_ray.x / 2);
  m_expand(1, 1) = connection_ellipse_scale_factor * sqrt(2) * (e2_ray.y / 2);

  Eigen::Matrix2d minv = m_expand.inverse();
  m_compress = minv.transpose() * minv;

  cut_plane(0) = long_ray.x;
  cut_plane(1) = long_ray.y;
}

void addRectInfo(const Place2d::CloudT& points,
                 const double connection_ellipse_scale_factor,
                 Place2dNodeAttributes& attrs) {
  Eigen::Vector2d cut_plane;         // place node attributes don't have cut_plane
  Eigen::Vector2d ellipse_centroid;  // attrs.ellipse_centroid is Vector3d
  addRectInfo(points,
              attrs.pcl_mesh_connections,
              connection_ellipse_scale_factor,
              ellipse_centroid,
              attrs.ellipse_matrix_expand,
              attrs.ellipse_matrix_compress,
              cut_plane);
  attrs.ellipse_centroid(0) = ellipse_centroid(0);
  attrs.ellipse_centroid(1) = ellipse_centroid(1);
}

void addRectInfo(const Place2d::CloudT& points,
                 const double connection_ellipse_scale_factor,
                 Place2d& place) {
  addRectInfo(points,
              place.indices,
              connection_ellipse_scale_factor,
              place.ellipse_centroid,
              place.ellipse_matrix_expand,
              place.ellipse_matrix_compress,
              place.cut_plane);
}

void addBoundaryInfo(const Place2d::CloudT& points,
                     const std::vector<Place2d::Index>& mindices,
                     Place2d::CentroidT& centroid,
                     std::vector<Eigen::Vector3d>& boundary,
                     std::vector<Place2d::Index>& boundary_mindices) {
  std::vector<cv::Point2f> region_pts;
  std::vector<Place2d::Index> region_to_cloud_index;
  centroid = Place2d::CentroidT();
  for (auto midx : mindices) {
    region_to_cloud_index.push_back(midx);
    region_pts.push_back(cv::Point2f(points[midx].x(), points[midx].y()));
    centroid.add(pcl::PointXYZ(points[midx].x(), points[midx].y(), points[midx].z()));
  }

  // compute convex hull for each place
  std::vector<int> ch;
  cv::convexHull(region_pts, ch);
  boundary.clear();
  boundary_mindices.clear();
  for (int pix : ch) {
    auto cloud_ix = region_to_cloud_index.at(pix);
    Place2d::PointT p = points[cloud_ix];
    Eigen::Vector3d v = {p.x(), p.y(), p.z()};
    boundary.push_back(v);
    boundary_mindices.push_back(cloud_ix);
  }
}

void addBoundaryInfo(const Place2d::CloudT& points, Place2d& place) {
  addBoundaryInfo(
      points, place.indices, place.centroid, place.boundary, place.boundary_indices);
}

void addBoundaryInfo(const Place2d::CloudT& points, Place2dNodeAttributes& attrs) {
  Place2d::CentroidT pcl_centroid;
  addBoundaryInfo(points,
                  attrs.pcl_mesh_connections,
                  pcl_centroid,
                  attrs.boundary,
                  attrs.pcl_boundary_connections);
  pcl::PointXYZ centroid;
  pcl_centroid.get(centroid);
  attrs.position << centroid.x, centroid.y, centroid.z;
  attrs.ellipse_centroid(2) = centroid.z;
}

std::pair<Place2d, Place2d> splitPlace(const Place2d::CloudT& points,
                                       const Place2d& place,
                                       const double connection_ellipse_scale_factor) {
  Place2d new_place_1;
  Place2d new_place_2;

  size_t min_ix_1 = SIZE_MAX;
  size_t max_ix_1 = 0;
  size_t min_ix_2 = SIZE_MAX;
  size_t max_ix_2 = 0;

  for (auto midx : place.indices) {
    Eigen::Vector2d pt(points[midx].x(), points[midx].y());
    double side = (pt - place.ellipse_centroid).dot(place.cut_plane);
    if (side >= 0) {
      new_place_1.indices.push_back(midx);
      min_ix_1 = std::min(min_ix_1, midx);
      max_ix_1 = std::max(max_ix_1, midx);
    }
    if (side <= 0) {
      new_place_2.indices.push_back(midx);
      min_ix_2 = std::min(min_ix_2, midx);
      max_ix_2 = std::max(max_ix_2, midx);
    }
  }

  new_place_1.min_mesh_index = min_ix_1;
  new_place_1.max_mesh_index = max_ix_1;

  new_place_2.min_mesh_index = min_ix_2;
  new_place_2.max_mesh_index = max_ix_2;

  addRectInfo(points, connection_ellipse_scale_factor, new_place_1);
  addRectInfo(points, connection_ellipse_scale_factor, new_place_2);

  return std::pair(new_place_1, new_place_2);
}

std::vector<Place2d> decomposePlaces(const Place2d::CloudT& cloud,
                                     const std::vector<Place2d>& initial_places,
                                     double min_size,
                                     size_t min_points,
                                     const double connection_ellipse_scale_factor) {
  std::vector<Place2d> final_places;
  for (auto p : initial_places) {
    // Recursively decompose initial place into smaller places
    std::vector<Place2d> sub_places =
        decomposePlace(cloud, p, min_size, min_points, connection_ellipse_scale_factor);

    for (Place2d sp : sub_places) {
      addBoundaryInfo(cloud, sp);
      final_places.push_back(sp);
    }
  }

  return final_places;
}

std::vector<Place2d> decomposePlace(const Place2d::CloudT& cloud_pts,
                                    const Place2d& place,
                                    const double min_size,
                                    const size_t min_points,
                                    const double connection_ellipse_scale_factor) {
  std::pair<Place2d, Place2d> children =
      splitPlace(cloud_pts, place, connection_ellipse_scale_factor);

  std::vector<Place2d> descendants;
  if (children.first.indices.size() > min_points &&
      children.first.cut_plane.norm() > min_size) {
    descendants = decomposePlace(cloud_pts,
                                 children.first,
                                 min_size,
                                 min_points,
                                 connection_ellipse_scale_factor);
  } else {
    descendants.push_back(children.first);
  }

  if (children.second.indices.size() > min_points &&
      children.second.cut_plane.norm() > min_size) {
    std::vector<Place2d> temp = decomposePlace(cloud_pts,
                                               children.second,
                                               min_size,
                                               min_points,
                                               connection_ellipse_scale_factor);
    descendants.insert(descendants.end(), temp.begin(), temp.end());
  } else {
    descendants.push_back(children.second);
  }

  return descendants;
}

bool shouldAddPlaceConnection(const Place2dNodeAttributes& attrs1,
                              const Place2dNodeAttributes& attrs2,
                              const double place_overlap_threshold,
                              const double place_max_neighbor_z_diff,
                              EdgeAttributes& edge_attrs) {
  double overlap_distance =
      ellipse::getEllipsoidTransverseOverlapDistance(attrs1.ellipse_matrix_compress,
                                                     attrs1.ellipse_centroid.head(2),
                                                     attrs2.ellipse_matrix_compress,
                                                     attrs2.ellipse_centroid.head(2));
  double centroid_height_offset = std::abs(attrs1.position(2) - attrs2.position(2));
  edge_attrs.weight = overlap_distance;
  edge_attrs.weighted = true;
  if (overlap_distance > place_overlap_threshold &&
      centroid_height_offset < place_max_neighbor_z_diff) {
    return true;
  } else {
    return false;
  }
}

bool shouldAddPlaceConnection(const Place2d& p1,
                              const Place2d& p2,
                              const double place_overlap_threshold,
                              const double place_max_neighbor_z_diff,
                              double& weight) {
  double overlap_distance =
      ellipse::getEllipsoidTransverseOverlapDistance(p1.ellipse_matrix_expand,
                                                     p1.ellipse_centroid,
                                                     p2.ellipse_matrix_expand,
                                                     p2.ellipse_centroid);

  pcl::PointXYZ centroid;
  p1.centroid.get(centroid);
  double p1z = centroid.z;
  p2.centroid.get(centroid);
  double p2z = centroid.z;
  double centroid_height_offset = std::abs(p1z - p2z);
  weight = overlap_distance;
  if (overlap_distance > place_overlap_threshold &&
      centroid_height_offset < place_max_neighbor_z_diff) {
    return true;
  } else {
    return false;
  }
}

}  // namespace hydra
