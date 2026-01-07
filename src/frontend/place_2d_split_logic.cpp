#include "hydra/frontend/place_2d_split_logic.h"

#include <kimera_pgmo/mesh_delta.h>
#include <spark_dsg/dynamic_scene_graph.h>
#include <spark_dsg/node_attributes.h>

#include "hydra/utils/place_2d_ellipsoid_math.h"
#include "opencv2/imgproc.hpp"

namespace hydra {

using spark_dsg::EdgeAttributes;
using spark_dsg::Place2dNodeAttributes;

namespace {

struct PointAdaptor {
  virtual Eigen::Vector3f get(size_t i) const = 0;

  inline Eigen::Vector2d get2d(size_t i) const {
    return get(i).head<2>().cast<double>();
  }

  inline cv::Point2f getCv(size_t i) const {
    auto p = get(i);
    return {p.x(), p.y()};
  }
};

struct MeshAdaptor : public PointAdaptor {
  explicit MeshAdaptor(const spark_dsg::Mesh& mesh) : mesh(mesh) {}

  Eigen::Vector3f get(size_t i) const override { return mesh.points[i]; }

  const spark_dsg::Mesh& mesh;
};

struct DeltaAdaptor : public PointAdaptor {
  DeltaAdaptor(const kimera_pgmo::MeshDelta& delta,
               const kimera_pgmo::MeshOffsetInfo& offsets)
      : delta(delta), offsets(offsets) {}

  Eigen::Vector3f get(size_t i) const override {
    return delta.getVertex(offsets.toLocalVertex(i)).pos;
  }

  const kimera_pgmo::MeshDelta& delta;
  const kimera_pgmo::MeshOffsetInfo& offsets;
};

inline void addRectInfo(const PointAdaptor& points,
                        const std::vector<size_t> mindices,
                        const double connection_ellipse_scale_factor,
                        Eigen::Vector2d& ellipse_centroid,
                        Eigen::Matrix2d& m_expand,
                        Eigen::Matrix2d& m_compress,
                        Eigen::Vector2d& cut_plane) {
  std::vector<cv::Point2f> region;
  for (auto midx : mindices) {
    region.push_back(points.getCv(midx));
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

inline void addRectInfo(const PointAdaptor& points,
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

inline void addBoundaryInfo(const PointAdaptor& points,
                            const std::vector<size_t>& mindices,
                            Eigen::Vector3f& centroid,
                            std::vector<Eigen::Vector3d>& boundary,
                            std::vector<size_t>& boundary_mindices) {
  std::vector<cv::Point2f> region_pts;
  std::vector<size_t> region_to_cloud_index;
  centroid = Eigen::Vector3f::Zero();
  for (const auto midx : mindices) {
    const auto p = points.get(midx);
    region_to_cloud_index.push_back(midx);
    region_pts.push_back({p.x(), p.y()});
    centroid += p;
  }

  if (mindices.size()) {
    centroid /= mindices.size();
  }

  // compute convex hull for each place
  std::vector<int> ch;
  cv::convexHull(region_pts, ch);
  boundary.clear();
  boundary_mindices.clear();
  for (const auto pix : ch) {
    const auto cloud_ix = region_to_cloud_index.at(pix);
    boundary.push_back(points.get(cloud_ix).cast<double>());
    boundary_mindices.push_back(cloud_ix);
  }
}

inline void addBoundaryInfo(const PointAdaptor& points, Place2d& p) {
  addBoundaryInfo(points, p.indices, p.centroid, p.boundary, p.boundary_indices);
}

inline std::pair<Place2d, Place2d> splitPlace(const PointAdaptor& points,
                                              const Place2d& place,
                                              const double scale_factor) {
  Place2d new_place_1;
  Place2d new_place_2;
  for (auto midx : place.indices) {
    const auto pt = points.get2d(midx);
    double side = (pt - place.ellipse_centroid).dot(place.cut_plane);
    if (side >= 0) {
      new_place_1.indices.push_back(midx);
      new_place_1.min_mesh_index = std::min(new_place_1.min_mesh_index, midx);
      new_place_1.max_mesh_index = std::max(new_place_1.max_mesh_index, midx);
    }

    if (side <= 0) {
      new_place_2.indices.push_back(midx);
      new_place_2.min_mesh_index = std::min(new_place_2.min_mesh_index, midx);
      new_place_2.max_mesh_index = std::max(new_place_2.max_mesh_index, midx);
    }
  }

  addRectInfo(points, scale_factor, new_place_1);
  addRectInfo(points, scale_factor, new_place_2);
  return std::pair(new_place_1, new_place_2);
}

inline void decomposePlace(const PointAdaptor& points,
                           const Place2d& place,
                           const double min_size,
                           const size_t min_points,
                           const double scale_factor,
                           std::vector<Place2d>& descendants) {
  const auto [c1, c2] = splitPlace(points, place, scale_factor);
  if (c1.indices.size() > min_points && c1.cut_plane.norm() > min_size) {
    decomposePlace(points, c1, min_size, min_points, scale_factor, descendants);
  } else {
    descendants.push_back(c1);
    addBoundaryInfo(points, descendants.back());
  }

  if (c2.indices.size() > min_points && c2.cut_plane.norm() > min_size) {
    decomposePlace(points, c2, min_size, min_points, scale_factor, descendants);
  } else {
    descendants.push_back(c2);
    addBoundaryInfo(points, descendants.back());
  }
}

inline void remapPlace2dConnections(Place2dNodeAttributes& attrs,
                                    const kimera_pgmo::MeshOffsetInfo& offsets) {
  kimera_pgmo::MeshOffsetInfo::RemapStats info;
  auto& connections = attrs.pcl_mesh_connections;
  connections = offsets.remapVertexIndices(connections, &info);
  attrs.pcl_min_index = info.min_index;
  attrs.pcl_max_index = info.max_index;
  attrs.has_active_mesh_indices = !info.all_archived;
}

}  // namespace

void Place2d::updateIndexBounds() {
  min_mesh_index = std::numeric_limits<size_t>::max();
  max_mesh_index = 0;
  for (const auto idx : indices) {
    min_mesh_index = std::min(idx, min_mesh_index);
    max_mesh_index = std::max(idx, max_mesh_index);
  }
}

void addRectInfo(const kimera_pgmo::MeshDelta& delta,
                 const kimera_pgmo::MeshOffsetInfo& offsets,
                 const double connection_ellipse_scale_factor,
                 Place2d& place) {
  addRectInfo(DeltaAdaptor(delta, offsets), connection_ellipse_scale_factor, place);
}

void addRectInfo(const spark_dsg::Mesh& mesh,
                 const double connection_ellipse_scale_factor,
                 Place2d& place) {
  addRectInfo(MeshAdaptor(mesh), connection_ellipse_scale_factor, place);
}

void addRectInfo(const spark_dsg::Mesh& mesh,
                 const double connection_ellipse_scale_factor,
                 Place2dNodeAttributes& attrs) {
  Eigen::Vector2d cut_plane;         // place node attributes don't have cut_plane
  Eigen::Vector2d ellipse_centroid;  // attrs.ellipse_centroid is Vector3d
  addRectInfo(MeshAdaptor(mesh),
              attrs.pcl_mesh_connections,
              connection_ellipse_scale_factor,
              ellipse_centroid,
              attrs.ellipse_matrix_expand,
              attrs.ellipse_matrix_compress,
              cut_plane);
  attrs.ellipse_centroid(0) = ellipse_centroid(0);
  attrs.ellipse_centroid(1) = ellipse_centroid(1);
}

void addBoundaryInfo(const kimera_pgmo::MeshDelta& delta,
                     const kimera_pgmo::MeshOffsetInfo& offsets,
                     Place2d& p) {
  addBoundaryInfo(DeltaAdaptor(delta, offsets), p);
}

void addBoundaryInfo(const spark_dsg::Mesh& mesh, Place2d& p) {
  addBoundaryInfo(MeshAdaptor(mesh), p);
}

void addBoundaryInfo(const spark_dsg::Mesh& mesh, Place2dNodeAttributes& attrs) {
  Eigen::Vector3f centroid;
  addBoundaryInfo(MeshAdaptor(mesh),
                  attrs.pcl_mesh_connections,
                  centroid,
                  attrs.boundary,
                  attrs.pcl_boundary_connections);
  attrs.position = centroid.cast<double>();
  attrs.ellipse_centroid(2) = centroid.z();
}

void decomposePlace(const kimera_pgmo::MeshDelta& delta,
                    const kimera_pgmo::MeshOffsetInfo& offsets,
                    const Place2d& place,
                    const double min_size,
                    const size_t min_points,
                    const double scale_factor,
                    std::vector<Place2d>& descendants) {
  decomposePlace(DeltaAdaptor(delta, offsets),
                 place,
                 min_size,
                 min_points,
                 scale_factor,
                 descendants);
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

void remapPlace2dMesh(Place2dNodeAttributes& attrs,
                      const kimera_pgmo::MeshOffsetInfo& offsets) {
  remapPlace2dConnections(attrs, offsets);

  auto& indices = attrs.pcl_boundary_connections;
  kimera_pgmo::MeshOffsetInfo::RemapStats info;
  indices = offsets.remapVertexIndices(indices, &info);

  const auto prev_boundary = attrs.boundary;
  attrs.boundary.clear();
  attrs.boundary.reserve(prev_boundary.size());
  for (size_t i = 0; i < prev_boundary.size(); ++i) {
    if (info.deleted_indices.count(i)) {
      continue;
    }

    attrs.boundary.push_back(prev_boundary[i]);
  }
}

}  // namespace hydra
