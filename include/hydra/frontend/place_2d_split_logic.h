#pragma once
#include <pcl/common/centroid.h>
#include <pcl/pcl_base.h>
#include <pcl/point_types.h>

#include "hydra/common/dsg_types.h"
#include "spark_dsg/dynamic_scene_graph.h"
#include "spark_dsg/node_attributes.h"

namespace hydra {

struct Place2d {
  // using PointT = pcl::PointXYZRGBA;
  using PointT = spark_dsg::Mesh::Pos;
  using CloudT = spark_dsg::Mesh::Positions;
  using CentroidT = pcl::CentroidPoint<pcl::PointXYZ>;
  using Index = size_t;
  CentroidT centroid;
  std::vector<Index> indices;
  size_t min_mesh_index;
  size_t max_mesh_index;
  std::vector<Index> boundary_indices;
  std::vector<Eigen::Vector3d> boundary;
  Eigen::Matrix2d ellipse_matrix_compress;
  Eigen::Matrix2d ellipse_matrix_expand;
  Eigen::Vector2d ellipse_centroid;
  Eigen::Vector2d cut_plane;
  bool can_split;
};

void addRectInfo(const Place2d::CloudT& points,
                 const double connection_ellipse_scale_factor,
                 Place2dNodeAttributes& attrs);

void addRectInfo(const Place2d::CloudT& points,
                 const double connection_ellipse_scale_factor,
                 Place2d& place);

void addBoundaryInfo(const Place2d::CloudT& points, Place2d& place);

void addBoundaryInfo(const Place2d::CloudT& points, Place2dNodeAttributes& attrs);

std::pair<Place2d, Place2d> splitPlace(const Place2d::CloudT& points,
                                       const Place2d& place,
                                       const double connection_ellipse_scale_factor);

std::vector<Place2d> decomposePlaces(const Place2d::CloudT& cloud,
                                     const std::vector<Place2d>& initial_places,
                                     double min_size,
                                     size_t min_points,
                                     const double connection_ellipse_scale_factor);

std::vector<Place2d> decomposePlace(const Place2d::CloudT& cloud_pts,
                                    const Place2d& place,
                                    const double min_size,
                                    const size_t min_points,
                                    const double connection_ellipse_scale_factor);

bool shouldAddPlaceConnection(const Place2dNodeAttributes& attrs1,
                              const Place2dNodeAttributes& attrs2,
                              const double place_overlap_threshold,
                              const double place_max_neighbor_z_diff,
                              EdgeAttributes& edge_attrs);

bool shouldAddPlaceConnection(const Place2d& p1,
                              const Place2d& p2,
                              const double place_overlap_threshold,
                              const double place_max_neighbor_z_diff,
                              double& weight);

}  // namespace hydra
