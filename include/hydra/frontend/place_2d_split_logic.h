#pragma once
#include <spark_dsg/edge_attributes.h>
#include <spark_dsg/node_attributes.h>

namespace kimera_pgmo {
class MeshDelta;
struct MeshOffsetInfo;
}  // namespace kimera_pgmo

namespace hydra {

struct Place2d {
  Eigen::Vector3f centroid = Eigen::Vector3f::Zero();
  std::vector<size_t> indices;
  size_t min_mesh_index = std::numeric_limits<size_t>::max();
  size_t max_mesh_index = 0;
  std::vector<size_t> boundary_indices;
  std::vector<Eigen::Vector3d> boundary;
  Eigen::Matrix2d ellipse_matrix_compress = Eigen::Matrix2d::Zero();
  Eigen::Matrix2d ellipse_matrix_expand = Eigen::Matrix2d::Zero();
  Eigen::Vector2d ellipse_centroid = Eigen::Vector2d::Zero();
  Eigen::Vector2d cut_plane = Eigen::Vector2d::Zero();
  bool can_split = false;

  void fillAttributes(spark_dsg::Place2dNodeAttributes& attrs) const;
  void updateIndexBounds();
};

void addRectInfo(const kimera_pgmo::MeshDelta& delta,
                 const kimera_pgmo::MeshOffsetInfo& offsets,
                 const double connection_ellipse_scale_factor,
                 Place2d& place);

// TODO(nathan) drop this when rebased on active dsg
void addRectInfo(const spark_dsg::Mesh& mesh,
                 const double connection_ellipse_scale_factor,
                 Place2d& place);

void addRectInfo(const spark_dsg::Mesh& mesh,
                 const double connection_ellipse_scale_factor,
                 spark_dsg::Place2dNodeAttributes& attrs);

void addBoundaryInfo(const kimera_pgmo::MeshDelta& delta,
                     const kimera_pgmo::MeshOffsetInfo& offsets,
                     Place2d& place);

// TODO(nathan) drop this when rebased on active dsg
void addBoundaryInfo(const spark_dsg::Mesh& mesh, Place2d& place);

void addBoundaryInfo(const spark_dsg::Mesh& mesh,
                     spark_dsg::Place2dNodeAttributes& attrs);

void decomposePlace(const kimera_pgmo::MeshDelta& delta,
                    const kimera_pgmo::MeshOffsetInfo& offset,
                    const Place2d& place,
                    const double min_size,
                    const size_t min_points,
                    const double connection_ellipse_scale_factor,
                    std::vector<Place2d>& descendants);

bool shouldAddPlaceConnection(const spark_dsg::Place2dNodeAttributes& attrs1,
                              const spark_dsg::Place2dNodeAttributes& attrs2,
                              const double place_overlap_threshold,
                              const double place_max_neighbor_z_diff,
                              spark_dsg::EdgeAttributes& edge_attrs);

void remapPlace2dMesh(spark_dsg::Place2dNodeAttributes& attrs,
                      const kimera_pgmo::MeshOffsetInfo& offsets);

}  // namespace hydra
