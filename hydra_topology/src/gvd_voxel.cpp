#include "hydra_topology/gvd_voxel.h"

namespace hydra {
namespace topology {

std::ostream& operator<<(std::ostream& out, const GvdVoxel& voxel) {
  out << "GvdVoxel<flags=";
  out << (voxel.observed ? 'y' : 'n');
  out << (voxel.on_surface ? 'y' : 'n');
  out << (voxel.fixed ? 'y' : 'n');
  out << (voxel.in_queue ? 'y' : 'n');
  out << (voxel.is_voronoi_parent ? 'y' : 'n');
  out << ", distance=" << voxel.distance << " -> ";
  if (voxel.has_parent) {
    out << Eigen::Map<const GlobalIndex>(voxel.parent).transpose();
  } else {
    out << "unknown";
  }
  out << ", voronoi=";
  if (voxel.num_extra_basis) {
    out << "y (" << static_cast<int>(voxel.num_extra_basis) << ")";
  } else {
    out << "n";
  }
  if (voxel.is_voronoi_parent) {
    out << ", nearest_voronoi="
        << Eigen::Map<const GlobalIndex>(voxel.nearest_voronoi).transpose();
  }
  out << ">";
  return out;
}

}  // namespace topology
}  // namespace hydra
