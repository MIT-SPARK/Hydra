#include <kimera_topology/voxblox_types.h>

namespace kimera {
namespace topology {

std::ostream& operator<<(std::ostream& out, const GvdVoxel& voxel) {
  out << "GvdVoxel<flags=";
  out << (voxel.observed ? 'y' : 'n');
  out << (voxel.in_queue ? 'y' : 'n');
  out << (voxel.fixed ? 'y' : 'n');
  out << (voxel.is_voronoi ? 'y' : 'n');
  out << (voxel.is_voronoi_parent ? 'y' : 'n');
  out << ", distance=" << voxel.distance << " -> ";
  if (voxel.has_parent) {
    out << Eigen::Map<const GlobalIndex>(voxel.parent).transpose();
  } else {
    out << "unknown";
  }
  out << ">";
  return out;
}

}  // namespace topology
}  // namespace kimera
