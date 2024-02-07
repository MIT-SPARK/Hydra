#include "hydra/frontend/frontier_extractor.h"

#include <hydra/utils/nearest_neighbor_utilities.h>

namespace hydra {

FrontierExtractor::FrontierExtractor() : next_node_id_('f', 0) {}

void FrontierExtractor::detectFrontiers(const ReconstructionOutput& input) {
  // 1. get list of x,y,z values of each frontier from archived vertices
  std::vector<Eigen::Vector3d> frontier_positions;
  std::vector<double> frontier_distances;
  for (const auto& idx : input.archived_blocks) {
    voxblox::Block<voxblox::TsdfVoxel>::Ptr block = input.tsdf->getBlockPtrByIndex(idx);
    for (size_t v = 0; v < block->num_voxels(); ++v) {
      const voxblox::TsdfVoxel& voxel = block->getVoxelByLinearIndex(v);
      if (voxel.weight <= 1e-6) {
        voxblox::Point center = block->computeCoordinatesFromLinearIndex(v);
        frontier_positions.push_back({center.x(), center.y(), center.z()});
        frontier_distances.push_back(voxel.distance);
      }
    }
  }
}

void FrontierExtractor::addFrontiers(uint64_t timestamp_ns,
                                     DynamicSceneGraph& graph,
                                     NearestNodeFinder& finder) {
  // 2. add a node in the place graph for each point

  for (size_t ix = 0; ix < frontier_positions_.size(); ++ix) {
    // finder->find(
    //    frontier_positions_.at(ix), 1, false, [&](NodeId place_id, size_t, double) {
    //      dsg_->graph->insertEdge(place_id, object_id);
    //    });

    double d = frontier_distances_.at(ix);
    PlaceNodeAttributes::Ptr attrs(new PlaceNodeAttributes(d, 0));
    attrs->position = frontier_positions_.at(ix);
    attrs->real_place = false;
    attrs->last_update_time_ns = timestamp_ns;
    graph.emplaceNode('f', next_node_id_, std::move(attrs));
    ++next_node_id_;
  }
}
}  // namespace hydra
