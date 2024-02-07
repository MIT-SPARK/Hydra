#pragma once
#include <hydra/utils/nearest_neighbor_utilities.h>

#include "hydra/common/dsg_types.h"
#include "hydra/reconstruction/reconstruction_output.h"

namespace hydra {

class FrontierExtractor {
 public:
  FrontierExtractor();
  void detectFrontiers(const ReconstructionOutput& input);
  void addFrontiers(uint64_t timestamp_ns,
                    DynamicSceneGraph& graph,
                    NearestNodeFinder& finder);

 private:
  NodeSymbol next_node_id_;
  std::vector<Eigen::Vector3d> frontier_positions_;
  std::vector<double> frontier_distances_;
};

}  // namespace hydra
