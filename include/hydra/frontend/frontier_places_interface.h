#pragma once
#include <hydra/utils/nearest_neighbor_utilities.h>

#include "hydra/common/dsg_types.h"
#include "hydra/reconstruction/reconstruction_output.h"

namespace hydra {

class FrontierPlacesInterface {
 public:
  virtual ~FrontierPlacesInterface() = default;
  virtual void updateRecentBlocks(Eigen::Vector3d current_position,
                                  double block_size) = 0;
  virtual void detectFrontiers(const ReconstructionOutput& input,
                               DynamicSceneGraph& graph,
                               NearestNodeFinder& finder) = 0;
  virtual void addFrontiers(uint64_t timestamp_ns,
                            DynamicSceneGraph& graph,
                            NearestNodeFinder& finder) = 0;

  std::vector<NodeId> archived_places_;
};

}  // namespace hydra
