#pragma once

#include <pose_graph_tools/pose_graph.h>

#include <map>

#include "hydra_multi/operators/interface_operator.h"

namespace hydra_multi {

class PoseGraphOperator : public InterfaceOperator<pose_graph_tools::PoseGraph,
                                                   pose_graph_tools::PoseGraph> {
 public:
  using Ptr = std::shared_ptr<PoseGraphOperator>;

  using Connections = std::map<uint64_t, std::pair<uint64_t, Eigen::Affine3d>>;
  using HashedNodes = std::map<uint64_t, pose_graph_tools::PoseGraphNode>;

  using InterfaceOperator<pose_graph_tools::PoseGraph,
                          pose_graph_tools::PoseGraph>::InterfaceOperator;

  virtual ~PoseGraphOperator() = default;

  bool incrementalAppend(
      const pose_graph_tools::PoseGraph& incremental_source) override;

 protected:
  bool update(const pose_graph_tools::PoseGraph& source) override;

  bool rebase(const pose_graph_tools::PoseGraph& source) override;

  bool merge(const pose_graph_tools::PoseGraph& source) override;

 protected:
  Connections connections_;
  HashedNodes tracked_nodes_;
};

}  // namespace hydra_multi
