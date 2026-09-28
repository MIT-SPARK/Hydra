#pragma once

#include <pose_graph_tools/pose_graph.h>

#include <map>
#include <set>

#include "hydra_multi/operators/interface_operator.h"

namespace hydra_multi {

class DeformationGraphOperator : public InterfaceOperator<pose_graph_tools::PoseGraph,
                                                          pose_graph_tools::PoseGraph> {
 public:
  using Ptr = std::shared_ptr<DeformationGraphOperator>;

  using Connections = std::map<std::pair<uint64_t, uint64_t>, Eigen::Affine3d>;
  using HashedNodes = std::map<uint64_t, pose_graph_tools::PoseGraphNode>;

  using InterfaceOperator<pose_graph_tools::PoseGraph,
                          pose_graph_tools::PoseGraph>::InterfaceOperator;

  virtual ~DeformationGraphOperator() = default;

  bool incrementalAppend(const pose_graph_tools::PoseGraph& source) override;

  bool verbose = false;

 protected:
  bool update(const pose_graph_tools::PoseGraph& source) override;

  bool rebase(const pose_graph_tools::PoseGraph& source) override;

  bool merge(const pose_graph_tools::PoseGraph& source) override;

  void processEdge(const pose_graph_tools::PoseGraphEdge& edge);

  bool nodeDisconnected(uint64_t id) const;

 protected:
  Connections connections_;
  HashedNodes tracked_nodes_;
  std::map<uint64_t, std::set<uint64_t>> adjacency_;
};

}  // namespace hydra_multi
