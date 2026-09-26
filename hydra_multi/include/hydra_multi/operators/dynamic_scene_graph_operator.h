#pragma once

#include <map>

#include "hydra_multi/common/types.h"
#include "hydra_multi/operators/interface_operator.h"
namespace hydra_multi {

class DynamicSceneGraphOperator
    : public InterfaceOperator<DynamicSceneGraph, SceneGraphDelta> {
 public:
  using Ptr = std::shared_ptr<DynamicSceneGraphOperator>;

  using InterfaceOperator<DynamicSceneGraph, SceneGraphDelta>::InterfaceOperator;
  virtual ~DynamicSceneGraphOperator() = default;

  bool incrementalAppend(const SceneGraphDelta& incremental_source) override;

 protected:
  bool update(const DynamicSceneGraph& source) override;

  bool rebase(const DynamicSceneGraph& source) override;

  bool merge(const DynamicSceneGraph& source) override;

 private:
  Eigen::Isometry3d computeSourceDataTransform(const DynamicSceneGraph& source);
  void updateAppendTransform(const DynamicSceneGraph& source);

  // This does not have to be very precise: just not disjoint
  Eigen::Isometry3d append_T_data_ = Eigen::Isometry3d::Identity();
};
}  // namespace hydra_multi
