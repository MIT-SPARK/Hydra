#pragma once

#include "hydra_multi/common/types.h"
#include "hydra_multi/operators/interface_operator.h"

namespace hydra_multi {

class SceneGraphOperator
    : public InterfaceOperator<spark_dsg::SceneGraph, SceneGraphDelta> {
 public:
  using Ptr = std::shared_ptr<SceneGraphOperator>;

  using InterfaceOperator<spark_dsg::SceneGraph, SceneGraphDelta>::InterfaceOperator;
  virtual ~SceneGraphOperator() = default;

  bool incrementalAppend(const SceneGraphDelta& incremental_source) override;

 protected:
  bool update(const spark_dsg::SceneGraph& source) override;

  bool rebase(const spark_dsg::SceneGraph& source) override;

  bool merge(const spark_dsg::SceneGraph& source) override;

 private:
  Eigen::Isometry3d computeSourceDataTransform(const spark_dsg::SceneGraph& source);
  void updateAppendTransform(const spark_dsg::SceneGraph& source);

  // This does not have to be very precise: just not disjoint
  Eigen::Isometry3d append_T_data_ = Eigen::Isometry3d::Identity();
};
}  // namespace hydra_multi
