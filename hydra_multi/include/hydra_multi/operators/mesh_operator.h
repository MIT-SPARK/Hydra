#pragma once

#include <map>

#include "hydra_multi/common/types.h"
#include "hydra_multi/operators/interface_operator.h"

namespace hydra_multi {

class MeshOperator : public InterfaceOperator<MeshData, MeshDelta> {
 public:
  using Ptr = std::shared_ptr<MeshOperator>;

  using InterfaceOperator<MeshData, MeshDelta>::InterfaceOperator;
  virtual ~MeshOperator() = default;

  bool incrementalAppend(const MeshDelta& incremental_source) override;

 protected:
  bool update(const MeshData& source) override;

  bool rebase(const MeshData& source) override;

  bool merge(const MeshData& source) override;

 private:
  void updateAppendTransform();

  // This does not have to be very precise: just not disjoint
  Eigen::Isometry3f data_T_orig_ = Eigen::Isometry3f::Identity();
};
}  // namespace hydra_multi
