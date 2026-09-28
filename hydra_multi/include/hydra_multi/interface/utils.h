#pragma once
#include <Eigen/Dense>
#include <vector>

namespace hydra_multi {
void estimateRigidTransformSVD(const std::vector<Eigen::Vector3d>& src,
                               const std::vector<Eigen::Vector3d>& trgt,
                               Eigen::Isometry3d& src_T_trgt);

}  // namespace hydra_multi
