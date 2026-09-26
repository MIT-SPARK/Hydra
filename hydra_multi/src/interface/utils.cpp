#include "hydra_multi/interface/utils.h"

#include <glog/logging.h>

namespace hydra_multi {
using namespace Eigen;
void estimateRigidTransformSVD(const std::vector<Vector3d>& src,
                               const std::vector<Vector3d>& trgt,
                               Eigen::Isometry3d& src_T_trgt) {
  CHECK_EQ(src.size(), trgt.size());
  CHECK_GE(src.size(), 3);

  // Compute the centroids
  const auto n = src.size();
  Vector3d centroid_src = Vector3d::Zero();
  Vector3d centroid_trgt = Vector3d::Zero();

  for (size_t i = 0; i < n; ++i) {
    centroid_src += src[i];
    centroid_trgt += trgt[i];
  }

  centroid_src /= src.size();
  centroid_trgt /= trgt.size();

  // Center the points by subtracting the centroids
  MatrixXd src_centered(n, 3);
  MatrixXd trgt_centered(n, 3);

  for (size_t i = 0; i < n; ++i) {
    src_centered.row(i) = src[i] - centroid_src;
    trgt_centered.row(i) = trgt[i] - centroid_trgt;
  }

  // Compute the covariance matrix
  Matrix3d H = src_centered.transpose() * trgt_centered;

  // Perform SVD on the covariance matrix
  JacobiSVD<MatrixXd> svd(H, ComputeFullU | ComputeFullV);
  Matrix3d R = svd.matrixU() * svd.matrixV().transpose();

  if (R.determinant() < 0) {
    // If the determinant is negative, flip the sign of the last column of U
    Matrix3d U = svd.matrixU();
    U.col(U.cols() - 1) *= -1;
    R = U * svd.matrixV().transpose();
  }

  Vector3d t = centroid_trgt - R * centroid_src;

  src_T_trgt.linear() = R;
  src_T_trgt.translation() = t;
}

}  // namespace hydra_multi
