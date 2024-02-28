#include <Eigen/Dense>
#include <utility>

namespace hydra::ellipse {

std::pair<Eigen::Vector2d, Eigen::Vector2d> intersectLineEllipseBase(
    const Eigen::Matrix2d& A,
    const Eigen::Vector2d& a,
    const Eigen::Vector2d& m,
    double d);

std::pair<Eigen::Vector2d, Eigen::Vector2d> computeMinTransverseIntersectionPoints(
    const Eigen::Matrix2d& A,
    const Eigen::Vector2d& a,
    const Eigen::Matrix2d& B,
    const Eigen::Vector2d& b,
    const Eigen::Vector2d& m,
    const Eigen::Vector2d& dm);

Eigen::Vector2d computeMLambda(const Eigen::Matrix2d& A,
                                 const Eigen::Vector2d& a,
                                 const Eigen::Matrix2d& B,
                                 const Eigen::Vector2d& b,
                                 const double lambda);

Eigen::Vector2d computeDmLambda(const Eigen::Matrix2d& A,
                                  const Eigen::Vector2d& a,
                                  const Eigen::Matrix2d& B,
                                  const Eigen::Vector2d& b,
                                  const double lambda);

struct IntersectionCenters {
  bool certified = false;
  double lambda_A = 0.0;
  double lambda_B = 0.0;
};

void findIntersectionCenterPoints(const Eigen::Matrix2d& A,
                                     const Eigen::Vector2d& a,
                                     const Eigen::Matrix2d& B,
                                     const Eigen::Vector2d& b,
                                     IntersectionCenters& intersection_centers);

double getEllipsoidTransverseOverlapDistance(const Eigen::Matrix2d& A,
                                                 const Eigen::Vector2d& a,
                                                 const Eigen::Matrix2d& B,
                                                 const Eigen::Vector2d& b);

}  // namespace hydra::ellipse
