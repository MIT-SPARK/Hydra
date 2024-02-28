#include "hydra/utils/place_2d_ellipsoid_math.h"

namespace hydra::ellipse {

// gets called to find intersection between perpendicular line and ellipses
std::pair<Eigen::Vector2d, Eigen::Vector2d> intersectLineEllipseBase(
    const Eigen::Matrix2d& A_in,
    const Eigen::Vector2d& a_in,
    const Eigen::Vector2d& m_in,
    double d) {
  Eigen::Matrix2d A = A_in;
  Eigen::Vector2d a = a_in;
  Eigen::Vector2d m = m_in;
  double eps = .001;
  bool flip_result = false;
  if (std::abs(m[1]) < eps) {
    flip_result = true;
    auto A_temp = A;
    A(0, 0) = A_temp(1, 1);
    A(1, 1) = A_temp(0, 0);
    a = a.reverse().eval();
    m = m.reverse().eval();
  }
  double c1 = -m[0] / m[1];
  double c2 = -m[0] / m[1] * a[0] - d / m[1] - a[1];
  double m11 = A(0, 0);
  double m12 = A(0, 1);
  double m22 = A(1, 1);

  double qa = m11 + 2 * m12 * c1 + std::pow(c1, 2) * m22;
  double qb = 2 * m12 * c2 + 2 * c1 * m22 * c2;
  double qc = std::pow(c2, 2) * m22 - 1;

  double disc = std::sqrt(std::pow(qb, 2) - 4 * qa * qc);

  double x1 = (-qb + disc) / (2 * qa);
  double x2 = (-qb - disc) / (2 * qa);

  double y1 = -m[0] / m[1] * (x1 + a[0]) - d / m[1] - a[1];
  double y2 = -m[0] / m[1] * (x2 + a[0]) - d / m[1] - a[1];

  Eigen::Vector2d p1(x1 + a[0], y1 + a[1]);
  Eigen::Vector2d p2(x2 + a[0], y2 + a[1]);

  if (flip_result) {
    std::pair<Eigen::Vector2d, Eigen::Vector2d> ret(p1.reverse(), p2.reverse());
    return ret;
  } else {
    std::pair<Eigen::Vector2d, Eigen::Vector2d> ret(p1, p2);
    return ret;
  }
}

std::pair<Eigen::Vector2d, Eigen::Vector2d> computeMinTransverseIntersectionPoints(
    const Eigen::Matrix2d& A,
    const Eigen::Vector2d& a,
    const Eigen::Matrix2d& B,
    const Eigen::Vector2d& b,
    const Eigen::Vector2d& m,
    const Eigen::Vector2d& dm) {
  std::pair<Eigen::Vector2d, Eigen::Vector2d> intersections_a =
      intersectLineEllipseBase(A, a, dm, -dm.transpose() * m);
  std::pair<Eigen::Vector2d, Eigen::Vector2d> intersections_b =
      intersectLineEllipseBase(B, b, dm, -dm.transpose() * m);

  Eigen::Vector2d p1 = intersections_a.first;
  Eigen::Vector2d p2 = intersections_a.second;
  Eigen::Vector2d p3 = intersections_b.first;
  Eigen::Vector2d p4 = intersections_b.second;

  Eigen::Vector2d perp = dm.reverse();
  perp(0) *= -1;
  double p1d = ((p1 - m).transpose() * perp);
  double p2d = ((p2 - m).transpose() * perp);
  double p3d = ((p3 - m).transpose() * perp);
  double p4d = ((p4 - m).transpose() * perp);

  Eigen::Vector2d left1, right1, left2, right2;
  double pl1d = 0;
  double pr1d = 0;
  double pl2d = 0;
  double pr2d = 0;
  if (p1d > 0) {
    left1 = p1;
    pl1d = p1d;
    right1 = p2;
    pr1d = p2d;
  } else {
    left1 = p2;
    pl1d = p2d;
    right1 = p1;
    pr1d = p1d;
  }

  if (p3d > 0) {
    left2 = p3;
    pl2d = p3d;
    right2 = p4;
    pr2d = p4d;
  } else {
    left2 = p4;
    pl2d = p4d;
    right2 = p3;
    pr2d = p3d;
  }

  Eigen::Vector2d pr = std::abs(pr1d) < std::abs(pr2d) ? right1 : right2;
  Eigen::Vector2d pl = std::abs(pl1d) < std::abs(pl2d) ? left1 : left2;

  return std::pair<Eigen::Vector2d, Eigen::Vector2d>(pr, pl);
}

Eigen::Vector2d computeMLambda(const Eigen::Matrix2d& A,
                                 const Eigen::Vector2d& a,
                                 const Eigen::Matrix2d& B,
                                 const Eigen::Vector2d& b,
                                 const double lambda) {
  // Point interpolated between ellipses A and B. lambda = 0 equals b, lambda = 1 equals
  // a
  Eigen::Matrix2d e_inv = (lambda * A + (1 - lambda) * B).inverse();
  return e_inv * (lambda * A * a + (1 - lambda) * B * b);
}

Eigen::Vector2d computeDmLambda(const Eigen::Matrix2d& A,
                                  const Eigen::Vector2d& a,
                                  const Eigen::Matrix2d& B,
                                  const Eigen::Vector2d& b,
                                  const double lambda) {
  // Gradient of m_lambda curve
  Eigen::Matrix2d e_inv = (lambda * A + (1 - lambda) * B).inverse();
  return -e_inv * (A - B) * e_inv * (lambda * A * a + (1 - lambda) * B * b) +
         e_inv * (A * a - B * b);
}

IntersectionCenters findIntersectionCenterPoints(const Eigen::Matrix2d& A,
                                                    const Eigen::Vector2d& a,
                                                    const Eigen::Matrix2d& B,
                                                    const Eigen::Vector2d& b) {
  IntersectionCenters intersection_centers;
  double lower_a = 0;
  double upper_a = 1;
  double lower_b = 0;
  double upper_b = 1;

  bool found_a = false;
  bool found_b = false;

  bool proven_intersection = false;

  while (!found_a || !found_b) {
    double lam;
    if (!found_a) {
      lam = (lower_a + upper_a) / 2;
    } else {
      lam = (lower_b + upper_b) / 2;
    }
    Eigen::Vector2d m = computeMLambda(A, a, B, b, lam);

    Eigen::Vector2d ra = m - a;
    double da = ra.transpose() * A * ra;
    Eigen::Vector2d rb = m - b;
    double db = rb.transpose() * B * rb;

    bool in_a = da < 1;
    bool in_b = db < 1;
    if (!in_a && !in_b) {
      intersection_centers.certified = false;
      return intersection_centers;
    } else if (in_a && in_b) {
      proven_intersection = true;
    }

    if (!found_a) {
      if (in_a) {
        upper_a = lam;
      } else {
        lower_a = lam;
      }
      if (upper_a - lower_a < 0.01) {
        found_a = true;
      }
    }

    if (in_b) {
      lower_b = std::max(lower_b, lam);
    } else {
      upper_b = std::min(upper_b, lam);
    }

    if (upper_b - lower_b < 0.01) {
      found_b = true;
    }
  }

  intersection_centers.certified = proven_intersection;
  intersection_centers.lambda_A = (lower_a + upper_a) / 2.;
  intersection_centers.lambda_B = (lower_b + upper_b) / 2.;
  return intersection_centers;
}

double getEllipsoidTransverseOverlapDistance(const Eigen::Matrix2d& A,
                                                 const Eigen::Vector2d& a,
                                                 const Eigen::Matrix2d& B,
                                                 const Eigen::Vector2d& b) {
  IntersectionCenters centers = findIntersectionCenterPoints(A, a, B, b);
  if (centers.certified) {
    Eigen::Vector2d m =
        computeMLambda(A, a, B, b, (centers.lambda_A + centers.lambda_B) / 2);
    Eigen::Vector2d dm =
        computeDmLambda(A, a, B, b, (centers.lambda_A + centers.lambda_B) / 2);
    std::pair<Eigen::Vector2d, Eigen::Vector2d> min_points =
        computeMinTransverseIntersectionPoints(A, a, B, b, m, dm);
    return (min_points.first - min_points.second).norm();
  } else {
    return 0;
  }
}

}  // namespace hydra::ellipse
