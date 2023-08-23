#include "hydra/frontend/place_2d_ellipsoid_math.h"

// gets called to find intersection between perpendicular line and ellipses
std::pair<Eigen::Vector2f, Eigen::Vector2f> intersect_line_ellipse_base(
    Eigen::Matrix<float, 2, 2> A, Eigen::Vector2f a, Eigen::Vector2f m, double d) {
  double eps = .001;
  bool flip_result = false;
  if (std::abs(m[1]) < eps) {
    flip_result = true;
    auto A_temp = A;
    A(0, 0) = A_temp(1, 1);
    A(1, 1) = A_temp(0, 0);
    a = a.reverse();
    m = m.reverse();
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

  Eigen::Vector2f p1(x1 + a[0], y1 + a[1]);
  Eigen::Vector2f p2(x2 + a[0], y2 + a[1]);

  if (flip_result) {
    std::pair<Eigen::Vector2f, Eigen::Vector2f> ret(p1.reverse(), p2.reverse());
    return ret;
  } else {
    std::pair<Eigen::Vector2f, Eigen::Vector2f> ret(p1, p2);
    return ret;
  }
}

std::pair<Eigen::Vector2f, Eigen::Vector2f> compute_min_transverse_intersection_points(
    const Eigen::Matrix<float, 2, 2>& A,
    const Eigen::Vector2f& a,
    const Eigen::Matrix<float, 2, 2>& B,
    const Eigen::Vector2f& b,
    const Eigen::Vector2f& m,
    const Eigen::Vector2f& dm) {
  std::pair<Eigen::Vector2f, Eigen::Vector2f> intersections_a =
      intersect_line_ellipse_base(A, a, dm, -dm.transpose() * m);
  std::pair<Eigen::Vector2f, Eigen::Vector2f> intersections_b =
      intersect_line_ellipse_base(B, b, dm, -dm.transpose() * m);

  Eigen::Vector2f p1 = intersections_a.first;
  Eigen::Vector2f p2 = intersections_a.second;
  Eigen::Vector2f p3 = intersections_b.first;
  Eigen::Vector2f p4 = intersections_b.second;

  Eigen::Vector2f perp = dm.reverse();
  perp(0) *= -1;
  float p1d = ((p1 - m).transpose() * perp);
  float p2d = ((p2 - m).transpose() * perp);
  float p3d = ((p3 - m).transpose() * perp);
  float p4d = ((p4 - m).transpose() * perp);

  Eigen::Vector2f left1, right1, left2, right2;
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

  Eigen::Vector2f pr = std::abs(pr1d) < std::abs(pr2d) ? right1 : right2;
  Eigen::Vector2f pl = std::abs(pl1d) < std::abs(pl2d) ? left1 : left2;

  return std::pair<Eigen::Vector2f, Eigen::Vector2f>(pr, pl);
}

Eigen::Vector2f compute_m_lambda(const Eigen::Matrix<float, 2, 2>& A,
                                 const Eigen::Vector2f& a,
                                 const Eigen::Matrix<float, 2, 2>& B,
                                 const Eigen::Vector2f& b,
                                 const double lambda) {
  // Point interpolated between ellipses A and B. lambda = 0 equals b, lambda = 1 equals
  // a
  Eigen::Matrix<float, 2, 2> e_inv = (lambda * A + (1 - lambda) * B).inverse();
  return e_inv * (lambda * A * a + (1 - lambda) * B * b);
}

Eigen::Vector2f compute_dm_lambda(const Eigen::Matrix<float, 2, 2>& A,
                                  const Eigen::Vector2f& a,
                                  const Eigen::Matrix<float, 2, 2>& B,
                                  const Eigen::Vector2f& b,
                                  const double lambda) {
  // Gradient of m_lambda curve
  Eigen::Matrix<float, 2, 2> e_inv = (lambda * A + (1 - lambda) * B).inverse();
  return -e_inv * (A - B) * e_inv * (lambda * A * a + (1 - lambda) * B * b) +
         e_inv * (A * a - B * b);
}

bool find_intersection_center_points(const Eigen::Matrix<float, 2, 2>& A,
                                     const Eigen::Vector2f& a,
                                     const Eigen::Matrix<float, 2, 2>& B,
                                     const Eigen::Vector2f& b,
                                     double& lambda_A,
                                     double& lambda_B) {
  double lower_a = 0;
  double upper_a = 1;
  double lower_b = 0;
  double upper_b = 1;

  bool found_a = false;
  bool found_b = false;

  while (!found_a && !found_b) {
    double lam;
    if (!found_a) {
      lam = (lower_a + upper_a) / 2;
    } else {
      lam = (lower_b + upper_b) / 2;
    }
    Eigen::Vector2f m = compute_m_lambda(A, a, B, b, lam);

    Eigen::Vector2f ra = m - a;
    double da = ra.transpose() * A * ra;
    Eigen::Vector2f rb = m - b;
    double db = rb.transpose() * B * rb;

    bool in_a = da < 1;
    bool in_b = db < 1;
    if (!in_a && !in_b) {
      return false;
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

  lambda_A = (lower_a + upper_a) / 2.;
  lambda_B = (lower_b + upper_b) / 2.;
  return true;
}

double get_ellipsoid_transverse_overlap_distance(Eigen::Matrix<float, 2, 2> A,
                                                 Eigen::Vector2f a,
                                                 Eigen::Matrix<float, 2, 2> B,
                                                 Eigen::Vector2f b) {
  double lam_a, lam_b;
  if (find_intersection_center_points(A, a, B, b, lam_a, lam_b)) {
    Eigen::Vector2f m = compute_m_lambda(A, a, B, b, (lam_a + lam_b) / 2);
    Eigen::Vector2f dm = compute_dm_lambda(A, a, B, b, (lam_a + lam_b) / 2);
    std::pair<Eigen::Vector2f, Eigen::Vector2f> min_points =
        compute_min_transverse_intersection_points(A, a, B, b, m, dm);
    return (min_points.first - min_points.second).norm();
  } else {
    return 0;
  }
}
