#include <Eigen/Dense>
#include <utility>

std::pair<Eigen::Vector2f, Eigen::Vector2f> intersect_line_ellipse_base(
    Eigen::Matrix<float, 2, 2> A, Eigen::Vector2f a, Eigen::Vector2f m, double d);

std::pair<Eigen::Vector2f, Eigen::Vector2f> compute_min_transverse_intersection_points(
    const Eigen::Matrix<float, 2, 2>& A,
    const Eigen::Vector2f& a,
    const Eigen::Matrix<float, 2, 2>& B,
    const Eigen::Vector2f& b,
    const Eigen::Vector2f& m,
    const Eigen::Vector2f& dm);

Eigen::Vector2f compute_m_lambda(const Eigen::Matrix<float, 2, 2>& A,
                                 const Eigen::Vector2f& a,
                                 const Eigen::Matrix<float, 2, 2>& B,
                                 const Eigen::Vector2f& b,
                                 const double lambda);

Eigen::Vector2f compute_dm_lambda(const Eigen::Matrix<float, 2, 2>& A,
                                  const Eigen::Vector2f& a,
                                  const Eigen::Matrix<float, 2, 2>& B,
                                  const Eigen::Vector2f& b,
                                  const double lambda);

bool find_intersection_center_points(const Eigen::Matrix<float, 2, 2>& A,
                                     const Eigen::Vector2f& a,
                                     const Eigen::Matrix<float, 2, 2>& B,
                                     const Eigen::Vector2f& b,
                                     double& lambda_A,
                                     double& lambda_B);

double get_ellipsoid_transverse_overlap_distance(Eigen::Matrix<float, 2, 2> A,
                                                 Eigen::Vector2f a,
                                                 Eigen::Matrix<float, 2, 2> B,
                                                 Eigen::Vector2f b);
