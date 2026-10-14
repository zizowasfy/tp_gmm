#pragma once
#include <Eigen/Dense>
#include <random>
#include <stdexcept>
#include <limits>
#include <vector>

namespace tp_gmm::sampling
{
struct Gaussian
{
  Eigen::Vector3d mean;
  Eigen::Matrix3d covariance, factor, axes;
  Eigen::Vector3d eigenvalues;
};
inline Gaussian canonicalGaussian(const Eigen::Vector3d& mean, const Eigen::Matrix3d& input, double floor)
{
  if (!mean.allFinite() || !input.allFinite() || !std::isfinite(floor) || floor <= 0)
    throw std::invalid_argument("Non-finite Gaussian or invalid covariance floor");
  const Eigen::Matrix3d symmetric = (0.5 * (input + input.transpose())).eval();
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(symmetric);
  if (eig.info() != Eigen::Success || !eig.eigenvalues().allFinite() || eig.eigenvalues().minCoeff() < -1e-10)
    throw std::invalid_argument("Covariance is not positive semidefinite");
  Gaussian g;
  g.mean = mean;
  g.eigenvalues = eig.eigenvalues().cwiseMax(floor);
  g.axes = eig.eigenvectors();
  if (g.axes.determinant() < 0) g.axes.col(0) *= -1;
  g.factor = g.axes * g.eigenvalues.cwiseSqrt().asDiagonal();
  g.covariance = g.factor * g.factor.transpose();
  return g;
}
inline bool truncatedNormal(std::mt19937& rng, std::normal_distribution<double>& normal,
                            double cutoff, Eigen::Vector3d& z)
{
  // Small historical corridors need efficient rejection from a uniform ball.
  // Accepting with exp(-||z||^2/2) gives the same conditional normal density.
  if (!std::isfinite(cutoff) || cutoff <= 0) return false;
  if (cutoff < 1)
  {
    std::uniform_real_distribution<double> uniform(0, 1);
    for (unsigned i = 0; i < 128; ++i)
    {
      for (int j = 0; j < 3; ++j) z[j] = 2 * uniform(rng) - 1;
      if (z.squaredNorm() > 1) continue;
      z *= cutoff;
      if (uniform(rng) <= std::exp(-0.5 * z.squaredNorm())) return true;
    }
    return false;
  }
  // Bounded rejection, never radial clipping.
  for (unsigned i = 0; i < 128; ++i)
  {
    for (int j = 0; j < 3; ++j) z[j] = normal(rng);
    if (z.squaredNorm() <= cutoff * cutoff) return true;
  }
  return false;
}
struct Projection { Eigen::MatrixXd inverse, null_basis; };
inline Projection project(const Eigen::MatrixXd& jacobian)
{
  if (jacobian.rows() != 3 || jacobian.cols() < 3 || !jacobian.allFinite())
    throw std::invalid_argument("Invalid position Jacobian");
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(jacobian, Eigen::ComputeFullU | Eigen::ComputeFullV);
  const auto values = svd.singularValues();
  // Do not amplify near-singular directions or disguise them with task-space-leaking damping.
  if (values.minCoeff() < 1e-3) throw std::invalid_argument("Singular anchor Jacobian");
  return {svd.matrixV().leftCols(3) * values.cwiseInverse().asDiagonal() * svd.matrixU().transpose(),
          svd.matrixV().rightCols(jacobian.cols() - 3)};
}
inline double mahalanobisSquared(const Gaussian& g, const Eigen::Vector3d& x)
{
  return (g.axes.transpose() * (x - g.mean)).cwiseQuotient(g.eigenvalues.cwiseSqrt()).squaredNorm();
}
// Roundoff allowance in squared Mahalanobis units, not a geometric dilation.
inline double supportRoundoff(double cutoff)
{ return 64 * std::numeric_limits<double>::epsilon() * std::max(1., cutoff * cutoff); }
inline double unionMahalanobisSquared(const std::vector<Gaussian>& gaussians,
                                     const std::vector<double>& weights, const Eigen::Vector3d& x)
{
  double best = std::numeric_limits<double>::infinity();
  if (!x.allFinite()) return best;
  for (size_t i = 0; i < gaussians.size(); ++i)
    if (weights.at(i) > 0) best = std::min(best, mahalanobisSquared(gaussians[i], x));
  return best;
}
inline bool unionContains(const std::vector<Gaussian>& gaussians, const std::vector<double>& weights,
                          const Eigen::Vector3d& x, double cutoff)
{ return unionMahalanobisSquared(gaussians, weights, x) <= cutoff * cutoff + supportRoundoff(cutoff); }

// Exact Euclidean projection onto an ellipsoid, solved by the monotone secular
// equation. Outside points have a unique nonnegative Lagrange multiplier.
inline double ellipsoidDistance(const Gaussian& g, const Eigen::Vector3d& x, double cutoff)
{
  const Eigen::Vector3d p = g.axes.transpose() * (x - g.mean);
  const Eigen::Vector3d a2 = cutoff * cutoff * g.eigenvalues;
  if ((p.array().square() / a2.array()).sum() <= 1.) return 0.;
  double lo = 0., hi = (a2.array() * p.array().square()).sum();
  hi = std::sqrt(hi); // This upper bound makes the secular sum <= 1.
  for (unsigned i = 0; i < 80; ++i)
  {
    const double lambda = (lo + hi) / 2.;
    const double value = (a2.array() * p.array().square() / (a2.array() + lambda).square()).sum();
    if (value > 1.) lo = lambda; else hi = lambda;
  }
  const Eigen::Vector3d nearest = (a2.array() * p.array() / (a2.array() + (lo + hi) / 2.)).matrix();
  return (p - nearest).norm();
}
inline double unionDistance(const std::vector<Gaussian>& gaussians, const std::vector<double>& weights,
                            const Eigen::Vector3d& x, double cutoff)
{
  if (unionMahalanobisSquared(gaussians, weights, x) <= cutoff * cutoff) return 0.;
  double best = std::numeric_limits<double>::infinity();
  for (size_t i = 0; i < gaussians.size(); ++i)
    if (weights.at(i) > 0) best = std::min(best, ellipsoidDistance(gaussians[i], x, cutoff));
  return best;
}
}  // namespace tp_gmm::sampling
