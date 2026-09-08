#pragma once
#include <Eigen/Dense>
#include <random>
#include <stdexcept>

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
  // Bounded rejection, never radial clipping. cutoff >= 1 is required by the service.
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
}  // namespace tp_gmm::sampling
