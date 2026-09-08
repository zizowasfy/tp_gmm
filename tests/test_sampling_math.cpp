#include <tp_gmm/sampling_math.hpp>
#include <cassert>
#include <iostream>
int main()
{
  using namespace tp_gmm::sampling;
  Eigen::Matrix3d covariance;
  covariance << 0.04, 0.006, 0, 0.006, 0.01, 0.002, 0, 0.002, 0.005;
  auto g = canonicalGaussian(Eigen::Vector3d(1, 2, 3), covariance, 1e-8);
  assert((g.factor*g.factor.transpose() - covariance).norm() < 1e-12);
  assert(g.axes.determinant() > 0.999);
  bool rejected = false;
  try { canonicalGaussian(Eigen::Vector3d::Zero(), -Eigen::Matrix3d::Identity(), 1e-8); }
  catch (const std::invalid_argument&) { rejected = true; }
  assert(rejected);
  Eigen::Matrix3d asymmetric = covariance; asymmetric(0, 1) += 0.001;
  auto sym = canonicalGaussian(g.mean, asymmetric, 1e-8);
  assert((sym.covariance - (0.5*(asymmetric+asymmetric.transpose())).eval()).norm() < 1e-12);
  std::mt19937 rng(42); std::normal_distribution<double> normal;
  Eigen::Vector3d sum = Eigen::Vector3d::Zero(); Eigen::Matrix3d outer = Eigen::Matrix3d::Zero();
  unsigned boundary = 0; const unsigned count = 150000;
  for (unsigned i = 0; i < count; ++i)
  {
    Eigen::Vector3d z; assert(truncatedNormal(rng, normal, 3, z));
    assert(z.norm() <= 3); if (std::abs(z.norm()-3) < 1e-12) boundary++;
    sum += z; outer += z*z.transpose();
    const auto x = g.mean + g.factor*z;
    assert(std::abs(mahalanobisSquared(g,x)-z.squaredNorm()) < 1e-10);
  }
  assert(boundary == 0 && (sum/count).norm() < 0.015);
  // E[z_i^2 | ||z||<=3] = 0.917807... in three dimensions.
  assert((outer/count - 0.917807*Eigen::Matrix3d::Identity()).norm() < 0.02);
  Eigen::MatrixXd j(3,7);
  j << 1,0,0,0.2,0.1,0.3,0, 0,1,0,0,0.3,0.1,0.1, 0,0,1,0.1,0,0.3,0.1;
  auto p = project(j);
  assert((j*p.inverse - Eigen::Matrix3d::Identity()).norm() < 1e-12);
  assert((j*p.null_basis).norm() < 1e-12);
  const Eigen::MatrixXd qcov = p.inverse*covariance*p.inverse.transpose() + 0.1*p.null_basis*p.null_basis.transpose();
  assert((j*qcov*j.transpose()-covariance).norm() < 1e-12);
  std::cout << "Gaussian factors, invalid covariance, rejection distribution, Mahalanobis support and SVD covariance/null space: PASS\n";
}
