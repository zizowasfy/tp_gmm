#include <tp_gmm/path_proposal.hpp>
#include <tp_gmm/sampling_math.hpp>
#include <cassert>
#include <iostream>
int main()
{
  using namespace tp_gmm::sampling;
  const ReferencePath path({{0,0,0},{.1,0,0},{.1,0,0},{1,0,0}});
  assert(std::abs(path.length()-1) < 1e-12);
  assert((path.at(.5)-Eigen::Vector3d(.5,0,0)).norm() < 1e-12);
  assert(path.at(1).x() == 1 && path.at(0).x() == 0);
  assert(std::abs(path.distance({.5,.1,0})-.1) < 1e-12);
  assert(std::abs(path.distance({1.1,0,0})-.1) < 1e-12);
  for (const auto& bad : std::vector<std::vector<Eigen::Vector3d>>{
      {}, {{0,0,0}}, {{0,0,0},{0,0,0}}, {{0,0,0},{NAN,0,0}}})
  {
    bool rejected = false;
    try { ReferencePath invalid(bad); } catch (const std::invalid_argument&) { rejected = true; }
    assert(rejected);
  }
  std::mt19937 rng(42);
  std::normal_distribution<double> normal;
  std::uniform_real_distribution<double> uniform(0.,1.);
  unsigned short_segment=0; double mean=0, offset=0;
  for (unsigned i=0; i<100000; ++i)
  {
    const auto center = path.at(uniform(rng));
    short_segment += center.x() < .1; mean += center.x();
    Eigen::Vector3d z; assert(truncatedNormal(rng,normal,2.,z));
    const Eigen::Vector3d x = center + .01*z;
    assert(path.distance(x) <= .02000001);
    offset += z.squaredNorm();
  }
  // Arc weighting, not uniform waypoint/segment weighting, despite uneven spacing.
  assert(short_segment > 9700 && short_segment < 10300);
  assert(std::abs(mean/100000-.5) < .004);
  // E[||Z||^2 | ||Z||<=2] ~= 1.83; no atom from radial clipping.
  assert(offset/100000 > 1.80 && offset/100000 < 1.86);
  std::cout << "PASS: arc-length sampling, duplicate handling, finite geometry, truncated neighborhoods\n";
}
