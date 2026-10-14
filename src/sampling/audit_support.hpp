#pragma once
#include <tp_gmm/sampling_math.hpp>
#include <json/json.h>
#include <sstream>

namespace tp_gmm::sampling
{
// Consume the canonical descriptor returned by PrepareSampling, independently of
// the live allocator/session. Native MoveIt boxes are only a broad-phase test.
struct AuditSupport
{
  std::vector<Gaussian> gaussians;
  std::vector<double> weights;
  double cutoff = 0.;
  bool exact = false;
  AuditSupport(const std::string& encoded, const std::string& constraints_name)
  {
    if (encoded.empty()) return;  // Unrestricted or historical native geometry.
    Json::Value root;
    Json::CharReaderBuilder builder;
    std::string error;
    std::istringstream input(encoded);
    if (!Json::parseFromStream(builder, input, &root, &error) || root["schema_version"].asInt() != 1 ||
        root["constraints_name"].asString() != constraints_name)
      throw std::invalid_argument("Invalid/mismatched corridor audit descriptor");
    const auto geometry = root["geometry"].asString();
    if (geometry == "gmm_boxes" || geometry == "gmr_tube") return;
    if (geometry != "gmm_ellipsoids") throw std::invalid_argument("Unknown corridor geometry");
    exact = true;
    cutoff = root["cutoff"].asDouble();
    if (!std::isfinite(cutoff) || cutoff <= 0.) throw std::invalid_argument("Invalid exact cutoff");
    for (const auto& component : root["components"])
    {
      if (component["mean"].size() != 3 || component["eigenvalues"].size() != 3 || component["axes"].size() != 9)
        throw std::invalid_argument("Malformed canonical ellipsoid");
      Gaussian g;
      for (unsigned i=0; i<3; ++i)
      {
        g.mean[i] = component["mean"][i].asDouble();
        g.eigenvalues[i] = component["eigenvalues"][i].asDouble();
        for (unsigned j=0; j<3; ++j) g.axes(i,j) = component["axes"][3*i+j].asDouble();
      }
      const double weight = component["weight"].asDouble();
      if (!g.mean.allFinite() || !g.eigenvalues.allFinite() || g.eigenvalues.minCoeff() <= 0. ||
          !g.axes.allFinite() || !(g.axes.transpose()*g.axes).isApprox(Eigen::Matrix3d::Identity(), 1e-9) ||
          !std::isfinite(weight) || weight < 0.)
        throw std::invalid_argument("Invalid canonical ellipsoid");
      gaussians.push_back(g); weights.push_back(weight);
    }
    if (gaussians.empty() || std::none_of(weights.begin(), weights.end(), [](double w) { return w > 0.; }))
      throw std::invalid_argument("Empty ellipsoid support");
  }
  bool contains(const Eigen::Vector3d& x) const
  { return !exact || unionContains(gaussians, weights, x, cutoff); }
};
}
