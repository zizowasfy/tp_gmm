#include "sampler.hpp"

namespace tp_gmm
{
GMMConstraintSamplerAllocator::GMMConstraintSamplerAllocator()
{
  rclcpp::NodeOptions options;
  options.arguments({"--ros-args", "-r", "__node:=gmm_sampling"});
  node_ = std::make_shared<rclcpp::Node>("gmm_sampling", options);
  epoch_ = std::to_string(std::chrono::system_clock::now().time_since_epoch().count());
  markers_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>("~/markers", rclcpp::QoS(1).transient_local());
  paths_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>("~/paths", rclcpp::QoS(1).transient_local());
  prepare_service_ = node_->create_service<srv::PrepareSampling>("~/prepare", [this](const srv::PrepareSampling::Request::SharedPtr req, srv::PrepareSampling::Response::SharedPtr res) { prepare(*req, *res); });
  report_service_ = node_->create_service<srv::SamplingReport>("~/report", [this](const srv::SamplingReport::Request::SharedPtr req, srv::SamplingReport::Response::SharedPtr res) { report(*req, *res); });
  timer_ = node_->create_wall_timer(std::chrono::milliseconds(100), [this] { publishMarkers(); });
  executor_.add_node(node_);
  worker_ = std::thread([this] { executor_.spin(); });
}
GMMConstraintSamplerAllocator::~GMMConstraintSamplerAllocator()
{
  executor_.cancel();
  if (worker_.joinable()) worker_.join();
}
void GMMConstraintSamplerAllocator::prepare(const srv::PrepareSampling::Request& req, srv::PrepareSampling::Response& res)
{
  try
  {
    const auto begin = Clock::now();
    if (req.mode != "cartesian_ik" && req.mode != "joint_projected" && req.mode != "uniform") throw std::invalid_argument("Unknown sampling mode");
    if (req.group_name.empty() || req.link_name.empty() || req.model.header.frame_id.empty()) throw std::invalid_argument("Group, link and model frame are required");
    auto range = [](double x, double lo, double hi) { return std::isfinite(x) && x >= lo && x <= hi; };
    if (req.corridor_mode != "legacy_weighted" && req.corridor_mode != "covariance")
      throw std::invalid_argument("Unknown corridor_mode");
    if (!range(req.corridor_scale, 0.001, 1000) || !range(req.cutoff, 1, 6) || !range(req.covariance_floor, 1e-12, 1e-3) ||
        !range(req.uniform_fraction, 0, 1) || !range(req.cartesian_fraction, 0, 1) ||
        !range(req.ik_timeout, 0.0001, 0.1) || !range(req.nullspace_stddev, 0, 0.5) ||
        !range(req.max_joint_delta, 0.01, 2) || !range(req.linearization_tolerance, 1e-5, 0.1) ||
        req.branches < 1 || req.branches > 16 || req.anchor_attempts < 1 || req.anchor_attempts > 128)
      throw std::invalid_argument("Invalid sampler options");
    if (!quaternion(req.orientation).coeffs().allFinite() || std::abs(quaternion(req.orientation).norm() - 1) > 1e-5)
      throw std::invalid_argument("Orientation must be a normalized quaternion in the GMM frame");
    if (req.model.gaussians.empty() || req.model.gaussians.size() > 64 || req.model.weights.size() != req.model.gaussians.size())
      throw std::invalid_argument("Expected 1..64 Gaussians and one weight per component");
    if (req.proposal != "gmm" && req.proposal != "gmr_path" && req.proposal != "hybrid")
      throw std::invalid_argument("Unknown proposal");
    if (req.proposal != "gmm" && req.mode != "cartesian_ik")
      throw std::invalid_argument("GMR path proposals currently require cartesian_ik mapping");
    if (!range(req.gmr_stddev, 0., 0.5) || !range(req.gmr_cutoff, 1., 6.) || !range(req.gmr_fraction, 0., 1.))
      throw std::invalid_argument("Invalid GMR proposal settings");
    auto session = std::make_shared<SamplingSession>();
    session->config = req;
    const auto path_begin = Clock::now();
    if (!req.reference_path.poses.empty())
    {
      if (req.reference_path.header.frame_id != req.model.header.frame_id)
        throw std::invalid_argument("Reference path frame must match the GMM frame");
      std::vector<Eigen::Vector3d> points;
      for (const auto& p : req.reference_path.poses) points.emplace_back(p.position.x, p.position.y, p.position.z);
      session->reference = std::make_shared<sampling::ReferencePath>(points);
      session->metrics["reference_length_m"] = session->reference->length();
      session->metrics["reference_points"] = session->reference->points().size();
    }
    if (req.proposal != "gmm" && !session->reference)
      throw std::invalid_argument("GMR proposal requires a request-matched reference path");
    session->metrics["reference_prepare_s"] = seconds(path_begin);
    for (const char* key : {"instances", "setup_s", "sampling_s", "draw_s", "online_ik_s", "anchor_ik_s",
         "jacobian_setup_s", "fk_mapping_s", "validity_s", "uniform_sampler_s", "sampler_calls", "failed_calls",
         "attempts", "valid_samples", "uniform_attempts", "uniform_valid", "cartesian_attempts", "cartesian_valid",
         "projected_attempts", "projected_valid", "online_ik_calls", "online_ik_success", "anchor_ik_calls",
         "anchor_ik_success", "anchors", "unanchored_components", "missing_anchor_fallbacks", "missing_anchor_rejections", "singular_anchors",
         "trust_region_rejections", "projection_support_rejections", "bounds_rejections", "constraint_rejections",
         "collision_checks", "collision_rejections", "callback_rejections", "feasibility_rejections",
         "linearization_residual_sum_m", "linearization_checks", "proposal_mahalanobis_squared_sum",
         "valid_mahalanobis_squared_sum", "gmr_attempts", "gmr_valid", "gmr_draws", "gmr_fraction_sum", "gmr_offset_squared_sum_m2"}) session->metrics[key] = 0;
    double total = 0;
    moveit_msgs::msg::PositionConstraint position;
    position.header = req.model.header; position.link_name = req.link_name; position.weight = 1;
    for (size_t k = 0; k < req.model.gaussians.size(); ++k)
    {
      const auto& message = req.model.gaussians[k];
      const size_t d = message.means.size();
      if ((d != 3 && d != 4) || message.covariances.size() != d * d) throw std::invalid_argument("Expected xyz or time/xyz with a square row-major covariance");
      for (double value : message.means) if (!std::isfinite(value)) throw std::invalid_argument("Non-finite mean");
      for (double value : message.covariances) if (!std::isfinite(value)) throw std::invalid_argument("Non-finite covariance");
      const size_t offset = d - 3;
      Eigen::Vector3d mean; Eigen::Matrix3d covariance;
      for (size_t i = 0; i < 3; ++i)
      {
        mean[i] = message.means[i + offset];
        for (size_t j = 0; j < 3; ++j) covariance(i, j) = message.covariances[(i + offset) * d + j + offset];
      }
      auto g = sampling::canonicalGaussian(mean, covariance, req.covariance_floor);
      if (!range(req.model.weights[k], 0, 1e20)) throw std::invalid_argument("Invalid component weight");
      total += req.model.weights[k]; session->weights.push_back(req.model.weights[k]); session->gaussians.push_back(g);
      // Preserve the historical hard region independently of the sampling covariance.
      const double radius = req.corridor_mode == "legacy_weighted" ?
          0.5 * req.corridor_scale * req.model.weights[k] : req.cutoff;
      session->cutoffs.push_back(radius);
      if (radius == 0) continue;  // A zero-prior historical component has no region.
      shape_msgs::msg::SolidPrimitive box; box.type = box.BOX;
      for (int j = 0; j < 3; ++j) box.dimensions.push_back(2 * radius * std::sqrt(g.eigenvalues[j]));
      geometry_msgs::msg::Pose pose; pose.position = point(mean);
      const Eigen::Quaterniond q(g.axes); pose.orientation.w = q.w(); pose.orientation.x = q.x(); pose.orientation.y = q.y(); pose.orientation.z = q.z();
      position.constraint_region.primitives.push_back(box); position.constraint_region.primitive_poses.push_back(pose);
    }
    if (!std::isfinite(total) || total <= 0) throw std::invalid_argument("Weights must have positive finite sum");
    for (double& w : session->weights) w /= total;
    std::lock_guard<std::mutex> lock(mutex_);
    // Expire only unreferenced, abandoned sessions. Active samplers retain their immutable snapshot.
    for (auto it = sessions_.begin(); it != sessions_.end();)
      if (it->second.use_count() == 1 && seconds(it->second->created) > 600) it = sessions_.erase(it); else ++it;
    if (sessions_.size() >= 64) throw std::invalid_argument("Session capacity reached; release completed requests with report");
    session->id = epoch_ + "_" + std::to_string(++sequence_);
    session->constraints.name = "tpgmm:" + session->id;
    session->constraints.position_constraints.push_back(position);
    session->metrics["prepare_s"] = seconds(begin);
    sessions_[session->id] = session;
    res.success = true; res.request_id = session->id; res.constraints = session->constraints;
    res.message = "Prepared immutable model and matching corridor";
  }
  catch (const std::exception& ex) { res.success = false; res.message = ex.what(); }
}
bool GMMConstraintSamplerAllocator::canService(const planning_scene::PlanningSceneConstPtr&, const std::string&,
                                               const moveit_msgs::msg::Constraints& constraints) const
{ return constraints.name.rfind("tpgmm:", 0) == 0; }
constraint_samplers::ConstraintSamplerPtr GMMConstraintSamplerAllocator::alloc(const planning_scene::PlanningSceneConstPtr& scene,
    const std::string& group, const moveit_msgs::msg::Constraints& constraints)
{
  std::shared_ptr<SamplingSession> session;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto it = sessions_.find(constraints.name.substr(6));
    if (it == sessions_.end()) throw std::runtime_error("Unknown/expired TP-GMM request ID; prepare a fresh request");
    session = it->second;
  }
  auto sampler = std::make_shared<GMMConstraintSampler>(scene, group, session);
  if (!sampler->configure(constraints)) throw std::runtime_error("TP-GMM configuration mismatch (check model frame, chain, link and corridor)");
  return sampler;
}

}  // namespace tp_gmm

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(tp_gmm::GMMConstraintSamplerAllocator, constraint_samplers::ConstraintSamplerAllocator)
