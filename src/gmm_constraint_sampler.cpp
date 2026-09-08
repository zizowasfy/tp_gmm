#include <tp_gmm/gmm_constraint_sampler.hpp>
#include <tp_gmm/sampling_math.hpp>
#include <moveit/constraint_samplers/default_constraint_samplers.hpp>
#include <moveit/robot_state/conversions.hpp>
#include <moveit/robot_trajectory/robot_trajectory.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <random_numbers/random_numbers.h>
#include <chrono>
#include <algorithm>
#include <limits>
#include <deque>
#include <iomanip>
#include <sstream>

namespace tp_gmm
{
using Clock = std::chrono::steady_clock;
static double seconds(Clock::time_point start) { return std::chrono::duration<double>(Clock::now() - start).count(); }
using Metrics = std::map<std::string, double>;
struct SamplingSession
{
  srv::PrepareSampling::Request config;
  moveit_msgs::msg::Constraints constraints;
  std::string id;
  std::vector<sampling::Gaussian> gaussians;
  std::vector<double> weights, cutoffs;
  std::mutex mutex;
  Metrics metrics;
  // Four independently bounded buffers: raw Cartesian, valid FK, rejected FK, uniform valid.
  std::deque<geometry_msgs::msg::Point> points[4];
  planning_scene::PlanningSceneConstPtr scene;
  Clock::time_point created = Clock::now();
};
static geometry_msgs::msg::Point point(const Eigen::Vector3d& x)
{
  geometry_msgs::msg::Point p; p.x = x.x(); p.y = x.y(); p.z = x.z(); return p;
}
static Eigen::Quaterniond quaternion(const geometry_msgs::msg::Quaternion& q) { return {q.w, q.x, q.y, q.z}; }
static void moments(Metrics& m, const std::string& prefix, const Eigen::Vector3d& x)
{
  m[prefix + "count"]++;
  for (int i = 0; i < 3; ++i)
  {
    m[prefix + "sum_" + std::to_string(i)] += x[i];
    for (int j = 0; j < 3; ++j)
      m[prefix + "outer_" + std::to_string(i) + std::to_string(j)] += x[i] * x[j];
  }
}
class GMMConstraintSampler final : public constraint_samplers::ConstraintSampler
{
  struct Anchor { std::vector<double> q; Eigen::MatrixXd task_factor, null_basis, jacobian; };
public:
  GMMConstraintSampler(const planning_scene::PlanningSceneConstPtr& scene, const std::string& group,
                      std::shared_ptr<SamplingSession> session)
    : ConstraintSampler(scene, group), session_(std::move(session)), rng_(session_->config.seed),
      seed_rng_(std::make_unique<random_numbers::RandomNumberGenerator>(session_->config.seed)), mixture_(session_->weights.begin(), session_->weights.end()),
      constraint_set_(scene->getRobotModel())
  {
    std::lock_guard<std::mutex> lock(session_->mutex);
    const auto instance = static_cast<unsigned>(session_->metrics["allocated_instances"]++);
    rng_.seed(session_->config.seed + 104729u * instance);
    seed_rng_ = std::make_unique<random_numbers::RandomNumberGenerator>(session_->config.seed + 104729u * instance);
  }
  const std::string& getName() const override { static const std::string name = "GMMConstraintSampler"; return name; }
  bool configure(const moveit_msgs::msg::Constraints& constraints) override
  {
    is_valid_ = false;
    const auto start = Clock::now();
    if (!jmg_ || constraints != session_->constraints || jmg_->getName() != session_->config.group_name)
      return false;
    link_ = scene_->getRobotModel()->getLinkModel(session_->config.link_name);
    if (!link_ || !jmg_->isChain() || !jmg_->getUpdatedLinkModelsSet().count(link_)) return false;
    // Keep Gaussians in their fixed message frame; transform IK/FK and the Jacobian consistently.
    const auto& frame = session_->config.model.header.frame_id;
    if (!scene_->knowsFrameTransform(frame)) throw std::runtime_error("Unknown GMM frame: " + frame);
    if (const auto* frame_link = scene_->getRobotModel()->getLinkModel(frame))
    {
      for (const auto* parent = frame_link; parent; parent = parent->getParentLinkModel())
        if (parent->getParentJointModel()->getType() != moveit::core::JointModel::FIXED)
          throw std::runtime_error("GMM frame must be fixed relative to the model frame");
    }
    else if (!scene_->getTransforms().isFixedFrame(frame))
      throw std::runtime_error("GMM frame must be a known fixed frame");
    model_from_gmm_ = scene_->getFrameTransform(frame);
    if (!constraint_set_.add(constraints, scene_->getTransforms())) return false;
    fallback_ = std::make_shared<constraint_samplers::IKConstraintSampler>(scene_, jmg_->getName());
    if (!fallback_->configure(constraints)) return false;
    anchors_.resize(session_->gaussians.size());
    if (session_->config.mode == "joint_projected") buildAnchors();
    local_["setup_s"] += seconds(start);
    local_["instances"] = 1;
    is_valid_ = true;
    {
      std::lock_guard<std::mutex> lock(session_->mutex);
      // Allocation is against the request's planning-scene snapshot, not a global/latest scene.
      session_->scene = scene_;
    }
    flush();
    return true;
  }
  bool sample(moveit::core::RobotState& state, const moveit::core::RobotState& reference,
              unsigned int max_attempts) override
  {
    const auto start = Clock::now();
    local_["sampler_calls"]++;
    // Copy before touching output: OMPL may pass the same object as state and reference.
    const moveit::core::RobotState reference_copy(reference);
    bool success = false;
    for (unsigned attempt = 0; is_valid_ && attempt < max_attempts; ++attempt)
    {
      moveit::core::RobotState candidate(reference_copy);
      local_["attempts"]++;
      if (session_->config.mode == "uniform" || uniform_(rng_) < session_->config.uniform_fraction)
      {
        local_["uniform_attempts"]++;
        const auto begin = Clock::now();
        bool ok = fallback_->sample(candidate, reference_copy, 1);
        local_["uniform_sampler_s"] += seconds(begin);
        if (ok && valid(candidate)) { state = candidate; local_["uniform_valid"]++; record(3, fk(candidate)); success = true; }
      }
      else
      {
        const size_t k = mixture_(rng_);
        local_["component_" + std::to_string(k) + "_selected"]++;
        bool projected = session_->config.mode == "joint_projected" &&
                         uniform_(rng_) >= session_->config.cartesian_fraction && !anchors_[k].empty();
        if (session_->config.mode == "joint_projected" && anchors_[k].empty()) local_["missing_anchor_fallbacks"]++;
        bool ok = projected ? jointSample(candidate, k) : cartesianSample(candidate, k);
        if (ok)
        {
          const Eigen::Vector3d x = fk(candidate);
          if (valid(candidate))
          {
            state = candidate; success = true; record(1, x);
            local_[projected ? "projected_valid" : "cartesian_valid"]++;
            moments(local_, "component_" + std::to_string(k) + "_valid_", x);
            local_["valid_mahalanobis_squared_sum"] += sampling::mahalanobisSquared(session_->gaussians[k], x);
          }
          else record(2, x);
        }
      }
      if (success) { local_["valid_samples"]++; break; }
    }
    if (!success) local_["failed_calls"]++;
    local_["sampling_s"] += seconds(start);
    flush();
    return success;
  }
private:
  Eigen::Vector3d fk(moveit::core::RobotState& state) const
  { state.update(); return model_from_gmm_.inverse() * state.getGlobalLinkTransform(link_).translation(); }
  bool valid(moveit::core::RobotState& state)
  {
    const auto start = Clock::now();
    state.update();
    bool ok = true;
    if (!state.satisfiesBounds(jmg_)) { local_["bounds_rejections"]++; ok = false; }
    else if (!constraint_set_.decide(state).satisfied) { local_["constraint_rejections"]++; ok = false; }
    else if (!scene_->isStateFeasible(state, false)) { local_["feasibility_rejections"]++; ok = false; }
    else
    {
      local_["collision_checks"]++;
      if (scene_->isStateColliding(state, jmg_->getName(), false)) { local_["collision_rejections"]++; ok = false; }
    }
    if (ok && group_state_validity_callback_)
    {
      std::vector<double> q; state.copyJointGroupPositions(jmg_, q);
      if (!group_state_validity_callback_(&state, jmg_, q.data())) { local_["callback_rejections"]++; ok = false; }
    }
    local_["validity_s"] += seconds(start);
    return ok;
  }
  bool ik(moveit::core::RobotState& state, const Eigen::Vector3d& x, bool anchor)
  {
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.linear() = quaternion(session_->config.orientation).toRotationMatrix();
    pose.translation() = x;
    const auto start = Clock::now();
    local_[anchor ? "anchor_ik_calls" : "online_ik_calls"]++;
    bool ok = state.setFromIK(jmg_, model_from_gmm_ * pose, link_->getName(), session_->config.ik_timeout);
    local_[anchor ? "anchor_ik_s" : "online_ik_s"] += seconds(start);
    if (ok && state.satisfiesBounds(jmg_) && (fk(state) - x).norm() <= 1e-4)
    { local_[anchor ? "anchor_ik_success" : "online_ik_success"]++; return true; }
    return false;
  }
  bool cartesianSample(moveit::core::RobotState& state, size_t k)
  {
    local_["cartesian_attempts"]++;
    const auto start = Clock::now();
    Eigen::Vector3d z;
    if (!sampling::truncatedNormal(rng_, normal_, session_->cutoffs[k], z)) return false;
    Eigen::Vector3d x = session_->gaussians[k].mean + session_->gaussians[k].factor * z;
    local_["draw_s"] += seconds(start);
    moments(local_, "component_" + std::to_string(k) + "_proposal_", x);
    local_["proposal_mahalanobis_squared_sum"] += z.squaredNorm();
    record(0, x);
    state.setToRandomPositions(jmg_, *seed_rng_);
    return ik(state, x, false);
  }
  bool jointSample(moveit::core::RobotState& state, size_t k)
  {
    local_["projected_attempts"]++;
    const auto start = Clock::now();
    const Anchor& anchor = anchors_[k][std::uniform_int_distribution<size_t>(0, anchors_[k].size() - 1)(rng_)];
    Eigen::Vector3d z;
    if (!sampling::truncatedNormal(rng_, normal_, session_->cutoffs[k], z)) return false;
    const Eigen::Vector3d linear_target = session_->gaussians[k].mean + session_->gaussians[k].factor * z;
    moments(local_, "component_" + std::to_string(k) + "_projected_linear_", linear_target);
    record(0, linear_target);
    Eigen::VectorXd u(anchor.null_basis.cols());
    for (int i = 0; i < u.size(); ++i) u[i] = normal_(rng_);
    const Eigen::VectorXd delta = anchor.task_factor * z + session_->config.nullspace_stddev * anchor.null_basis * u;
    local_["draw_s"] += seconds(start);
    if (delta.norm() > session_->config.max_joint_delta) { local_["trust_region_rejections"]++; return false; }
    Eigen::VectorXd q = Eigen::Map<const Eigen::VectorXd>(anchor.q.data(), anchor.q.size()) + delta;
    state.setJointGroupPositions(jmg_, q);
    if (!state.satisfiesBounds(jmg_)) { local_["bounds_rejections"]++; return false; }
    const auto fk_start = Clock::now();
    const Eigen::Vector3d x = fk(state);
    const Eigen::Vector3d linear = session_->gaussians[k].mean + anchor.jacobian * delta;
    const double residual = (x - linear).norm();
    local_["fk_mapping_s"] += seconds(fk_start);
    local_["linearization_residual_sum_m"] += residual;
    local_["linearization_checks"]++;
    moments(local_, "component_" + std::to_string(k) + "_projected_fk_", x);
    if (residual > session_->config.linearization_tolerance ||
        sampling::mahalanobisSquared(session_->gaussians[k], x) > session_->cutoffs[k] * session_->cutoffs[k])
    { local_["projection_support_rejections"]++; record(2, x); return false; }
    return true;
  }
  void buildAnchors()
  {
    moveit::core::RobotState seed(scene_->getCurrentState());
    for (size_t k = 0; k < anchors_.size(); ++k)
    {
      for (unsigned attempt = 0; attempt < session_->config.anchor_attempts && anchors_[k].size() < session_->config.branches; ++attempt)
      {
        if (attempt) seed.setToRandomPositions(jmg_, *seed_rng_);
        if (!ik(seed, session_->gaussians[k].mean, true)) continue;
        std::vector<double> q; seed.copyJointGroupPositions(jmg_, q);
        bool duplicate = false;
        for (const auto& a : anchors_[k])
          if ((Eigen::Map<const Eigen::VectorXd>(q.data(), q.size()) - Eigen::Map<const Eigen::VectorXd>(a.q.data(), a.q.size())).norm() < 0.1) duplicate = true;
        if (duplicate) continue;
        const auto start = Clock::now();
        Eigen::MatrixXd full;
        if (!seed.getJacobian(jmg_, link_, Eigen::Vector3d::Zero(), full) || full.rows() != 6 || full.cols() != int(jmg_->getVariableCount())) continue;
        // MoveIt returns the Jacobian in the group-root frame. Rotate it to the model frame.
        const auto* root = jmg_->getJointModels().front()->getParentLinkModel();
        Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
        if (root) rotation = seed.getGlobalLinkTransform(root).linear();
        Eigen::MatrixXd jacobian = model_from_gmm_.linear().transpose() * rotation * full.topRows(3);
        try
        {
          const auto projection = sampling::project(jacobian);
          anchors_[k].push_back({q, projection.inverse * session_->gaussians[k].factor, projection.null_basis, jacobian});
          local_["anchors"]++;
        }
        catch (const std::invalid_argument&) { local_["singular_anchors"]++; }
        local_["jacobian_setup_s"] += seconds(start);
      }
      if (anchors_[k].empty()) local_["unanchored_components"]++;
    }
  }
  void record(int stage, const Eigen::Vector3d& x)
  {
    if (!session_->config.visualize) return;
    std::lock_guard<std::mutex> lock(session_->mutex);
    auto& buffer = session_->points[stage];
    if (buffer.size() == 1000) buffer.pop_front();
    buffer.push_back(point(x));
  }
  void flush()
  {
    std::lock_guard<std::mutex> lock(session_->mutex);
    for (const auto& [key, value] : local_) session_->metrics[key] += value;
    local_.clear();
  }
  std::shared_ptr<SamplingSession> session_;
  std::mt19937 rng_;
  std::unique_ptr<random_numbers::RandomNumberGenerator> seed_rng_;
  std::normal_distribution<double> normal_{0, 1};
  std::uniform_real_distribution<double> uniform_{0, 1};
  std::discrete_distribution<size_t> mixture_;
  kinematic_constraints::KinematicConstraintSet constraint_set_;
  const moveit::core::LinkModel* link_{nullptr};
  Eigen::Isometry3d model_from_gmm_ = Eigen::Isometry3d::Identity();
  constraint_samplers::ConstraintSamplerPtr fallback_;
  std::vector<std::vector<Anchor>> anchors_;
  Metrics local_;
};

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
    auto session = std::make_shared<SamplingSession>();
    session->config = req;
    for (const char* key : {"instances", "setup_s", "sampling_s", "draw_s", "online_ik_s", "anchor_ik_s",
         "jacobian_setup_s", "fk_mapping_s", "validity_s", "uniform_sampler_s", "sampler_calls", "failed_calls",
         "attempts", "valid_samples", "uniform_attempts", "uniform_valid", "cartesian_attempts", "cartesian_valid",
         "projected_attempts", "projected_valid", "online_ik_calls", "online_ik_success", "anchor_ik_calls",
         "anchor_ik_success", "anchors", "unanchored_components", "missing_anchor_fallbacks", "singular_anchors",
         "trust_region_rejections", "projection_support_rejections", "bounds_rejections", "constraint_rejections",
         "collision_checks", "collision_rejections", "callback_rejections", "feasibility_rejections",
         "linearization_residual_sum_m", "linearization_checks", "proposal_mahalanobis_squared_sum",
         "valid_mahalanobis_squared_sum"}) session->metrics[key] = 0;
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
void GMMConstraintSamplerAllocator::report(const srv::SamplingReport::Request& req, srv::SamplingReport::Response& res)
{
  try
  {
    std::shared_ptr<SamplingSession> session;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      const auto it = sessions_.find(req.request_id);
      if (it == sessions_.end()) throw std::invalid_argument("Unknown request ID");
      session = it->second;
    }
    Metrics metrics;
    planning_scene::PlanningSceneConstPtr scene;
    { std::lock_guard<std::mutex> lock(session->mutex); metrics = session->metrics; scene = session->scene; }
    metrics["sampler_active"] = metrics["instances"] > 0;
    if (scene && !req.trajectory.joint_trajectory.points.empty())
    {
      moveit::core::RobotState start(scene->getCurrentState());
      moveit::core::robotStateMsgToRobotState(req.trajectory_start, start);
      robot_trajectory::RobotTrajectory trajectory(scene->getRobotModel(), session->config.group_name);
      trajectory.setRobotTrajectoryMsg(start, req.trajectory);
      metrics["path_invalid_samples"] = 0;
      metrics["path_collision_samples"] = 0;
      metrics["path_constraint_violations"] = 0;
      kinematic_constraints::KinematicConstraintSet path_constraints(scene->getRobotModel());
      path_constraints.add(session->constraints, scene->getTransforms());
      metrics["trajectory_waypoints"] = trajectory.getWayPointCount();
      metrics["trajectory_duration_s"] = trajectory.getDuration();
      double length = 0, cartesian_length = 0, clearance = std::numeric_limits<double>::infinity();
      const auto* group = scene->getRobotModel()->getJointModelGroup(session->config.group_name);
      moveit::core::RobotState interpolated(start);
      visualization_msgs::msg::Marker path;
      path.header.frame_id = scene->getRobotModel()->getModelFrame();
      path.ns = session->config.mode; path.type = path.LINE_STRIP;
      path.pose.orientation.w = 1; path.scale.x = 0.005; path.color.a = 1;
      path.color.r = session->config.mode == "uniform" ? 1 : 0.1;
      path.color.g = session->config.mode == "cartesian_ik" ? 1 : 0.3;
      path.color.b = session->config.mode == "joint_projected" ? 1 : 0.1;
      Eigen::Vector3d previous; bool first = true;
      for (size_t i = 0; i < trajectory.getWayPointCount(); ++i)
      {
        const auto& a = trajectory.getWayPoint(i ? i - 1 : 0); const auto& b = trajectory.getWayPoint(i);
        const double distance = a.distance(b, group);
        length += distance;
        const unsigned steps = std::max(1u, static_cast<unsigned>(std::ceil(distance / 0.02)));
        for (unsigned j = (i ? 1 : 0); j <= steps; ++j)
        {
          a.interpolate(b, double(j) / steps, interpolated); interpolated.update();
          const Eigen::Vector3d x = interpolated.getGlobalLinkTransform(session->config.link_name).translation();
          if (!first) cartesian_length += (x - previous).norm();
          first = false; previous = x; path.points.push_back(point(x));
          // FCL's negative sentinel denotes collision, not a measured penetration depth.
          clearance = std::min(clearance, std::max(0.0, scene->distanceToCollision(interpolated)));
          metrics["path_validation_samples"]++;
          const bool collision = scene->isStateColliding(interpolated, session->config.group_name, false);
          const bool constrained = path_constraints.decide(interpolated).satisfied;
          if (collision) metrics["path_collision_samples"]++;
          if (!constrained) metrics["path_constraint_violations"]++;
          if (collision || !constrained || !interpolated.satisfiesBounds(group) || !scene->isStateFeasible(interpolated, false))
            metrics["path_invalid_samples"]++;
        }
      }
      metrics["joint_path_length"] = length; metrics["ee_path_length_m"] = cartesian_length;
      metrics["min_world_clearance_m"] = clearance;
      metrics["path_interpolation_step"] = 0.02;
      if (session->config.visualize)
      {
        if (metrics["path_invalid_samples"] > 0) { path.color.r = 1; path.color.g = 0; path.color.b = 0; }
        auto& stored = comparison_paths_.markers;
        stored.erase(std::remove_if(stored.begin(), stored.end(), [&](const auto& m) { return m.ns == path.ns; }), stored.end());
        stored.push_back(path); paths_->publish(comparison_paths_);
      }
    }
    std::ostringstream json; json << std::setprecision(12) << "{\"request_id\":\"" << session->id << "\",\"mode\":\"" << session->config.mode << "\"";
    for (const auto& [key, value] : metrics)
    { json << ",\"" << key << "\":"; if (std::isfinite(value)) json << value; else json << "null"; }
    json << ",\"corridor_mode\":\"" << session->config.corridor_mode << "\",\"component_cutoffs\":[";
    for (size_t k = 0; k < session->cutoffs.size(); ++k) { if (k) json << ','; json << session->cutoffs[k]; }
    json << "],\"weights\":[";
    for (size_t k = 0; k < session->weights.size(); ++k) { if (k) json << ','; json << session->weights[k]; }
    json << "]}";
    publishMarkers();
    res.success = true; res.json = json.str(); res.message = "Snapshot after planning; valid targets are not RRT vertices";
    if (req.release) { std::lock_guard<std::mutex> lock(mutex_); sessions_.erase(req.request_id); }
  }
  catch (const std::exception& ex) { res.success = false; res.message = ex.what(); }
}
void GMMConstraintSamplerAllocator::publishMarkers()
{
  visualization_msgs::msg::MarkerArray array;
  std::lock_guard<std::mutex> lock(mutex_);
  for (const auto& [id, session] : sessions_)
  {
    if (!session->config.visualize) continue;
    std::lock_guard<std::mutex> session_lock(session->mutex);
    auto base = [&] {
      visualization_msgs::msg::Marker m; m.header.frame_id = session->config.model.header.frame_id;
      m.header.stamp = node_->now(); m.pose.orientation.w = 1; m.lifetime = rclcpp::Duration::from_seconds(30); return m;
    };
    const char* names[] = {"cartesian_proposals", "valid_fk", "rejected_fk", "uniform_valid"};
    const float colors[4][3] = {{0, 0.8f, 1}, {0.1f, 1, 0.1f}, {1, 0.1f, 0.1f}, {1, 0.8f, 0}};
    for (int s = 0; s < 4; ++s)
    {
      auto m = base(); m.ns = id + "/" + names[s]; m.type = m.POINTS;
      m.scale.x = m.scale.y = 0.006;
      m.color.r = colors[s][0]; m.color.g = colors[s][1]; m.color.b = colors[s][2]; m.color.a = 0.85;
      m.points.assign(session->points[s].begin(), session->points[s].end()); array.markers.push_back(m);
    }
    for (size_t k = 0; k < session->gaussians.size(); ++k)
    {
      if (session->cutoffs[k] == 0) continue;
      const auto& g = session->gaussians[k]; auto m = base(); m.ns = id + "/support"; m.id = k; m.type = m.SPHERE;
      m.pose.position = point(g.mean); const Eigen::Quaterniond q(g.axes);
      m.pose.orientation.w = q.w(); m.pose.orientation.x = q.x(); m.pose.orientation.y = q.y(); m.pose.orientation.z = q.z();
      const Eigen::Vector3d scale = 2 * session->cutoffs[k] * g.eigenvalues.cwiseSqrt();
      m.scale.x = scale.x(); m.scale.y = scale.y(); m.scale.z = scale.z();
      m.color.r = 0.65; m.color.g = 0.4; m.color.b = 1; m.color.a = 0.12; array.markers.push_back(m);
    }
  }
  if (!array.markers.empty()) markers_->publish(array);
}
}  // namespace tp_gmm
PLUGINLIB_EXPORT_CLASS(tp_gmm::GMMConstraintSamplerAllocator, constraint_samplers::ConstraintSamplerAllocator)
