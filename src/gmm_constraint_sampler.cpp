#include <tp_gmm/gmm_constraint_sampler.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/logging.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <std_msgs/msg/string.hpp>

#include <cmath>
#include <sstream>
#include <algorithm>

namespace tp_gmm
{

namespace
{
rclcpp::Logger getLogger()
{
  return rclcpp::get_logger("tp_gmm.constraint_sampler");
}
} // namespace

// =========================================================================
// GMMConstraintSampler Implementation
// =========================================================================

GMMConstraintSampler::GMMConstraintSampler(
    const planning_scene::PlanningSceneConstPtr& scene,
    const std::string& group_name,
    tp_gmm::msg::GaussianMixture::ConstSharedPtr gmm_msg,
    SamplingMode mode,
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr viz_pub,
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr stats_pub)
  : constraint_samplers::ConstraintSampler(scene, group_name)
  , mode_(mode)
  , gmm_msg_(std::move(gmm_msg))
  , viz_pub_(std::move(viz_pub))
  , stats_pub_(std::move(stats_pub))
{
  rng_.seed(std::random_device{}());

  // Setup visual marker template
  sample_marker_.header.frame_id = scene_->getRobotModel()->getModelFrame();
  sample_marker_.ns = "gmm_samples";
  sample_marker_.id = 0;
  sample_marker_.type = visualization_msgs::msg::Marker::SPHERE_LIST;
  sample_marker_.action = visualization_msgs::msg::Marker::ADD;
  sample_marker_.scale.x = 0.012; // 1.2 cm sphere
  sample_marker_.scale.y = 0.012;
  sample_marker_.scale.z = 0.012;
  sample_marker_.color.a = 0.8;
}

GMMConstraintSampler::~GMMConstraintSampler()
{
  publishSamplingStats(true);
  RCLCPP_INFO(getLogger(), "[%s] Destroyed. Total samples drawn: %zu, accepted: %zu (rate: %.1f%%)",
              sampler_name_.c_str(), samples_drawn_, samples_accepted_,
              samples_drawn_ > 0 ? (100.0 * static_cast<double>(samples_accepted_) / samples_drawn_) : 0.0);
}

void GMMConstraintSampler::publishSamplingStats(bool is_final)
{
  if (!stats_pub_) return;

  std::string mode_str = "cartesian_ik";
  if (mode_ == SamplingMode::JOINT_PROJECTED) mode_str = "joint_projected";
  else if (mode_ == SamplingMode::UNIFORM_BOX) mode_str = "uniform_box";

  double rate = samples_drawn_ > 0 ? (static_cast<double>(samples_accepted_) / static_cast<double>(samples_drawn_)) : 0.0;

  std::stringstream ss;
  ss << "{"
     << "\"mode\":\"" << mode_str << "\","
     << "\"samples_drawn\":" << samples_drawn_ << ","
     << "\"samples_accepted\":" << samples_accepted_ << ","
     << "\"rate\":" << rate << ","
     << "\"final\":" << (is_final ? "true" : "false")
     << "}";

  std_msgs::msg::String msg;
  msg.data = ss.str();
  stats_pub_->publish(msg);
}

bool GMMConstraintSampler::configure(const moveit_msgs::msg::Constraints& constr)
{
  clear();
  samples_drawn_ = 0;
  samples_accepted_ = 0;

  if (!jmg_)
  {
    RCLCPP_ERROR(getLogger(), "Null joint model group specified for GMMConstraintSampler");
    return false;
  }

  // Set sampling mode based on constraint name
  if (constr.name == "gmm_joint_projected")
  {
    mode_ = SamplingMode::JOINT_PROJECTED;
    sampler_name_ = "GMMConstraintSampler_JointProjected";
    sample_marker_.ns = "samples_joint_projected";
    sample_marker_.id = 2;
    RCLCPP_INFO(getLogger(), "Configured in Approach 2: Direct Joint-Space GMM Projection mode.");
  }
  else if (constr.name == "gmm_uniform_box")
  {
    mode_ = SamplingMode::UNIFORM_BOX;
    sampler_name_ = "GMMConstraintSampler_UniformBox";
    sample_marker_.ns = "samples_uniform_box";
    sample_marker_.id = 3;
    RCLCPP_INFO(getLogger(), "Configured in Approach 3: Baseline Uniform Bounding Box Corridor mode.");
  }
  else
  {
    mode_ = SamplingMode::CARTESIAN_IK;
    sampler_name_ = "GMMConstraintSampler_CartesianIK";
    sample_marker_.ns = "samples_cartesian_ik";
    sample_marker_.id = 1;
    RCLCPP_INFO(getLogger(), "Configured in Approach 1: Cartesian GMM + Warm-Started IK mode.");
  }

  // Determine end-effector link
  ee_link_ = nullptr;
  if (!constr.position_constraints.empty() && !constr.position_constraints[0].link_name.empty())
  {
    ee_link_ = scene_->getRobotModel()->getLinkModel(constr.position_constraints[0].link_name);
  }

  if (!ee_link_)
  {
    std::vector<std::string> tips;
    jmg_->getEndEffectorTips(tips);
    if (!tips.empty())
    {
      ee_link_ = scene_->getRobotModel()->getLinkModel(tips[0]);
    }
  }

  if (!ee_link_)
  {
    RCLCPP_ERROR(getLogger(), "Could not determine end-effector tip link for group '%s'", jmg_->getName().c_str());
    return false;
  }

  RCLCPP_INFO(getLogger(), "Using end-effector link '%s' for group '%s'",
              ee_link_->getName().c_str(), jmg_->getName().c_str());

  // Determine nominal orientation:
  // 1. If an orientation constraint was provided, use it.
  // 2. Otherwise, inherit the current end-effector orientation from the planning scene (e.g. current robot pose).
  // 3. Fallback to standard Franka downward orientation (w=0, x=1, y=0, z=0: 180 deg around X).
  if (!constr.orientation_constraints.empty())
  {
    const auto& q_msg = constr.orientation_constraints[0].orientation;
    default_orientation_ = Eigen::Quaterniond(q_msg.w, q_msg.x, q_msg.y, q_msg.z);
    default_orientation_.normalize();
    RCLCPP_INFO(getLogger(), "[GMMConstraintSampler] Nominal orientation set from OrientationConstraint: [w=%.3f, x=%.3f, y=%.3f, z=%.3f]",
                default_orientation_.w(), default_orientation_.x(), default_orientation_.y(), default_orientation_.z());
  }
  else if (scene_ && ee_link_)
  {
    Eigen::Matrix3d current_rot = scene_->getCurrentState().getGlobalLinkTransform(ee_link_).rotation();
    default_orientation_ = Eigen::Quaterniond(current_rot);
    default_orientation_.normalize();
    RCLCPP_INFO(getLogger(), "[GMMConstraintSampler] Nominal orientation inherited from current robot state: [w=%.3f, x=%.3f, y=%.3f, z=%.3f]",
                default_orientation_.w(), default_orientation_.x(), default_orientation_.y(), default_orientation_.z());
  }
  else
  {
    default_orientation_ = Eigen::Quaterniond(0.0, 1.0, 0.0, 0.0);
    RCLCPP_INFO(getLogger(), "[GMMConstraintSampler] Nominal orientation defaulted to downward: [w=0.0, x=1.0, y=0.0, z=0.0]");
  }

  // Parse GMM components if message is available
  if (gmm_msg_ && !gmm_msg_->gaussians.empty())
  {
    if (parseGMMMessage(*gmm_msg_))
    {
      for (auto& comp : components_)
      {
        comp.orientation = default_orientation_;
      }
      precomputeNominalJointStatesAndJacobians();
    }
  }

  // Approach 3: Uniform Bounding Box configuration
  if (mode_ == SamplingMode::UNIFORM_BOX)
  {
    uniform_boxes_.clear();
    box_weights_.clear();

    if (!constr.position_constraints.empty())
    {
      const auto& region = constr.position_constraints[0].constraint_region;
      for (size_t i = 0; i < region.primitives.size() && i < region.primitive_poses.size(); ++i)
      {
        const auto& prim = region.primitives[i];
        const auto& pose = region.primitive_poses[i];
        if (prim.type == shape_msgs::msg::SolidPrimitive::BOX && prim.dimensions.size() >= 3)
        {
          BoundingBoxData b;
          b.half_extents = Eigen::Vector3d(prim.dimensions[0] * 0.5,
                                           prim.dimensions[1] * 0.5,
                                           prim.dimensions[2] * 0.5);
          b.center = Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);
          b.orientation = Eigen::Quaterniond(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
          b.orientation.normalize();
          b.volume = prim.dimensions[0] * prim.dimensions[1] * prim.dimensions[2];
          uniform_boxes_.push_back(b);
          box_weights_.push_back(std::max(b.volume, 1e-6));
        }
      }
    }

    // Fallback: build bounding boxes from GMM components if no box primitives specified
    if (uniform_boxes_.empty() && !components_.empty())
    {
      for (const auto& comp : components_)
      {
        BoundingBoxData b;
        Eigen::Vector3d diag = comp.cov_cartesian.diagonal().cwiseSqrt();
        b.half_extents = 2.5 * diag;
        b.center = comp.mean;
        b.orientation = comp.orientation;
        b.volume = 8.0 * b.half_extents.x() * b.half_extents.y() * b.half_extents.z();
        uniform_boxes_.push_back(b);
        box_weights_.push_back(std::max(b.volume, 1e-6));
      }
    }

    if (uniform_boxes_.empty())
    {
      RCLCPP_ERROR(getLogger(), "UNIFORM_BOX mode requested but no bounding box primitives found!");
      return false;
    }

    box_dist_ = std::discrete_distribution<int>(box_weights_.begin(), box_weights_.end());
    is_valid_ = true;
    RCLCPP_INFO(getLogger(), "GMMConstraintSampler successfully configured in UNIFORM_BOX mode with %zu bounding boxes.",
                uniform_boxes_.size());

    // Publish initial box centers preview
    if (viz_pub_)
    {
      for (const auto& b : uniform_boxes_)
      {
        publishVisualSample(b.center, true);
      }
    }
    return true;
  }

  // Approaches 1 & 2 require GMM components
  if (components_.empty())
  {
    RCLCPP_ERROR(getLogger(), "No valid GMM components available to configure GMMConstraintSampler.");
    return false;
  }

  is_valid_ = !components_.empty();
  RCLCPP_INFO(getLogger(), "GMMConstraintSampler successfully configured with %zu Gaussian components.",
              components_.size());

  // Immediately publish initial preview of GMM centers to /gmm_sampling_visualization
  if (is_valid_ && viz_pub_)
  {
    for (const auto& comp : components_)
    {
      publishVisualSample(comp.mean, true);
    }
  }

  return is_valid_;
}

bool GMMConstraintSampler::parseGMMMessage(const tp_gmm::msg::GaussianMixture& msg)
{
  components_.clear();
  weights_.clear();

  size_t K = msg.gaussians.size();
  if (K == 0) return false;

  for (size_t i = 0; i < K; ++i)
  {
    const auto& g = msg.gaussians[i];
    GMMComponent comp;

    size_t dim = g.means.size();
    if (dim == 4)
    {
      // 4D TP-GMM: [time, x, y, z] -> extract spatial coordinates [1, 2, 3]
      comp.mean = Eigen::Vector3d(g.means[1], g.means[2], g.means[3]);

      // Extract 3x3 spatial block from 4x4 covariance matrix
      Eigen::Matrix3d cov;
      for (int r = 0; r < 3; ++r)
      {
        for (int c = 0; c < 3; ++c)
        {
          cov(r, c) = g.covariances[(r + 1) * 4 + (c + 1)];
        }
      }
      comp.cov_cartesian = cov;
    }
    else if (dim == 3)
    {
      comp.mean = Eigen::Vector3d(g.means[0], g.means[1], g.means[2]);
      Eigen::Matrix3d cov;
      for (int r = 0; r < 3; ++r)
      {
        for (int c = 0; c < 3; ++c)
        {
          cov(r, c) = g.covariances[r * 3 + c];
        }
      }
      comp.cov_cartesian = cov;
    }
    else
    {
      RCLCPP_ERROR(getLogger(), "Unsupported Gaussian dimensionality: %zu", dim);
      return false;
    }

    // Symmetrize and add small regularizer for positive definiteness
    comp.cov_cartesian = 0.5 * (comp.cov_cartesian + comp.cov_cartesian.transpose());
    comp.cov_cartesian += 1e-4 * Eigen::Matrix3d::Identity();

    // Compute Cholesky factorization Sigma = L * L^T for efficient sampling
    Eigen::LLT<Eigen::Matrix3d> llt(comp.cov_cartesian);
    if (llt.info() == Eigen::Success)
    {
      comp.cholesky_L_x = llt.matrixL();
    }
    else
    {
      // Fallback: eigenvalue decomposition for semi-definite matrices
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(comp.cov_cartesian);
      Eigen::Vector3d eigenvalues = es.eigenvalues().cwiseMax(1e-4);
      comp.cholesky_L_x = es.eigenvectors() * eigenvalues.cwiseSqrt().asDiagonal();
    }

    double weight = (i < msg.weights.size()) ? msg.weights[i] : 1.0;
    weights_.push_back(std::max(weight, 1e-6));
    components_.push_back(comp);
  }

  // Normalize component weights
  double total_weight = 0.0;
  for (double w : weights_) total_weight += w;
  if (total_weight > 0.0)
  {
    for (double& w : weights_) w /= total_weight;
  }

  component_dist_ = std::discrete_distribution<int>(weights_.begin(), weights_.end());
  return true;
}

void GMMConstraintSampler::precomputeNominalJointStatesAndJacobians()
{
  if (components_.empty() || !jmg_ || !ee_link_) return;

  unsigned int num_joints = jmg_->getVariableCount();
  moveit::core::RobotState temp_state(scene_->getCurrentState());

  std::vector<double> previous_joint_positions;
  temp_state.copyJointGroupPositions(jmg_, previous_joint_positions);

  for (size_t k = 0; k < components_.size(); ++k)
  {
    auto& comp = components_[k];

    Eigen::Isometry3d target_pose = Eigen::Translation3d(comp.mean) * comp.orientation;

    // Warm-start IK with previous component's joint solution to maintain branch continuity
    if (!previous_joint_positions.empty())
    {
      temp_state.setJointGroupPositions(jmg_, previous_joint_positions);
    }

    bool ik_success = temp_state.setFromIK(jmg_, target_pose, ee_link_->getName(), 0.02);
    if (ik_success)
    {
      temp_state.copyJointGroupPositions(jmg_, comp.nominal_joint_positions);
      previous_joint_positions = comp.nominal_joint_positions;
    }
    else
    {
      // Fallback: use reference state joint values
      comp.nominal_joint_positions = previous_joint_positions;
    }
    RCLCPP_INFO(getLogger(), "[GMMConstraintSampler] Component %zu nominal IK %s (pos: [%.2f, %.2f, %.2f])",
                k, ik_success ? "SUCCEEDED" : "FAILED (using fallback)", comp.mean.x(), comp.mean.y(), comp.mean.z());

    // Approach 2: Compute Joint-Space Covariance via First-Order Jacobian Projection
    if (mode_ == SamplingMode::JOINT_PROJECTED)
    {
      temp_state.setJointGroupPositions(jmg_, comp.nominal_joint_positions);
      temp_state.update();

      // Compute geometric Jacobian (6 x num_joints)
      Eigen::MatrixXd J;
      temp_state.getJacobian(jmg_, ee_link_, Eigen::Vector3d::Zero(), J);

      // Extract translational part J_v (3 x num_joints)
      Eigen::MatrixXd J_v = J.topRows(3);

      // Damped pseudo-inverse: J_pinv = J_v^T * (J_v * J_v^T + lambda^2 * I)^(-1)
      double lambda = 0.05;
      Eigen::Matrix3d J_damped = J_v * J_v.transpose() + lambda * lambda * Eigen::Matrix3d::Identity();
      Eigen::MatrixXd J_pinv = J_v.transpose() * J_damped.inverse();

      // Joint covariance projection: Sigma_q = J_pinv * Sigma_x * J_pinv^T + regularizer
      comp.cov_joint = J_pinv * comp.cov_cartesian * J_pinv.transpose();

      // Null-space variance to allow exploration without changing end-effector pose
      Eigen::MatrixXd I_n = Eigen::MatrixXd::Identity(num_joints, num_joints);
      Eigen::MatrixXd N = I_n - J_pinv * J_v;
      double nullspace_var = 0.005; // 0.005 rad^2 exploration variance
      comp.cov_joint += nullspace_var * (N * N.transpose());

      // Regularize for numerical stability
      comp.cov_joint += 1e-4 * I_n;

      // Compute Cholesky factorization for joint distribution
      Eigen::LLT<Eigen::MatrixXd> llt_q(comp.cov_joint);
      if (llt_q.info() == Eigen::Success)
      {
        comp.cholesky_L_q = llt_q.matrixL();
      }
      else
      {
        Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es(comp.cov_joint);
        Eigen::VectorXd evs = es.eigenvalues().cwiseMax(1e-4);
        comp.cholesky_L_q = es.eigenvectors() * evs.cwiseSqrt().asDiagonal();
      }
    }
  }
}

bool GMMConstraintSampler::sample(moveit::core::RobotState& state,
                                  const moveit::core::RobotState& /*reference_state*/,
                                  unsigned int max_attempts)
{
  if (!is_valid_) return false;

  unsigned int num_joints = jmg_->getVariableCount();

  for (unsigned int attempt = 0; attempt < max_attempts; ++attempt)
  {
    ++samples_drawn_;

    if (mode_ == SamplingMode::UNIFORM_BOX)
    {
      // =====================================================================
      // Approach 3: Baseline Uniform Bounding Box Corridor Sampling
      // =====================================================================
      if (uniform_boxes_.empty()) continue;

      int b_idx = box_dist_(rng_);
      const auto& box = uniform_boxes_[b_idx];

      // Uniform random in [-half_extents, +half_extents]
      std::uniform_real_distribution<double> dist_x(-box.half_extents.x(), box.half_extents.x());
      std::uniform_real_distribution<double> dist_y(-box.half_extents.y(), box.half_extents.y());
      std::uniform_real_distribution<double> dist_z(-box.half_extents.z(), box.half_extents.z());

      Eigen::Vector3d local_p(dist_x(rng_), dist_y(rng_), dist_z(rng_));
      Eigen::Vector3d pos = box.center + box.orientation * local_p;

      Eigen::Isometry3d target_pose = Eigen::Translation3d(pos) * default_orientation_;

      // Standard MoveIt IK solve without GMM warm-start (baseline uniform)
      state.setToRandomPositions(jmg_);
      if (state.setFromIK(jmg_, target_pose, ee_link_->getName(), ik_timeout_))
      {
        state.update();
        if (!scene_->isStateColliding(state, jmg_->getName()))
        {
          ++samples_accepted_;
          publishVisualSample(pos, true);
          publishSamplingStats(false);
          return true;
        }
      }
      publishVisualSample(pos, false);
      publishSamplingStats(false);
    }
    else
    {
      if (components_.empty()) continue;

      // 1. Discrete component selection k ~ Categorical(weights)
      int k = component_dist_(rng_);
      const auto& comp = components_[k];

      if (mode_ == SamplingMode::CARTESIAN_IK)
      {
        // =====================================================================
        // Approach 1: Cartesian GMM Sampling + Warm-Started IK
        // =====================================================================
        Eigen::Vector3d z(normal_dist_(rng_), normal_dist_(rng_), normal_dist_(rng_));
        if (z.norm() > 3.0)
        {
          z = z.normalized() * 3.0;
        }

        Eigen::Vector3d pos = comp.mean + comp.cholesky_L_x * z;
        Eigen::Isometry3d target_pose = Eigen::Translation3d(pos) * comp.orientation;

        // Warm-start IK with nominal joint state
        if (!comp.nominal_joint_positions.empty())
        {
          state.setJointGroupPositions(jmg_, comp.nominal_joint_positions);
        }

        // Fast IK solve (warm-started)
        if (state.setFromIK(jmg_, target_pose, ee_link_->getName(), ik_timeout_))
        {
          state.update();
          if (!scene_->isStateColliding(state, jmg_->getName()))
          {
            ++samples_accepted_;
            publishVisualSample(pos, true);
            publishSamplingStats(false);
            return true;
          }
        }
        publishVisualSample(pos, false);
        publishSamplingStats(false);
      }
      else if (mode_ == SamplingMode::JOINT_PROJECTED)
      {
        // =====================================================================
        // Approach 2: Direct Joint-Space GMM Projection (Zero Runtime IK)
        // =====================================================================
        if (comp.nominal_joint_positions.empty() || comp.cholesky_L_q.rows() != num_joints)
        {
          continue;
        }

        Eigen::VectorXd w(num_joints);
        for (unsigned int j = 0; j < num_joints; ++j)
        {
          w(j) = normal_dist_(rng_);
        }

        double max_norm = std::sqrt(18.48);
        if (w.norm() > max_norm)
        {
          w = w.normalized() * max_norm;
        }

        Eigen::VectorXd q_nominal(num_joints);
        for (unsigned int j = 0; j < num_joints; ++j)
        {
          q_nominal(j) = comp.nominal_joint_positions[j];
        }

        Eigen::VectorXd q_sample = q_nominal + comp.cholesky_L_q * w;

        state.setJointGroupPositions(jmg_, q_sample.data());
        state.enforceBounds(jmg_);
        state.update();

        Eigen::Vector3d pos = state.getGlobalLinkTransform(ee_link_).translation();

        if (!scene_->isStateColliding(state, jmg_->getName()))
        {
          ++samples_accepted_;
          publishVisualSample(pos, true);
          publishSamplingStats(false);
          return true;
        }
        publishVisualSample(pos, false);
        publishSamplingStats(false);
      }
    }
  }

  return false;
}

void GMMConstraintSampler::publishVisualSample(const Eigen::Vector3d& point, bool accepted)
{
  if (!viz_pub_) return;

  std::lock_guard<std::mutex> lock(viz_mutex_);

  geometry_msgs::msg::Point p;
  p.x = point.x();
  p.y = point.y();
  p.z = point.z();

  std_msgs::msg::ColorRGBA color;
  if (mode_ == SamplingMode::JOINT_PROJECTED)
  {
    if (accepted)
    {
      // Electric Cyan / Blue for Joint Projection
      color.r = 0.0f; color.g = 0.85f; color.b = 1.0f; color.a = 0.85f;
    }
    else
    {
      // Magenta / Purple for rejected
      color.r = 0.85f; color.g = 0.15f; color.b = 0.85f; color.a = 0.35f;
    }
  }
  else if (mode_ == SamplingMode::UNIFORM_BOX)
  {
    if (accepted)
    {
      // Bright Amber / Gold for Uniform Box
      color.r = 1.0f; color.g = 0.75f; color.b = 0.0f; color.a = 0.85f;
    }
    else
    {
      // Rust / Dark Orange for rejected
      color.r = 0.75f; color.g = 0.35f; color.b = 0.0f; color.a = 0.35f;
    }
  }
  else // CARTESIAN_IK
  {
    if (accepted)
    {
      // Vibrant Emerald Green for Cartesian GMM
      color.r = 0.05f; color.g = 0.95f; color.b = 0.3f; color.a = 0.85f;
    }
    else
    {
      // Bright Red for rejected
      color.r = 0.95f; color.g = 0.15f; color.b = 0.15f; color.a = 0.35f;
    }
  }

  sample_marker_.header.stamp = rclcpp::Clock().now();
  sample_marker_.points.push_back(p);
  sample_marker_.colors.push_back(color);

  // Keep marker buffer from growing unbounded (limit to 500 recent sample spheres)
  if (sample_marker_.points.size() > 500)
  {
    sample_marker_.points.erase(sample_marker_.points.begin(), sample_marker_.points.begin() + 100);
    sample_marker_.colors.erase(sample_marker_.colors.begin(), sample_marker_.colors.begin() + 100);
  }

  viz_pub_->publish(sample_marker_);
}

// =========================================================================
// GMMConstraintSamplerAllocator Implementation
// =========================================================================

GMMConstraintSamplerAllocator::GMMConstraintSamplerAllocator()
{
  node_ = std::make_shared<rclcpp::Node>("gmm_constraint_sampler_plugin_node");

  // Subscribe to deformed GMM topics
  gmm_sub_ = node_->create_subscription<tp_gmm::msg::GaussianMixture>(
      "/gmm/cartesian_space", 1,
      std::bind(&GMMConstraintSamplerAllocator::gmmCallback, this, std::placeholders::_1));

  deformed_gmm_sub_ = node_->create_subscription<tp_gmm::msg::GaussianMixture>(
      "/gmm/deformed_cartesian_space", 1,
      std::bind(&GMMConstraintSamplerAllocator::gmmCallback, this, std::placeholders::_1));

  // Publisher for RViz sample point visualization
  viz_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>(
      "/gmm_sampling_visualization", 10);

  // Publisher for sampling benchmark statistics
  stats_pub_ = node_->create_publisher<std_msgs::msg::String>(
      "/planning_sampling_stats", 10);

  // Background spin thread for topic callbacks
  spin_thread_ = std::thread([this]() {
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node_);
    while (running_ && rclcpp::ok())
    {
      executor.spin_some(std::chrono::milliseconds(50));
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  });

  RCLCPP_INFO(getLogger(), "GMMConstraintSamplerAllocator plugin initialized successfully.");
}

GMMConstraintSamplerAllocator::~GMMConstraintSamplerAllocator()
{
  running_ = false;
  if (spin_thread_.joinable())
  {
    spin_thread_.join();
  }
}

void GMMConstraintSamplerAllocator::gmmCallback(const tp_gmm::msg::GaussianMixture::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(gmm_mutex_);
  latest_gmm_ = msg;
  RCLCPP_INFO(getLogger(), "Received new GMM message with %zu Gaussians on topic.", msg->gaussians.size());
}

bool GMMConstraintSamplerAllocator::canService(
    const planning_scene::PlanningSceneConstPtr& /*scene*/,
    const std::string& group_name,
    const moveit_msgs::msg::Constraints& constr) const
{
  bool match = (constr.name == "gmm_cartesian_ik" ||
                constr.name == "gmm_joint_projected" ||
                constr.name == "gmm_uniform_box");
  RCLCPP_INFO(getLogger(), "[GMMConstraintSamplerAllocator] canService check: group='%s', constraint.name='%s' -> canService=%s",
              group_name.c_str(), constr.name.c_str(), match ? "TRUE" : "FALSE");
  return match;
}

constraint_samplers::ConstraintSamplerPtr GMMConstraintSamplerAllocator::alloc(
    const planning_scene::PlanningSceneConstPtr& scene,
    const std::string& group_name,
    const moveit_msgs::msg::Constraints& constr)
{
  RCLCPP_INFO(getLogger(), "[GMMConstraintSamplerAllocator] alloc() invoked for constraint '%s' on group '%s'!",
              constr.name.c_str(), group_name.c_str());

  SamplingMode mode = SamplingMode::CARTESIAN_IK;
  if (constr.name == "gmm_joint_projected")
  {
    mode = SamplingMode::JOINT_PROJECTED;
  }
  else if (constr.name == "gmm_uniform_box")
  {
    mode = SamplingMode::UNIFORM_BOX;
  }

  tp_gmm::msg::GaussianMixture::SharedPtr gmm_copy;
  {
    std::lock_guard<std::mutex> lock(gmm_mutex_);
    gmm_copy = latest_gmm_;
  }

  if (mode != SamplingMode::UNIFORM_BOX && !gmm_copy)
  {
    RCLCPP_ERROR(getLogger(), "[GMMConstraintSamplerAllocator] Cannot alloc: latest_gmm_ has not been received yet!");
    return nullptr;
  }

  auto sampler = std::make_shared<GMMConstraintSampler>(scene, group_name, gmm_copy, mode, viz_pub_, stats_pub_);
  if (sampler->configure(constr))
  {
    RCLCPP_INFO(getLogger(), "[GMMConstraintSamplerAllocator] GMMConstraintSampler allocated and configured successfully!");
    return sampler;
  }

  RCLCPP_WARN(getLogger(), "[GMMConstraintSamplerAllocator] Failed to configure GMMConstraintSampler. Falling back.");
  return nullptr;
}

} // namespace tp_gmm

PLUGINLIB_EXPORT_CLASS(tp_gmm::GMMConstraintSamplerAllocator, constraint_samplers::ConstraintSamplerAllocator)
