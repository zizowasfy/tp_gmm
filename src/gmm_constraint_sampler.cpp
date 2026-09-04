#include <tp_gmm/gmm_constraint_sampler.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/logging.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/color_rgba.hpp>

#include <cmath>
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
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr viz_pub)
  : constraint_samplers::ConstraintSampler(scene, group_name)
  , mode_(mode)
  , gmm_msg_(std::move(gmm_msg))
  , viz_pub_(std::move(viz_pub))
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

bool GMMConstraintSampler::configure(const moveit_msgs::msg::Constraints& constr)
{
  clear();

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
    RCLCPP_INFO(getLogger(), "Configured in Approach 2: Direct Joint-Space GMM Projection mode.");
  }
  else
  {
    mode_ = SamplingMode::CARTESIAN_IK;
    sampler_name_ = "GMMConstraintSampler_CartesianIK";
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

  if (!gmm_msg_ || gmm_msg_->gaussians.empty())
  {
    RCLCPP_WARN(getLogger(), "No GMM data available in GMMConstraintSampler. Cannot configure.");
    return false;
  }

  if (!parseGMMMessage(*gmm_msg_))
  {
    RCLCPP_ERROR(getLogger(), "Failed to parse GaussianMixture message.");
    return false;
  }

  // Determine nominal orientation
  Eigen::Quaterniond default_orientation(1.0, 0.0, 0.0, 0.0); // Default downward orientation
  if (!constr.orientation_constraints.empty())
  {
    const auto& q_msg = constr.orientation_constraints[0].orientation;
    default_orientation = Eigen::Quaterniond(q_msg.w, q_msg.x, q_msg.y, q_msg.z);
    default_orientation.normalize();
  }

  for (auto& comp : components_)
  {
    comp.orientation = default_orientation;
  }

  precomputeNominalJointStatesAndJacobians();

  is_valid_ = !components_.empty();
  RCLCPP_INFO(getLogger(), "GMMConstraintSampler successfully configured with %zu Gaussian components.",
              components_.size());
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

      comp.cov_cartesian.setZero();
      for (size_t r = 0; r < 3; ++r)
      {
        for (size_t c = 0; c < 3; ++c)
        {
          size_t idx = (r + 1) * 4 + (c + 1);
          if (idx < g.covariances.size())
            comp.cov_cartesian(r, c) = g.covariances[idx];
        }
      }
    }
    else if (dim >= 3)
    {
      // 3D GMM: [x, y, z]
      comp.mean = Eigen::Vector3d(g.means[0], g.means[1], g.means[2]);

      comp.cov_cartesian.setZero();
      for (size_t r = 0; r < 3; ++r)
      {
        for (size_t c = 0; c < 3; ++c)
        {
          size_t idx = r * dim + c;
          if (idx < g.covariances.size())
            comp.cov_cartesian(r, c) = g.covariances[idx];
        }
      }
    }
    else
    {
      RCLCPP_ERROR(getLogger(), "Gaussian dimension %zu is less than 3! Cannot use for 3D sampling.", dim);
      return false;
    }

    // Ensure symmetry and positive definiteness via small diagonal regularization
    comp.cov_cartesian = 0.5 * (comp.cov_cartesian + comp.cov_cartesian.transpose());
    comp.cov_cartesian += 1e-5 * Eigen::Matrix3d::Identity();

    // Cholesky decomposition of 3x3 Cartesian covariance: Sigma = L * L^T
    Eigen::LLT<Eigen::Matrix3d> llt(comp.cov_cartesian);
    if (llt.info() == Eigen::Success)
    {
      comp.cholesky_L_x = llt.matrixL();
    }
    else
    {
      // Extra regularization if ill-conditioned
      Eigen::Matrix3d reg = comp.cov_cartesian + 1e-4 * Eigen::Matrix3d::Identity();
      Eigen::LLT<Eigen::Matrix3d> llt_reg(reg);
      comp.cholesky_L_x = llt_reg.matrixL();
    }

    double w = (i < msg.weights.size()) ? static_cast<double>(msg.weights[i]) : 1.0;
    if (w <= 0.0) w = 1e-4;

    weights_.push_back(w);
    components_.push_back(comp);
  }

  // Normalize weights
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

    // Approach 2: Compute Joint-Space Covariance via First-Order Jacobian Projection
    if (mode_ == SamplingMode::JOINT_PROJECTED)
    {
      temp_state.setJointGroupPositions(jmg_, comp.nominal_joint_positions);
      temp_state.update();

      // Compute geometric Jacobian (6 x num_joints)
      Eigen::MatrixXd J;
      temp_state.getJacobian(jmg_, ee_link_, Eigen::Vector3d::Zero(), J);

      // Position Jacobian (3 x num_joints)
      Eigen::MatrixXd J_pos = J.topRows(3);

      // Damped Moore-Penrose pseudo-inverse: J_dag = J^T * (J * J^T + lambda^2 * I)^(-1)
      double lambda = 0.01;
      Eigen::Matrix3d JJt = J_pos * J_pos.transpose() + (lambda * lambda) * Eigen::Matrix3d::Identity();
      Eigen::MatrixXd J_dag = J_pos.transpose() * JJt.inverse();

      // Null space projection matrix: N = (I - J_dag * J_pos)
      Eigen::MatrixXd I_n = Eigen::MatrixXd::Identity(num_joints, num_joints);
      Eigen::MatrixXd N = I_n - J_dag * J_pos;

      // Joint covariance: Sigma_q = J_dag * Sigma_x * J_dag^T + sigma_null^2 * (N * N^T) + eps * I
      double sigma_null = 0.05; // 0.05 rad variance along null space
      comp.cov_joint = J_dag * comp.cov_cartesian * J_dag.transpose()
                       + (sigma_null * sigma_null) * (N * N.transpose())
                       + 1e-5 * I_n;

      // Ensure exact symmetry
      comp.cov_joint = 0.5 * (comp.cov_joint + comp.cov_joint.transpose());

      // Cholesky decomposition of joint covariance (num_joints x num_joints)
      Eigen::LLT<Eigen::MatrixXd> llt_q(comp.cov_joint);
      if (llt_q.info() == Eigen::Success)
      {
        comp.cholesky_L_q = llt_q.matrixL();
      }
      else
      {
        Eigen::MatrixXd reg_q = comp.cov_joint + 1e-4 * I_n;
        Eigen::LLT<Eigen::MatrixXd> llt_q_reg(reg_q);
        comp.cholesky_L_q = llt_q_reg.matrixL();
      }
    }
  }
}

bool GMMConstraintSampler::sample(moveit::core::RobotState& state,
                                  const moveit::core::RobotState& /*reference_state*/,
                                  unsigned int max_attempts)
{
  if (!is_valid_ || components_.empty()) return false;

  unsigned int num_joints = jmg_->getVariableCount();

  for (unsigned int attempt = 0; attempt < max_attempts; ++attempt)
  {
    ++samples_drawn_;

    // 1. Discrete component selection k ~ Categorical(weights)
    int k = component_dist_(rng_);
    const auto& comp = components_[k];

    if (mode_ == SamplingMode::CARTESIAN_IK)
    {
      // =====================================================================
      // Approach 1: Cartesian GMM Sampling + Warm-Started IK
      // =====================================================================
      // Draw standard normal vector z ~ N(0, I_3)
      Eigen::Vector3d z(normal_dist_(rng_), normal_dist_(rng_), normal_dist_(rng_));

      // Truncate at 3-sigma (confidence boundary)
      if (z.norm() > 3.0)
      {
        z = z.normalized() * 3.0;
      }

      // Compute Cartesian sample: x = mu_k + L_x * z
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
          return true;
        }
      }
      publishVisualSample(pos, false);
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

      // Draw standard normal vector w ~ N(0, I_n)
      Eigen::VectorXd w(num_joints);
      for (unsigned int j = 0; j < num_joints; ++j)
      {
        w(j) = normal_dist_(rng_);
      }

      // Truncate at 99% Chi-squared confidence radius (for n=7, sqrt(chi^2_7(0.99)) ~ 4.29)
      double max_norm = std::sqrt(18.48);
      if (w.norm() > max_norm)
      {
        w = w.normalized() * max_norm;
      }

      // Compute joint sample: q = q_bar_k + L_q * w
      Eigen::VectorXd q_nominal(num_joints);
      for (unsigned int j = 0; j < num_joints; ++j)
      {
        q_nominal(j) = comp.nominal_joint_positions[j];
      }

      Eigen::VectorXd q_sample = q_nominal + comp.cholesky_L_q * w;

      // Enforce physical robot joint bounds
      state.setJointGroupPositions(jmg_, q_sample.data());
      state.enforceBounds(jmg_);
      state.update();

      Eigen::Vector3d pos = state.getGlobalLinkTransform(ee_link_).translation();

      if (!scene_->isStateColliding(state, jmg_->getName()))
      {
        ++samples_accepted_;
        publishVisualSample(pos, true);
        return true;
      }
      publishVisualSample(pos, false);
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
  if (accepted)
  {
    // Vibrant green for valid accepted samples
    color.r = 0.1f;
    color.g = 0.9f;
    color.b = 0.3f;
    color.a = 0.8f;
  }
  else
  {
    // Red/orange for collision/IK rejected samples
    color.r = 0.9f;
    color.g = 0.2f;
    color.b = 0.1f;
    color.a = 0.3f;
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
    const std::string& /*group_name*/,
    const moveit_msgs::msg::Constraints& constr) const
{
  // Allocate if the constraint name requests GMM Cartesian IK or Joint Projection
  return (constr.name == "gmm_cartesian_ik" || constr.name == "gmm_joint_projected");
}

constraint_samplers::ConstraintSamplerPtr GMMConstraintSamplerAllocator::alloc(
    const planning_scene::PlanningSceneConstPtr& scene,
    const std::string& group_name,
    const moveit_msgs::msg::Constraints& constr)
{
  SamplingMode mode = (constr.name == "gmm_joint_projected") ?
                      SamplingMode::JOINT_PROJECTED : SamplingMode::CARTESIAN_IK;

  tp_gmm::msg::GaussianMixture::SharedPtr gmm_copy;
  {
    std::lock_guard<std::mutex> lock(gmm_mutex_);
    gmm_copy = latest_gmm_;
  }

  auto sampler = std::make_shared<GMMConstraintSampler>(scene, group_name, gmm_copy, mode, viz_pub_);
  if (sampler->configure(constr))
  {
    return sampler;
  }

  RCLCPP_WARN(getLogger(), "Failed to configure GMMConstraintSampler. Falling back.");
  return nullptr;
}

} // namespace tp_gmm

PLUGINLIB_EXPORT_CLASS(tp_gmm::GMMConstraintSamplerAllocator, constraint_samplers::ConstraintSamplerAllocator)
