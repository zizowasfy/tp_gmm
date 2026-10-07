#include "sampler.hpp"
#include <moveit/robot_state/conversions.hpp>
#include <moveit/robot_trajectory/robot_trajectory.hpp>

namespace tp_gmm
{
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
    std::vector<Eigen::Vector3d> ee_points;
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
      path.ns = session->config.mode + (session->config.proposal == "gmm" ? "" : "/" + session->config.proposal); path.type = path.LINE_STRIP;
      path.pose.orientation.w = 1; path.scale.x = 0.005; path.color.a = 1;
      path.color.r = session->config.mode == "uniform" ? 1 : 0.1;
      path.color.g = session->config.mode == "cartesian_ik" ? 1 : 0.3;
      path.color.b = session->config.mode == "joint_projected" ? 1 : 0.1;
      if (session->config.proposal != "gmm") { path.color.r = 1; path.color.g = .2; path.color.b = .8; }
      const Eigen::Isometry3d gmm_from_model = scene->getFrameTransform(session->config.model.header.frame_id).inverse();
      double reference_sum = 0., reference_max = 0.;
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
          ee_points.push_back(gmm_from_model * x);
          if (session->reference)
          {
            const double d = session->reference->distance(gmm_from_model * x);
            reference_sum += d; reference_max = std::max(reference_max, d);
          }
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
      if (session->reference)
      {
        metrics["reference_deviation_mean_m"] = reference_sum / metrics["path_validation_samples"];
        metrics["reference_deviation_max_m"] = reference_max;
      }
      if (session->config.visualize_path)
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
    json << ",\"proposal\":\"" << session->config.proposal << "\"";
    json << ",\"gmr_stddev\":" << session->config.gmr_stddev
         << ",\"gmr_cutoff\":" << session->config.gmr_cutoff
         << ",\"gmr_fraction\":" << session->config.gmr_fraction
         << ",\"uniform_fraction\":" << session->config.uniform_fraction
         << ",\"cartesian_fraction\":" << session->config.cartesian_fraction;
    json << ",\"corridor_mode\":\"" << session->config.corridor_mode << "\",\"component_cutoffs\":[";
    for (size_t k = 0; k < session->cutoffs.size(); ++k) { if (k) json << ','; json << session->cutoffs[k]; }
    json << "],\"weights\":[";
    for (size_t k = 0; k < session->weights.size(); ++k) { if (k) json << ','; json << session->weights[k]; }
    json << "],\"ee_path_frame\":\"" << session->config.model.header.frame_id << "\",\"ee_path\":[";
    for (size_t i = 0; i < ee_points.size(); ++i)
    {
      if (i) json << ',';
      json << '[' << ee_points[i].x() << ',' << ee_points[i].y() << ',' << ee_points[i].z() << ']';
    }
    json << "]}";
    publishMarkers();
    res.success = true; res.json = json.str(); res.message = "Snapshot after planning; valid targets are not RRT vertices";
    if (req.release) { std::lock_guard<std::mutex> lock(mutex_); sessions_.erase(req.request_id); }
  }
  catch (const std::exception& ex) { res.success = false; res.message = ex.what(); }
}

}  // namespace tp_gmm
