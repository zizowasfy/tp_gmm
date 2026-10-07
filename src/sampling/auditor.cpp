#include <rclcpp/rclcpp.hpp>
#include <moveit/robot_model_loader/robot_model_loader.hpp>
#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/robot_state/conversions.hpp>
#include <moveit/robot_trajectory/robot_trajectory.hpp>
#include <tp_gmm/srv/audit_trajectory.hpp>
#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("sampling_auditor",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  robot_model_loader::RobotModelLoader loader(node);
  auto model = loader.getModel();
  if (!model) throw std::runtime_error("No robot model for trajectory audit");
  auto service = node->create_service<tp_gmm::srv::AuditTrajectory>("/sampling_study/audit",
    [model](const tp_gmm::srv::AuditTrajectory::Request::SharedPtr req,
            tp_gmm::srv::AuditTrajectory::Response::SharedPtr res) {
      try
      {
        const auto* group = model->getJointModelGroup(req->group_name);
        if (!group || !model->hasLinkModel(req->link_name) || !std::isfinite(req->joint_step) ||
            req->joint_step < 0.001 || req->joint_step > 0.05 || req->scene.is_diff)
          throw std::invalid_argument("Invalid audit group/link/step or incomplete scene");
        planning_scene::PlanningScene scene(model);
        scene.setPlanningSceneMsg(req->scene);
        moveit::core::RobotState start(scene.getCurrentState());
        moveit::core::robotStateMsgToRobotState(req->trajectory_start, start);
        const auto& points = req->trajectory.joint_trajectory.points;
        if (points.size() < 2) throw std::invalid_argument("Trajectory has fewer than two points");
        for (const auto& p : points)
        {
          if (p.positions.size() != req->trajectory.joint_trajectory.joint_names.size())
            throw std::invalid_argument("Malformed trajectory positions");
          for (double v : p.positions)
            if (!std::isfinite(v)) throw std::invalid_argument("Nonfinite trajectory");
        }
        robot_trajectory::RobotTrajectory trajectory(model, req->group_name);
        trajectory.setRobotTrajectoryMsg(start, req->trajectory);
        kinematic_constraints::KinematicConstraintSet constraints(model);
        constraints.add(req->constraints, scene.getTransforms());
        moveit::core::RobotState state(start);
        double length = 0, ee_length = 0, clearance = std::numeric_limits<double>::infinity();
        unsigned samples = 0, invalid = 0, collisions = 0, violations = 0;
        Eigen::Vector3d previous;
        std::ostringstream xyz;
        xyz << std::setprecision(12);
        for (size_t i = 0; i < trajectory.getWayPointCount(); ++i)
        {
          const auto& a = trajectory.getWayPoint(i ? i-1 : 0);
          const auto& b = trajectory.getWayPoint(i);
          const double distance = a.distance(b, group);
          length += distance;
          const unsigned steps = std::max(1u, static_cast<unsigned>(std::ceil(distance / req->joint_step)));
          if (samples + steps > 100000) throw std::invalid_argument("Audit exceeds sample limit");
          for (unsigned j = i ? 1 : 0; j <= (i ? steps : 0); ++j)
          {
            a.interpolate(b, double(j)/steps, state);
            state.update();
            const Eigen::Vector3d x = state.getGlobalLinkTransform(req->link_name).translation();
            if (samples) { ee_length += (x-previous).norm(); xyz << ','; }
            previous = x;
            xyz << '[' << x.x() << ',' << x.y() << ',' << x.z() << ']';
            const bool collision = scene.isStateColliding(state, req->group_name, false);
            const bool constrained = constraints.decide(state).satisfied;
            ++samples; collisions += collision; violations += !constrained;
            invalid += collision || !constrained || !state.satisfiesBounds(group) || !scene.isStateFeasible(state, false);
            clearance = std::min(clearance, std::max(0.0, scene.distanceToCollision(state)));
          }
        }
        std::ostringstream out;
        out << std::setprecision(12) << "{\"audit_samples\":" << samples
            << ",\"audit_invalid_samples\":" << invalid << ",\"collision_samples\":" << collisions
            << ",\"constraint_violations\":" << violations << ",\"joint_path_length\":" << length
            << ",\"ee_path_length_m\":" << ee_length << ",\"min_world_clearance_m\":";
        if (std::isfinite(clearance)) out << clearance; else out << "null";
        out << ",\"trajectory_duration_s\":" << trajectory.getDuration()
            << ",\"ee_path\":[" << xyz.str() << "]}";
        res->success = true; res->json = out.str();
      }
      catch (const std::exception& ex) { res->success = false; res->message = ex.what(); }
    });
  rclcpp::spin(node);
  rclcpp::shutdown();
}
