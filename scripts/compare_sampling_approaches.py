#!/usr/bin/env python3

"""
Comparison Benchmark Script for GMM Sampling Strategies in MoveIt 2:
- Approach 1: Cartesian GMM Sampling + Warm-Started IK ('cartesian_ik')
- Approach 2: Direct Joint-Space GMM Projection ('joint_projected')
- Approach 3: Baseline Uniform Bounding Box ('uniform_box')

Evaluates each test scenario side-by-side with identical start, goal, obstacle,
and deformed GMM corridor conditions. Records:
- Planning success rate (%)
- Stage-by-stage timing breakdown (ms): TP-GMM reproduction, RL deformation, OMPL search
- Sampling efficiency: Total samples drawn, Valid samples accepted, Acceptance rate (%)
- Path metrics: Joint space path length (rad), Waypoint count
"""

import sys
import os
import time
import math
import json
import argparse
from copy import deepcopy
from pathlib import Path
import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Pose, Point
from std_msgs.msg import String, ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
from moveit_msgs.msg import DisplayTrajectory, RobotTrajectory
from moveit_msgs.srv import GetPositionFK

from ament_index_python.packages import get_package_share_directory
pkg_share = get_package_share_directory('tp_gmm')
sys.path.append(os.path.join(pkg_share, 'scripts'))
sys.path.append(os.path.join(pkg_share, 'include'))

from run_experiments import GazeboExperimentRunner


class SamplingBenchmarkRunner(GazeboExperimentRunner):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        
        self.last_planned_trajectory = None
        self.display_path_sub = self.create_subscription(
            DisplayTrajectory,
            f"{self.namespace}/display_planned_path",
            self.display_trajectory_cb,
            10
        )

        # Real-time sampling statistics subscriber from MoveIt constraint sampler
        self.last_sampling_stats = None
        self.stats_sub = self.create_subscription(
            String,
            "/planning_sampling_stats",
            self.sampling_stats_cb,
            10
        )

        # FK service client for extracting 3D Cartesian paths
        self.fk_client = self.create_client(
            GetPositionFK,
            f"{self.namespace}/compute_fk" if self.namespace else "/compute_fk"
        )

        # Publishers for displaying planned trajectories and simultaneous 3D comparison path ribbons
        self.display_path_pub = self.create_publisher(
            DisplayTrajectory,
            f"{self.namespace}/display_planned_path",
            10
        )
        self.paths_marker_pub = self.create_publisher(
            MarkerArray,
            "/comparison_planned_paths",
            10
        )
        self.sample_marker_pub = self.create_publisher(
            Marker,
            "/gmm_sampling_visualization",
            10
        )

    def display_trajectory_cb(self, msg: DisplayTrajectory):
        if msg.trajectory:
            self.last_planned_trajectory = msg.trajectory[-1]

    def sampling_stats_cb(self, msg: String):
        try:
            self.last_sampling_stats = json.loads(msg.data)
        except Exception:
            pass

    def clear_visualizations(self):
        """Clears previously displayed 3D paths and sample markers in RViz."""
        # Clear 3D paths
        del_paths = MarkerArray()
        m = Marker()
        m.action = Marker.DELETEALL
        del_paths.markers.append(m)
        self.paths_marker_pub.publish(del_paths)

        # Clear sample markers
        del_samples = Marker()
        del_samples.action = Marker.DELETEALL
        self.sample_marker_pub.publish(del_samples)

    def visualize_trajectory(self, traj: RobotTrajectory):
        """Publishes trajectory to /display_planned_path so MoveIt animates the robot in RViz."""
        if not traj or not traj.joint_trajectory.points:
            return
        disp = DisplayTrajectory()
        disp.model_id = self.group_name
        disp.trajectory = [traj]
        self.display_path_pub.publish(disp)

    def get_trajectory_ee_points(self, traj: RobotTrajectory):
        """Extracts 3D Cartesian coordinates of ee_link along the trajectory using /compute_fk."""
        if not traj or not traj.joint_trajectory.points:
            return []

        if not self.fk_client.service_is_ready():
            if not self.fk_client.wait_for_service(timeout_sec=1.0):
                return []

        joint_names = traj.joint_trajectory.joint_names
        points = traj.joint_trajectory.points
        ee_points = []

        # Sample waypoints along trajectory (up to 40 waypoints for responsive rendering)
        step = max(1, len(points) // 40)
        sampled_pts = points[::step]
        if points[-1] not in sampled_pts:
            sampled_pts.append(points[-1])

        for pt in sampled_pts:
            fk_req = GetPositionFK.Request()
            fk_req.header.frame_id = self.frame_id
            fk_req.fk_link_names = [self.ee_link]
            fk_req.robot_state.joint_state.name = joint_names
            fk_req.robot_state.joint_state.position = list(pt.positions)

            future = self.fk_client.call_async(fk_req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=0.2)
            res = future.result()
            if res and res.error_code.val == 1 and res.pose_stamped:
                ee_points.append(res.pose_stamped[0].pose.position)

        return ee_points

    def publish_comparison_paths(self, all_trajectories: dict):
        """Publishes simultaneous 3D line strips for all planned modes on /comparison_planned_paths."""
        marker_array = MarkerArray()

        color_map = {
            "default_unconstrained": (0.9, 0.2, 0.2, 0.95),  # Crimson Red
            "uniform_box":           (1.0, 0.75, 0.0, 0.95), # Amber / Gold
            "cartesian_ik":          (0.05, 0.95, 0.3, 0.95),# Emerald Green
            "joint_projected":       (0.0, 0.85, 1.0, 0.95), # Electric Cyan
        }

        for idx, (mode, traj) in enumerate(all_trajectories.items()):
            if not traj:
                continue
            ee_points = self.get_trajectory_ee_points(traj)
            if len(ee_points) < 2:
                continue

            marker = Marker()
            marker.header.frame_id = self.frame_id
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = f"path_{mode}"
            marker.id = idx + 100
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.scale.x = 0.008  # 8mm line thickness

            r, g, b, a = color_map.get(mode, (0.7, 0.7, 0.7, 0.9))
            marker.color.r = float(r)
            marker.color.g = float(g)
            marker.color.b = float(b)
            marker.color.a = float(a)

            for p in ee_points:
                pt = Point()
                pt.x = p.x
                pt.y = p.y
                pt.z = p.z
                marker.points.append(pt)

            marker_array.markers.append(marker)

        if marker_array.markers:
            self.paths_marker_pub.publish(marker_array)

    def plan_only(self, target_pose, apply_constraints=True, use_deformed=True, planning_time=3.0):
        """Sends planning request to MoveGroup with plan_only=True to evaluate without moving the robot."""
        from moveit_msgs.action import MoveGroup
        from moveit_msgs.msg import Constraints, PositionConstraint

        req = MoveGroup.Goal()
        req.request.group_name = self.group_name
        req.request.planner_id = "RRTConnectkConfigDefault"
        req.request.num_planning_attempts = 1
        req.request.allowed_planning_time = planning_time
        req.request.max_velocity_scaling_factor = 0.8
        req.request.max_acceleration_scaling_factor = 0.8

        goal_constraint = self.construct_goal_constraints(target_pose)
        req.request.goal_constraints.append(goal_constraint)

        bv_to_use = self.deformed_gmm_bounding_volume if use_deformed else self.gmm_bounding_volume
        if apply_constraints and self.sampling_mode != "default_unconstrained":
            path_constraint = Constraints()
            p_const = PositionConstraint()
            p_const.header.frame_id = self.frame_id
            p_const.link_name = self.ee_link
            p_const.weight = 1.0

            if self.sampling_mode == "joint_projected":
                path_constraint.name = "gmm_joint_projected"
                if bv_to_use is not None:
                    p_const.constraint_region = bv_to_use
            elif self.sampling_mode == "cartesian_ik":
                path_constraint.name = "gmm_cartesian_ik"
                if bv_to_use is not None:
                    p_const.constraint_region = bv_to_use
            else:  # uniform_box (Approach 3 baseline)
                path_constraint.name = "gmm_uniform_box"
                if bv_to_use is not None:
                    p_const.constraint_region = bv_to_use

            path_constraint.position_constraints.append(p_const)
            req.request.path_constraints = path_constraint

        req.planning_options.plan_only = True
        self.last_planned_trajectory = None
        self.last_sampling_stats = None

        t0 = time.perf_counter()
        future = self.move_action_client.send_goal_async(req)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()

        if not goal_handle or not goal_handle.accepted:
            return False, 0.0, None, None

        res_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, res_future)
        dt = (time.perf_counter() - t0) * 1000.0  # ms

        # Spin briefly to ensure the final statistics message is received
        for _ in range(10):
            rclpy.spin_once(self, timeout_sec=0.01)
            if self.last_sampling_stats and self.last_sampling_stats.get("final", False):
                break

        res = res_future.result().result
        success = (res.error_code.val == res.error_code.SUCCESS)

        # Retrieve planned trajectory
        traj = res.planned_trajectory
        if not traj.joint_trajectory.points and self.last_planned_trajectory:
            traj = self.last_planned_trajectory

        stats = deepcopy(self.last_sampling_stats)
        return success, dt, traj, stats


def compute_trajectory_metrics(traj: RobotTrajectory, obs_center: np.ndarray, obs_radius: float = 0.04):
    """Calculates path length and waypoint count."""
    if not traj or not traj.joint_trajectory.points:
        return {
            "num_waypoints": 0,
            "joint_path_length": 0.0,
        }

    points = traj.joint_trajectory.points
    num_waypoints = len(points)

    joint_length = 0.0
    for i in range(1, num_waypoints):
        q1 = np.array(points[i - 1].positions)
        q2 = np.array(points[i].positions)
        joint_length += float(np.linalg.norm(q2 - q1))

    return {
        "num_waypoints": num_waypoints,
        "joint_path_length": joint_length,
    }


def run_benchmark(trials=5, modes=None, clearance=0.5, delay=2.0, step=False, replay=False):
    if modes is None:
        modes = ["default_unconstrained", "uniform_box", "cartesian_ik", "joint_projected"]
    else:
        # Normalize mode strings
        normalized = []
        for m in modes:
            m_clean = m.strip().lower()
            if m_clean in ["default", "default_ompl", "unconstrained", "ompl", "default_unconstrained"]:
                normalized.append("default_unconstrained")
            elif m_clean in ["uniform", "uniform_box", "box"]:
                normalized.append("uniform_box")
            elif m_clean in ["cartesian", "cartesian_ik", "ik"]:
                normalized.append("cartesian_ik")
            elif m_clean in ["joint", "joint_projected", "projected"]:
                normalized.append("joint_projected")
            else:
                normalized.append(m_clean)
        modes = normalized

    mode_names = {
        "default_unconstrained": "Default OMPL (Whole Workspace)",
        "uniform_box": "Approach 3: Uniform Box (Baseline)",
        "cartesian_ik": "Approach 1: Cartesian + Warm IK",
        "joint_projected": "Approach 2: Joint Projection",
    }

    runner = SamplingBenchmarkRunner(desired_clearance=clearance)
    results = {m: [] for m in modes}

    print(f"\n{'='*95}")
    print(f"   STARTING GMM SAMPLING BENCHMARK ({trials} Test Trials across {len(modes)} Modes)")
    print(f"{'='*95}\n")

    for trial_idx in range(1, trials + 1):
        print(f"\n--- [Trial {trial_idx}/{trials}] Setting up Randomized Scenario ---")

        # Clear previous markers so this trial starts with a clean visual scene
        runner.clear_visualizations()
        time.sleep(0.1)
        planned_trajectories = {}

        # 1. Sample randomized scene
        start_x = float(np.random.uniform(0.22, 0.35))
        start_y = float(np.random.uniform(-0.18, 0.18))
        start_z = float(np.random.uniform(0.58, 0.68))

        start_pose = PoseStamped()
        start_pose.header.frame_id = runner.frame_id
        start_pose.pose.position.x = start_x
        start_pose.pose.position.y = start_y
        start_pose.pose.position.z = start_z
        start_pose.pose.orientation.w = 0.0
        start_pose.pose.orientation.x = 1.0

        # Move robot to start pose
        runner.plan_and_execute(start_pose, apply_constraints=False)
        time.sleep(0.3)

        # Target cube
        cube_x = float(np.random.uniform(0.38, 0.62))
        cube_y = float(np.random.uniform(-0.35, 0.35))
        cube_z = 0.275
        runner.set_gazebo_pose("cube1", cube_x, cube_y, cube_z)

        # Cylinder obstacle placed midway
        alpha = float(np.random.uniform(0.45, 0.55))
        obs_x = alpha * start_x + (1.0 - alpha) * cube_x
        obs_y = alpha * start_y + (1.0 - alpha) * cube_y
        obs_z = 0.375
        runner.set_gazebo_pose("obstacle", obs_x, obs_y, obs_z)
        runner.update_moveit_planning_scene(obs_x, obs_y, obs_z, obs_radius=0.04, obs_height=0.25)
        time.sleep(0.3)

        # Goal pose (0.05m pre-grasp above cube)
        goal_pose = deepcopy(start_pose)
        goal_pose.pose.position.x = cube_x
        goal_pose.pose.position.y = cube_y
        goal_pose.pose.position.z = cube_z + 0.05 + 0.1

        obs_pose = PoseStamped()
        obs_pose.header.frame_id = runner.frame_id
        obs_pose.pose.position.x = obs_x
        obs_pose.pose.position.y = obs_y
        obs_pose.pose.position.z = obs_z + 0.125
        obs_pose.pose.orientation.w = 1.0

        # 2. Call TP-GMM + RL Policy Deformation once for this trial
        t_deform_start = time.perf_counter()
        deform_ok = runner.call_deform_tpgmm_service(start_pose, goal_pose, obs_pose)
        t_deform_ms = (time.perf_counter() - t_deform_start) * 1000.0

        if not deform_ok:
            print(f"[Trial {trial_idx}] TP-GMM Deformation service failed. Skipping trial.")
            continue

        print(f"Corridor generated in {t_deform_ms:.1f} ms. Evaluating sampling modes:")

        # 3. Test each sampling mode on the exact same scenario
        for mode in modes:
            runner.set_sampling_mode(mode)
            apply_constraints = (mode != "default_unconstrained")
            success, plan_time_ms, traj, stats = runner.plan_only(goal_pose, apply_constraints=apply_constraints)

            if success and traj:
                planned_trajectories[mode] = traj
                runner.visualize_trajectory(traj)

            metrics = compute_trajectory_metrics(traj, np.array([obs_x, obs_y, obs_z]))
            if stats:
                samples_drawn = int(stats.get("samples_drawn", 0))
                samples_accepted = int(stats.get("samples_accepted", 0))
                sampling_rate_pct = (samples_accepted / max(1, samples_drawn)) * 100.0 if samples_drawn > 0 else 0.0
                samples_info = f"Samples: {samples_drawn:4d} (Valid: {samples_accepted:3d}, {sampling_rate_pct:4.1f}%)"
            else:
                samples_drawn = None
                samples_accepted = None
                sampling_rate_pct = None
                samples_info = "Samples:    N/A (Workspace Sampling)"

            trial_record = {
                "trial": trial_idx,
                "mode": mode,
                "success": success,
                "deform_time_ms": t_deform_ms,
                "planning_time_ms": plan_time_ms,
                "samples_drawn": samples_drawn,
                "samples_accepted": samples_accepted,
                "sampling_rate_pct": sampling_rate_pct,
                "joint_path_length": metrics.get("joint_path_length", 0.0),
                "num_waypoints": metrics.get("num_waypoints", 0),
            }
            results[mode].append(trial_record)

            status_str = "SUCCESS" if success else "FAILED"
            print(f"  [{mode:21s}] {status_str:7s} | Plan Time: {plan_time_ms:6.1f} ms | "
                  f"{samples_info} | "
                  f"Waypoints: {metrics['num_waypoints']:3d} | Joint Len: {metrics['joint_path_length']:.2f} rad")

            # Allow user to inspect this mode's plan and samples in RViz
            if success and traj:
                if step:
                    try:
                        input(f"    --> [{mode}] displayed in RViz. Press [Enter] for next mode...")
                    except Exception:
                        pass
                elif delay > 0:
                    time.sleep(delay)

        # 4. Render simultaneous 3D comparison path ribbons in RViz
        if planned_trajectories:
            runner.publish_comparison_paths(planned_trajectories)
            print(f"  --> 3D Path ribbons for all modes published to /comparison_planned_paths.")

        # 5. Optional Replay of all modes at end of trial
        if replay and planned_trajectories:
            print(f"\n  [RViz Replay] Cycling through all planned modes for Trial {trial_idx}:")
            for m in modes:
                t = planned_trajectories.get(m)
                if t:
                    print(f"    -> Animating [{m}] in RViz...")
                    runner.visualize_trajectory(t)
                    time.sleep(delay if delay > 0 else 2.0)

    # 4. Print Summary Comparison Table
    print_comparison_summary(results, mode_names)

    # Save to JSON
    out_dir = Path(pkg_share) / "data"
    out_dir.mkdir(parents=True, exist_ok=True)
    out_file = out_dir / "sampling_comparison_results.json"
    with open(out_file, "w") as f:
        json.dump(results, f, indent=2)
    print(f"\nDetailed benchmark results saved to: {out_file}")

    runner.destroy_node()
    rclpy.shutdown()


def print_comparison_summary(results, mode_names):
    print("\n" + "=" * 110)
    print(f"{'GMM SAMPLING STRATEGIES & OMPL BENCHMARK SUMMARY':^110}")
    print("=" * 110)
    print(f"{'Sampling Mode':<35} | {'Success':<8} | {'Avg Plan Time':<15} | {'Avg Samples Drawn':<18} | {'Accept Rate':<12} | {'Avg Joint Len':<13}")
    print("-" * 110)

    for mode, records in results.items():
        if not records:
            continue
        total = len(records)
        successes = [r for r in records if r["success"]]
        succ_rate = (len(successes) / total) * 100.0 if total > 0 else 0.0

        if successes:
            avg_time = np.mean([r["planning_time_ms"] for r in successes])
            std_time = np.std([r["planning_time_ms"] for r in successes])
            
            valid_samples = [r["samples_drawn"] for r in successes if r["samples_drawn"] is not None]
            if valid_samples:
                avg_samples = np.mean(valid_samples)
                std_samples = np.std(valid_samples)
                samples_str = f"{avg_samples:5.1f} ± {std_samples:4.1f}"

                rates = [r["sampling_rate_pct"] for r in successes if r["sampling_rate_pct"] is not None]
                if rates:
                    avg_rate = np.mean(rates)
                    std_rate = np.std(rates)
                    rate_str = f"{avg_rate:4.1f} ± {std_rate:3.1f} %"
                else:
                    rate_str = "N/A"
            else:
                samples_str = "N/A (Workspace)"
                rate_str = "N/A"

            avg_jlen = np.mean([r["joint_path_length"] for r in successes])
            std_jlen = np.std([r["joint_path_length"] for r in successes])

            time_str = f"{avg_time:5.1f} ± {std_time:4.1f} ms"
            jlen_str = f"{avg_jlen:4.2f} ± {std_jlen:3.2f} rad"
        else:
            time_str = "N/A"
            samples_str = "N/A"
            rate_str = "N/A"
            jlen_str = "N/A"

        name = mode_names.get(mode, mode)
        print(f"{name:<35} | {succ_rate:5.1f}%  | {time_str:<15} | {samples_str:<18} | {rate_str:<12} | {jlen_str:<13}")

    print("=" * 110 + "\n")


def main():
    parser = argparse.ArgumentParser(description="GMM Sampling Approaches Comparison Benchmark")
    parser.add_argument('--trials', '-n', type=int, default=5, help='Number of benchmark test cases (default: 5)')
    parser.add_argument('--clearance', '-c', type=float, default=0.5, help='Clearance factor [0.0 - 1.0] (default: 0.5)')
    parser.add_argument('--modes', nargs='+', default=['default_unconstrained', 'uniform_box', 'cartesian_ik', 'joint_projected'],
                        help="Modes to test ('default_unconstrained', 'uniform_box', 'cartesian_ik', 'joint_projected')")
    parser.add_argument('--delay', '-d', type=float, default=2.0,
                        help="Seconds to pause and view each mode's plan in RViz (default: 2.0s, set 0 to disable)")
    parser.add_argument('--step', action='store_true', default=False,
                        help="Interactive step mode: pauses after each mode plan until [Enter] is pressed")
    parser.add_argument('--replay', action='store_true', default=False,
                        help="Replay all planned mode trajectories sequentially at the end of each trial")

    args, ros_args = parser.parse_known_args()
    rclpy.init(args=ros_args if ros_args else None)

    run_benchmark(trials=args.trials, modes=args.modes, clearance=args.clearance,
                  delay=args.delay, step=args.step, replay=args.replay)


if __name__ == "__main__":
    main()
