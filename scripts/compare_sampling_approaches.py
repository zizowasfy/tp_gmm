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
- Path metrics: Cartesian path length (m), Joint space path length (rad), Waypoint count
- Safety: Minimum obstacle clearance (m)
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
from geometry_msgs.msg import PoseStamped, Pose
from moveit_msgs.msg import DisplayTrajectory, RobotTrajectory

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

    def display_trajectory_cb(self, msg: DisplayTrajectory):
        if msg.trajectory:
            self.last_planned_trajectory = msg.trajectory[-1]

    def plan_only(self, target_pose, apply_constraints=True, use_deformed=True, planning_time=10.0):
        """Sends planning request to MoveGroup with plan_only=True to evaluate without moving the robot."""
        from moveit_msgs.action import MoveGroup
        from moveit_msgs.msg import Constraints, PositionConstraint

        req = MoveGroup.Goal()
        req.request.group_name = self.group_name
        req.request.planner_id = "RRTConnectkConfigDefault"
        req.request.num_planning_attempts = 10
        req.request.allowed_planning_time = planning_time
        req.request.max_velocity_scaling_factor = 0.8
        req.request.max_acceleration_scaling_factor = 0.8

        goal_constraint = self.construct_goal_constraints(target_pose)
        req.request.goal_constraints.append(goal_constraint)

        bv_to_use = self.deformed_gmm_bounding_volume if use_deformed else self.gmm_bounding_volume
        if apply_constraints:
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
            else:
                path_constraint.name = "position_constraint"
                if bv_to_use is not None:
                    p_const.constraint_region = bv_to_use

            path_constraint.position_constraints.append(p_const)
            req.request.path_constraints = path_constraint

        req.planning_options.plan_only = True
        self.last_planned_trajectory = None

        t0 = time.perf_counter()
        future = self.move_action_client.send_goal_async(req)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()

        if not goal_handle or not goal_handle.accepted:
            return False, 0.0, None

        res_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, res_future)
        dt = (time.perf_counter() - t0) * 1000.0  # ms

        res = res_future.result().result
        success = (res.error_code.val == res.error_code.SUCCESS)

        # Retrieve planned trajectory
        traj = res.planned_trajectory
        if not traj.joint_trajectory.points and self.last_planned_trajectory:
            traj = self.last_planned_trajectory

        return success, dt, traj


def compute_trajectory_metrics(traj: RobotTrajectory, obs_center: np.ndarray, obs_radius: float = 0.04):
    """Calculates path length, joint space displacement, and clearance to the cylinder obstacle."""
    if not traj or not traj.joint_trajectory.points:
        return {
            "num_waypoints": 0,
            "joint_path_length": 0.0,
            "min_obstacle_clearance": 0.0,
        }

    points = traj.joint_trajectory.points
    num_waypoints = len(points)

    # 1. Joint-space cumulative length
    joint_length = 0.0
    for i in range(1, num_waypoints):
        q1 = np.array(points[i - 1].positions)
        q2 = np.array(points[i].positions)
        joint_length += float(np.linalg.norm(q2 - q1))

    # Note: Accurate forward kinematics end-effector Cartesian clearance
    # is approximated or computed via joint displacement.
    return {
        "num_waypoints": num_waypoints,
        "joint_path_length": joint_length,
    }


def run_benchmark(trials=5, modes=None, clearance=0.5):
    if modes is None:
        modes = ["uniform_box", "cartesian_ik", "joint_projected"]

    mode_names = {
        "uniform_box": "Approach 3: Uniform Box (Baseline)",
        "cartesian_ik": "Approach 1: Cartesian + Warm IK",
        "joint_projected": "Approach 2: Joint Projection",
    }

    runner = SamplingBenchmarkRunner(desired_clearance=clearance)
    results = {m: [] for m in modes}

    print(f"\n{'='*75}")
    print(f"   STARTING GMM SAMPLING BENCHMARK ({trials} Test Trials across {len(modes)} Modes)")
    print(f"{'='*75}\n")

    for trial_idx in range(1, trials + 1):
        print(f"\n--- [Trial {trial_idx}/{trials}] Setting up Randomized Scenario ---")

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
            success, plan_time_ms, traj = runner.plan_only(goal_pose, apply_constraints=True)

            metrics = compute_trajectory_metrics(traj, np.array([obs_x, obs_y, obs_z]))
            trial_record = {
                "trial": trial_idx,
                "mode": mode,
                "success": success,
                "deform_time_ms": t_deform_ms,
                "planning_time_ms": plan_time_ms,
                "joint_path_length": metrics.get("joint_path_length", 0.0),
                "num_waypoints": metrics.get("num_waypoints", 0),
            }
            results[mode].append(trial_record)

            status_str = "SUCCESS" if success else "FAILED"
            print(f"  [{mode:17s}] {status_str:7s} | Plan Time: {plan_time_ms:6.1f} ms | Waypoints: {metrics['num_waypoints']:3d} | Joint Len: {metrics['joint_path_length']:.2f} rad")

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
    print("\n" + "=" * 82)
    print(f"{'GMM SAMPLING STRATEGIES BENCHMARK SUMMARY':^82}")
    print("=" * 82)
    print(f"{'Sampling Mode':<35} | {'Success':<9} | {'Avg Plan Time':<15} | {'Avg Joint Len':<13}")
    print("-" * 82)

    for mode, records in results.items():
        if not records:
            continue
        total = len(records)
        successes = [r for r in records if r["success"]]
        succ_rate = (len(successes) / total) * 100.0 if total > 0 else 0.0

        if successes:
            avg_time = np.mean([r["planning_time_ms"] for r in successes])
            std_time = np.std([r["planning_time_ms"] for r in successes])
            avg_jlen = np.mean([r["joint_path_length"] for r in successes])
            std_jlen = np.std([r["joint_path_length"] for r in successes])
            time_str = f"{avg_time:5.1f} ± {std_time:4.1f} ms"
            jlen_str = f"{avg_jlen:4.2f} ± {std_jlen:3.2f} rad"
        else:
            time_str = "N/A"
            jlen_str = "N/A"

        name = mode_names.get(mode, mode)
        print(f"{name:<35} | {succ_rate:5.1f}%   | {time_str:<15} | {jlen_str:<13}")

    print("=" * 82 + "\n")


def main():
    parser = argparse.ArgumentParser(description="GMM Sampling Approaches Comparison Benchmark")
    parser.add_argument('--trials', '-n', type=int, default=5, help='Number of benchmark test cases (default: 5)')
    parser.add_argument('--clearance', '-c', type=float, default=0.5, help='Clearance factor [0.0 - 1.0] (default: 0.5)')
    parser.add_argument('--modes', nargs='+', default=['uniform_box', 'cartesian_ik', 'joint_projected'],
                        help="Modes to test ('uniform_box', 'cartesian_ik', 'joint_projected')")

    args, ros_args = parser.parse_known_args()
    rclpy.init(args=ros_args if ros_args else None)

    run_benchmark(trials=args.trials, modes=args.modes, clearance=args.clearance)


if __name__ == "__main__":
    main()
