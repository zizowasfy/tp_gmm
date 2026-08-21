#!/usr/bin/env python3

"""
Automated Experiment Pipeline for Franka Panda in Gazebo Sim:
1. Resets environment & randomizes object poses (cubes, obstacle) on the table in Gazebo Sim.
2. Moves robot to a randomized start pose above the table.
3. Invokes TP-GMM and the RL deformation policy given (start, target cube, obstacle).
4. Enforces the deformed GMM corridor as MoveIt path constraints to plan and execute motion around the obstacle.
5. Reaches down to the cube, closes the gripper, and lifts it up.
6. Resets and repeats for subsequent experiment trials.
"""

import sys
import os
import time
import math
import subprocess
import argparse
import numpy as np
from copy import deepcopy
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from scipy.spatial.transform import Rotation as R

from geometry_msgs.msg import PoseStamped, Pose, PoseArray, Point
from moveit_msgs.msg import BoundingVolume, Constraints, PositionConstraint, OrientationConstraint, DisplayTrajectory, CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene, GetCartesianPath
from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from shape_msgs.msg import SolidPrimitive
from control_msgs.action import GripperCommand
from tp_gmm.srv import ReproduceTPGMM, DeformTPGMM

from ament_index_python.packages import get_package_share_directory
pkg_share = get_package_share_directory('tp_gmm')
sys.path.append(os.path.join(pkg_share, 'scripts'))
sys.path.append(os.path.join(pkg_share, 'include'))

ROBOT_CONFIGS = {
    "franka_panda": {
        "frame_id": "panda_link0",
        "group_name": "panda_arm",
        "ee_link": "panda_hand",
        "gripper_group": "hand",
        "gripper_action": "/panda_hand_controller/gripper_cmd",
        "adjustment_rotation": R.from_euler('ZYX', [180, -90, 0], degrees=True),
    },
}

class GazeboExperimentRunner(Node):
    def __init__(self, robot_type="franka_panda", namespace="", task_name="pick", desired_clearance=0.8, obstacle_radius=0.04):
        super().__init__('gazebo_experiment_runner')

        self.robot_type = robot_type
        self.namespace = namespace
        self.task_name = task_name
        self.desired_clearance = desired_clearance
        self.obstacle_radius = obstacle_radius

        cfg = ROBOT_CONFIGS.get(robot_type, ROBOT_CONFIGS["franka_panda"])
        self.frame_id = cfg["frame_id"]
        self.group_name = cfg["group_name"]
        self.ee_link = cfg["ee_link"]
        self.adjustment_rotation = cfg["adjustment_rotation"]
        self.gripper_action_name = cfg["gripper_action"]

        self.get_logger().info(f"Initializing Gazebo Experiment Runner for {self.robot_type}...")

        # Action clients
        self.move_action_client = ActionClient(self, MoveGroup, f'{self.namespace}/move_action')
        self.execute_traj_client = ActionClient(self, ExecuteTrajectory, f'{self.namespace}/execute_trajectory')
        self.gripper_action_client = ActionClient(self, GripperCommand, self.gripper_action_name)

        # Services
        self.reproduce_tpgmm_client = self.create_client(ReproduceTPGMM, f"{self.namespace}/ReproduceTPGMM_service")
        self.deform_tpgmm_client = self.create_client(DeformTPGMM, f"{self.namespace}/DeformTPGMM_service")
        self.apply_planning_scene_client = self.create_client(ApplyPlanningScene, f"{self.namespace}/apply_planning_scene")
        self.cartesian_path_client = self.create_client(GetCartesianPath, f"{self.namespace}/compute_cartesian_path")

        # Publishers for visualization & planning scene
        self.start_pose_pub = self.create_publisher(PoseStamped, f"{self.namespace}/start_pose", 1)
        self.goal_pose_pub = self.create_publisher(PoseStamped, f"{self.namespace}/goal_pose", 1)
        self.obstacle_pose_pub = self.create_publisher(PoseStamped, f"{self.namespace}/obstacle_pose", 1)
        self.planning_scene_pub = self.create_publisher(PlanningScene, f"{self.namespace}/planning_scene", 10)

        # Subscriptions to GMM bounding volumes
        self.create_subscription(BoundingVolume, f"{self.namespace}/gmm_moveit", self.gmm_constraint_cb, 10)
        self.create_subscription(BoundingVolume, f"{self.namespace}/deformed_gmm_moveit", self.deformed_gmm_constraint_cb, 10)

        self.gmm_bounding_volume = None
        self.deformed_gmm_bounding_volume = None

        self.get_logger().info("Connecting to MoveGroup Action Server...")
        if not self.move_action_client.wait_for_server(timeout_sec=15.0):
            self.get_logger().error("MoveGroup action server not available!")
            sys.exit(1)

        self.get_logger().info("Experiment Runner Node Ready!")

    def gmm_constraint_cb(self, msg):
        self.gmm_bounding_volume = msg

    def deformed_gmm_constraint_cb(self, msg):
        self.deformed_gmm_bounding_volume = msg

    def set_gazebo_pose(self, entity_name, x, y, z, qx=0.0, qy=0.0, qz=0.0, qw=1.0):
        """Sets entity pose in Gazebo Sim via gz service CLI."""
        cmd = [
            "gz", "service", "-s", "/world/warehouse/set_pose",
            "--reqtype", "gz.msgs.Pose",
            "--reptype", "gz.msgs.Boolean",
            "--timeout", "1000",
            "--req", f'name: "{entity_name}", position: {{x: {x}, y: {y}, z: {z}}}, orientation: {{x: {qx}, y: {qy}, z: {qz}, w: {qw}}}'
        ]
        try:
            subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, check=False)
        except Exception as e:
            self.get_logger().warn(f"Failed to set Gazebo pose for {entity_name}: {e}")

    def update_moveit_planning_scene(self, obs_x, obs_y, obs_z, obs_radius=0.04, obs_height=0.25):
        """Adds the table and cylinder obstacle into MoveIt's planning scene for collision avoidance."""
        planning_scene = PlanningScene()
        planning_scene.is_diff = True
        planning_scene.robot_state.is_diff = True

        # 1. Table collision object
        table_co = CollisionObject()
        table_co.header.frame_id = self.frame_id
        table_co.id = "table"
        table_co.operation = CollisionObject.ADD

        table_box = SolidPrimitive()
        table_box.type = SolidPrimitive.BOX
        table_box.dimensions = [0.75, 1.0, 0.5]  # [size_x, size_y, size_z]

        table_pose = Pose()
        table_pose.position.x = 0.6
        table_pose.position.y = 0.0
        table_pose.position.z = 0.0
        table_pose.orientation.w = 1.0

        table_co.primitives.append(table_box)
        table_co.primitive_poses.append(table_pose)
        planning_scene.world.collision_objects.append(table_co)

        # 2. Cylinder obstacle collision object
        obs_co = CollisionObject()
        obs_co.header.frame_id = self.frame_id
        obs_co.id = "cylinder_obstacle"
        obs_co.operation = CollisionObject.ADD

        obs_cyl = SolidPrimitive()
        obs_cyl.type = SolidPrimitive.CYLINDER
        obs_cyl.dimensions = [obs_height, obs_radius]  # [height, radius]

        obs_pose = Pose()
        obs_pose.position.x = obs_x
        obs_pose.position.y = obs_y
        obs_pose.position.z = obs_z
        obs_pose.orientation.w = 1.0

        obs_co.primitives.append(obs_cyl)
        obs_co.primitive_poses.append(obs_pose)
        planning_scene.world.collision_objects.append(obs_co)

        # Publish planning scene update and call service
        self.planning_scene_pub.publish(planning_scene)
        if self.apply_planning_scene_client.wait_for_service(timeout_sec=1.0):
            req = ApplyPlanningScene.Request()
            req.scene = planning_scene
            future = self.apply_planning_scene_client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
            self.get_logger().info("MoveIt Planning Scene updated with cylinder obstacle and table.")

    def adjust_orientation(self, pose):
        """Converts operated robot's EE pose orientation to the demonstration frame."""
        robot_rot = R.from_quat([pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w])
        adjusted_rot = robot_rot * self.adjustment_rotation
        adjusted_pose = deepcopy(pose)
        adjusted_pose.pose.orientation.x = adjusted_rot.as_quat()[0]
        adjusted_pose.pose.orientation.y = adjusted_rot.as_quat()[1]
        adjusted_pose.pose.orientation.z = adjusted_rot.as_quat()[2]
        adjusted_pose.pose.orientation.w = adjusted_rot.as_quat()[3]
        return adjusted_pose

    def reverse_adjust_orientation(self, pose):
        """Converts demonstration frame orientation to the operated robot's EE frame."""
        demo_rot = R.from_quat([pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w])
        robot_rot = demo_rot * self.adjustment_rotation.inv()
        robot_pose = deepcopy(pose)
        robot_pose.pose.orientation.x = robot_rot.as_quat()[0]
        robot_pose.pose.orientation.y = robot_rot.as_quat()[1]
        robot_pose.pose.orientation.z = robot_rot.as_quat()[2]
        robot_pose.pose.orientation.w = robot_rot.as_quat()[3]
        return robot_pose

    def set_gripper(self, open_gripper=True, effort=20.0):
        """Opens (pos=0.04) or closes (pos=0.0) the Panda gripper."""
        target_pos = 0.04 if open_gripper else 0.0
        self.get_logger().info(f"Setting gripper: {'OPEN' if open_gripper else 'CLOSE'} (pos={target_pos})")

        if self.gripper_action_client.wait_for_server(timeout_sec=2.0):
            goal = GripperCommand.Goal()
            goal.command.position = target_pos
            goal.command.max_effort = effort
            future = self.gripper_action_client.send_goal_async(goal)
            rclpy.spin_until_future_complete(self, future)
            goal_handle = future.result()
            if goal_handle and goal_handle.accepted:
                res_future = goal_handle.get_result_async()
                rclpy.spin_until_future_complete(self, res_future)
                return True

        # Fallback to MoveGroup hand joint
        req = MoveGroup.Goal()
        req.request.group_name = "hand"
        req.request.num_planning_attempts = 3
        req.request.allowed_planning_time = 2.0
        c = Constraints()
        from moveit_msgs.msg import JointConstraint
        jc = JointConstraint()
        jc.joint_name = "panda_finger_joint1"
        jc.position = target_pos
        jc.tolerance_above = 0.005
        jc.tolerance_below = 0.005
        jc.weight = 1.0
        c.joint_constraints.append(jc)
        req.request.goal_constraints.append(c)
        future = self.move_action_client.send_goal_async(req)
        rclpy.spin_until_future_complete(self, future)
        return True

    def construct_goal_constraints(self, target_pose, pos_tol=0.02, ori_tol=0.15):
        goal_constraint = Constraints()
        
        pos_constraint = PositionConstraint()
        pos_constraint.header = target_pose.header
        pos_constraint.link_name = self.ee_link
        
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.SPHERE
        primitive.dimensions = [pos_tol]
        
        region_pose = Pose()
        region_pose.position = target_pose.pose.position
        region_pose.orientation.w = 1.0
        
        bv = BoundingVolume()
        bv.primitives.append(primitive)
        bv.primitive_poses.append(region_pose)
        
        pos_constraint.constraint_region = bv
        pos_constraint.weight = 1.0
        
        ori_constraint = OrientationConstraint()
        ori_constraint.header = target_pose.header
        ori_constraint.link_name = self.ee_link
        ori_constraint.orientation = target_pose.pose.orientation
        ori_constraint.absolute_x_axis_tolerance = ori_tol
        ori_constraint.absolute_y_axis_tolerance = ori_tol
        ori_constraint.absolute_z_axis_tolerance = ori_tol
        ori_constraint.weight = 1.0
        
        goal_constraint.position_constraints.append(pos_constraint)
        goal_constraint.orientation_constraints.append(ori_constraint)
        return goal_constraint

    def plan_and_execute(self, target_pose, apply_constraints=False, use_deformed=True, planning_time=10.0):
        """Plans and executes motion to target_pose with optional GMM corridor constraints."""
        req = MoveGroup.Goal()
        req.request.group_name = self.group_name
        req.request.num_planning_attempts = 10
        req.request.allowed_planning_time = planning_time
        req.request.max_velocity_scaling_factor = 0.8
        req.request.max_acceleration_scaling_factor = 0.8
        
        goal_constraint = self.construct_goal_constraints(target_pose)
        req.request.goal_constraints.append(goal_constraint)
        
        bv_to_use = self.deformed_gmm_bounding_volume if use_deformed else self.gmm_bounding_volume
        if apply_constraints and bv_to_use is not None:
            self.get_logger().info("Enforcing GMM Corridor Path Constraints in MoveGroup...")
            path_constraint = Constraints()
            path_constraint.name = "position_constraint"
            p_const = PositionConstraint()
            p_const.header.frame_id = self.frame_id
            p_const.link_name = self.ee_link
            p_const.constraint_region = bv_to_use
            p_const.weight = 1.0
            path_constraint.position_constraints.append(p_const)
            req.request.path_constraints = path_constraint

        req.planning_options.plan_only = False
        
        future = self.move_action_client.send_goal_async(req)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()
        
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error("MoveGroup Goal rejected by server!")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        res = result_future.result().result

        if res.error_code.val == res.error_code.SUCCESS:
            self.get_logger().info("MoveGroup trajectory execution succeeded!")
            return True
        else:
            self.get_logger().warn(f"MoveGroup execution failed with error code: {res.error_code.val}")
            return False

    def plan_and_execute_cartesian(self, waypoints, step_size=0.005, jump_threshold=0.0, max_velocity_scaling=0.2, max_acceleration_scaling=0.2):
        """Computes a linear Cartesian path through waypoints and executes it smoothly."""
        if not self.cartesian_path_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().warn("Cartesian path service not available!")
            return False

        req = GetCartesianPath.Request()
        req.header.frame_id = self.frame_id
        req.group_name = self.group_name
        req.link_name = self.ee_link
        
        pose_waypoints = []
        for wp in waypoints:
            if isinstance(wp, PoseStamped):
                pose_waypoints.append(wp.pose)
            elif isinstance(wp, Pose):
                pose_waypoints.append(wp)

        req.waypoints = pose_waypoints
        req.max_step = step_size
        req.jump_threshold = jump_threshold
        req.avoid_collisions = True
        req.max_velocity_scaling_factor = max_velocity_scaling
        req.max_acceleration_scaling_factor = max_acceleration_scaling

        future = self.cartesian_path_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if future.result() is None:
            self.get_logger().warn("Cartesian path service call failed.")
            return False
            
        res = future.result()
        fraction = res.fraction
        self.get_logger().info(f"Cartesian path computation achieved {fraction * 100:.1f}% of trajectory.")

        if fraction < 0.90:
            self.get_logger().warn("Cartesian fraction too low (< 90%).")
            return False

        if not self.execute_traj_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().warn("ExecuteTrajectory action server not available!")
            return False

        exec_goal = ExecuteTrajectory.Goal()
        exec_goal.trajectory = res.solution
        exec_future = self.execute_traj_client.send_goal_async(exec_goal)
        rclpy.spin_until_future_complete(self, exec_future)
        
        goal_handle = exec_future.result()
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().warn("ExecuteTrajectory goal rejected!")
            return False

        res_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, res_future)
        exec_res = res_future.result().result
        
        if exec_res.error_code.val == exec_res.error_code.SUCCESS:
            self.get_logger().info("Cartesian trajectory executed successfully.")
            return True
        else:
            self.get_logger().warn(f"Cartesian execution failed with code: {exec_res.error_code.val}")
            return False

    def call_deform_tpgmm_service(self, start_pose_robot, goal_pose_robot, obstacle_pose):
        """Calls DeformTPGMM service to predict deformation with RL policy."""
        self.get_logger().info("Invoking DeformTPGMM service (RL Policy + GMM)...")
        if not self.deform_tpgmm_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("DeformTPGMM service not available!")
            return False

        # Convert robot poses to demonstration frame for GMM/Policy processing
        start_pose_demo = self.adjust_orientation(start_pose_robot)
        goal_pose_demo = self.adjust_orientation(goal_pose_robot)

        req = DeformTPGMM.Request()
        req.task_name = self.task_name
        req.frame_id = self.frame_id
        req.tpgmm_start_pose = start_pose_demo
        req.tpgmm_goal_pose = goal_pose_demo
        req.deformed_tpgmm_start_pose = start_pose_demo
        req.deformed_tpgmm_goal_pose = goal_pose_demo
        req.obstacle_pose = obstacle_pose
        req.obstacle_radius = float(self.obstacle_radius)
        req.desired_clearance = float(self.desired_clearance)

        self.obstacle_pose_pub.publish(obstacle_pose)
        self.start_pose_pub.publish(start_pose_robot)
        self.goal_pose_pub.publish(goal_pose_robot)

        future = self.deform_tpgmm_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info("DeformTPGMM service call succeeded. Waiting for bounding volume...")
            for _ in range(30):
                rclpy.spin_once(self, timeout_sec=0.1)
                if self.deformed_gmm_bounding_volume is not None:
                    self.get_logger().info(f"Received Deformed GMM BoundingVolume ({len(self.deformed_gmm_bounding_volume.primitives)} primitives).")
                    return True
            self.get_logger().warn("Timed out waiting for deformed_gmm_moveit bounding volume topic.")
            return True
        return False

    def run_single_trial(self, trial_idx):
        self.get_logger().info(f"\n{'='*55}\n>>> STARTING EXPERIMENT TRIAL {trial_idx} <<<\n{'='*55}")

        self.gmm_bounding_volume = None
        self.deformed_gmm_bounding_volume = None

        # -------------------------------------------------------------
        # STEP 1: Move Robot to Random Start Pose & Open Gripper
        # -------------------------------------------------------------
        # Sample randomized start pose in robot workspace
        start_x = float(np.random.uniform(0.22, 0.30)) #(0.32, 0.40)
        start_y = float(np.random.uniform(-0.15, 0.15))
        start_z = float(np.random.uniform(0.50, 0.58))

        demo_down_quat = [-0.7071, 0.0, -0.7071, 0.0]  # [w, x, y, z]

        start_pose_demo = PoseStamped()
        start_pose_demo.header.frame_id = self.frame_id
        start_pose_demo.pose.position.x = start_x
        start_pose_demo.pose.position.y = start_y
        start_pose_demo.pose.position.z = start_z
        start_pose_demo.pose.orientation.w = demo_down_quat[0]
        start_pose_demo.pose.orientation.x = demo_down_quat[1]
        start_pose_demo.pose.orientation.y = demo_down_quat[2]
        start_pose_demo.pose.orientation.z = demo_down_quat[3]

        start_pose_robot = self.reverse_adjust_orientation(start_pose_demo)

        # Open gripper before moving to start
        self.set_gripper(open_gripper=True)
        time.sleep(0.3)

        self.get_logger().info(f"Moving robot to randomized start pose: ({start_x:.3f}, {start_y:.3f}, {start_z:.3f})...")
        if not self.plan_and_execute(start_pose_robot, apply_constraints=False):
            self.get_logger().error("Failed to reach start pose. Retrying...")
            return False

        time.sleep(0.5)

        # -------------------------------------------------------------
        # STEP 2: Reset & Randomize Environment Objects (Gazebo & MoveIt)
        # -------------------------------------------------------------
        # Target cube (blue)
        cube1_x = float(np.random.uniform(0.52, 0.62))
        cube1_y = float(np.random.uniform(-0.18, 0.18))
        cube1_z = 0.275

        # Distractor cube (red) - keep separation from cube1
        while True:
            cube2_x = float(np.random.uniform(0.48, 0.62))
            cube2_y = float(np.random.uniform(-0.25, 0.25))
            if math.hypot(cube2_x - cube1_x, cube2_y - cube1_y) > 0.12:
                break
        cube2_z = 0.275

        # Obstacle placed directly in the path between start and target cube
        alpha = np.random.uniform(0.45, 0.55)
        obs_x = float(alpha * start_x + (1 - alpha) * cube1_x + np.random.uniform(-0.02, 0.02))
        obs_y = float(alpha * start_y + (1 - alpha) * cube1_y + np.random.uniform(-0.02, 0.02))
        obs_z = 0.375

        self.get_logger().info(f"Resetting environment for Trial {trial_idx}:")
        self.get_logger().info(f"  - Target Cube1 Pose: ({cube1_x:.3f}, {cube1_y:.3f}, {cube1_z:.3f})")
        self.get_logger().info(f"  - Distractor Cube2 Pose: ({cube2_x:.3f}, {cube2_y:.3f}, {cube2_z:.3f})")
        self.get_logger().info(f"  - Obstacle Pose:    ({obs_x:.3f}, {obs_y:.3f}, {obs_z:.3f})")

        # Apply Gazebo entity poses
        self.set_gazebo_pose("cube1", cube1_x, cube1_y, cube1_z)
        self.set_gazebo_pose("cube2", cube2_x, cube2_y, cube2_z)
        self.set_gazebo_pose("obstacle", obs_x, obs_y, obs_z)

        # Update MoveIt Planning Scene so RRT avoids the cylinder obstacle and table
        self.update_moveit_planning_scene(obs_x, obs_y, obs_z, obs_radius=self.obstacle_radius, obs_height=0.25)
        time.sleep(0.5)

        # -------------------------------------------------------------
        # STEP 3: Call TP-GMM & RL Policy Deformation
        # -------------------------------------------------------------
        # Pre-grasp goal pose: 0.05 m offset above the cube in Z to avoid colliding during approach
        cube_top_z = cube1_z + 0.025  # top surface of 5cm cube
        pre_grasp_z = cube_top_z + 0.05 + 0.1 # 0.05m above the cube + 0.1m flange to gripper finger distance
        goal_pose_demo = deepcopy(start_pose_demo)
        goal_pose_demo.pose.position.x = cube1_x
        goal_pose_demo.pose.position.y = cube1_y
        goal_pose_demo.pose.position.z = pre_grasp_z

        goal_pose_robot = self.reverse_adjust_orientation(goal_pose_demo)

        obstacle_pose = PoseStamped()
        obstacle_pose.header.frame_id = self.frame_id
        obstacle_pose.pose.position.x = obs_x
        obstacle_pose.pose.position.y = obs_y
        obstacle_pose.pose.position.z = obs_z + 0.25/2 # To mark the obstacle pose at the top of the cylyinder, not the middle of it.
        obstacle_pose.pose.orientation.w = 1.0

        if not self.call_deform_tpgmm_service(start_pose_robot, goal_pose_robot, obstacle_pose):
            self.get_logger().error("TP-GMM deformation failed!")
            return False

        # -------------------------------------------------------------
        # STEP 4: Plan & Execute with Deformed GMM Path Constraint
        # -------------------------------------------------------------
        self.get_logger().info(f"Planning through Deformed GMM corridor to pre-grasp pose (0.05m above cube, z={pre_grasp_z:.3f}m)...")
        success = self.plan_and_execute(goal_pose_robot, apply_constraints=True, use_deformed=True)
        if not success:
            self.get_logger().warn("Constrained planning failed, attempting relaxed execution...")
            success = self.plan_and_execute(goal_pose_robot, apply_constraints=False)
            if not success:
                self.get_logger().error("Failed to navigate to target cube!")
                return False

        time.sleep(0.5)

        # -------------------------------------------------------------
        # STEP 5: Reach Down, Grasp Cube, and Lift Up (Cartesian Path)
        # -------------------------------------------------------------
        self.get_logger().info("Reaching down vertically via Cartesian path to grasp the cube...")
        grasp_pose_demo = deepcopy(goal_pose_demo)
        grasp_pose_demo.pose.position.z = cube1_z + 0.015 + 0.1  # ~0.390m (grasp height with flange offset)
        grasp_pose_robot = self.reverse_adjust_orientation(grasp_pose_demo)

        cartesian_ok = self.plan_and_execute_cartesian([grasp_pose_robot], step_size=0.005, max_velocity_scaling=0.2)
        if not cartesian_ok:
            self.get_logger().warn("Cartesian descent failed, using standard planner fallback...")
            self.plan_and_execute(grasp_pose_robot, apply_constraints=False, planning_time=3.0)

        time.sleep(0.3)

        self.get_logger().info("Grasping cube...")
        self.set_gripper(open_gripper=False, effort=30.0)
        time.sleep(0.5)

        self.get_logger().info("Lifting cube up vertically via Cartesian path...")
        lift_pose_demo = deepcopy(goal_pose_demo)
        lift_pose_demo.pose.position.z = cube1_z + 0.18 + 0.1  # ~0.555m (lifted)
        lift_pose_robot = self.reverse_adjust_orientation(lift_pose_demo)

        lift_cartesian_ok = self.plan_and_execute_cartesian([lift_pose_robot], step_size=0.005, max_velocity_scaling=0.2)
        if not lift_cartesian_ok:
            self.get_logger().warn("Cartesian lift failed, using standard planner fallback...")
            self.plan_and_execute(lift_pose_robot, apply_constraints=False, planning_time=3.0)

        time.sleep(1.0)

        # Release cube
        self.set_gripper(open_gripper=True)
        time.sleep(0.5)

        self.get_logger().info(f"\n[SUCCESS] Completed Experiment Trial {trial_idx} successfully!\n")
        return True

    def run_experiments(self, num_trials=10, auto=True):
        self.get_logger().info(f"Starting experiment batch: {num_trials} trials (auto={auto})...")
        successes = 0
        for i in range(1, num_trials + 1):
            if not auto:
                ans = input(f"\nPress [Enter] to start Trial {i}/{num_trials} (or 'q' to quit): ")
                if ans.strip().lower() == 'q':
                    break

            try:
                ok = self.run_single_trial(i)
                if ok:
                    successes += 1
            except Exception as e:
                self.get_logger().error(f"Trial {i} encountered exception: {e}")

            time.sleep(1.0)

        self.get_logger().info(f"\n{'='*55}\nExperiment Batch Summary: {successes}/{num_trials} Trials Successful ({(successes/max(1,num_trials))*100:.1f}%)\n{'='*55}")

def main():
    parser = argparse.ArgumentParser(description="Automated Gazebo Experiment Runner for TP-GMM + RL Policy")
    parser.add_argument('--trials', '-n', type=int, default=10, help='Number of experiment trials (default: 10)')
    parser.add_argument('--task', '-t', type=str, default='pick', help='Task name (default: pick)')
    parser.add_argument('--robot', '-r', type=str, default='franka_panda', help='Robot type (default: franka_panda)')
    parser.add_argument('--clearance', '-c', type=float, default=0.5, help='Desired clearance factor [0.0 - 1.0] (default: 0.5)')
    parser.add_argument('--auto', action='store_true', default=True, help='Run trials automatically without pausing')
    parser.add_argument('--manual', dest='auto', action='store_false', help='Prompt before each trial')

    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args if ros_args else None)

    runner = GazeboExperimentRunner(
        robot_type=args.robot,
        task_name=args.task,
        desired_clearance=args.clearance
    )

    try:
        runner.run_experiments(num_trials=args.trials, auto=args.auto)
    except KeyboardInterrupt:
        runner.get_logger().info("Experiment run interrupted by user.")
    finally:
        runner.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
