#!/usr/bin/env python3

"""
Modular Demonstration Collection Pipeline for Robot Manipulation (ROS 2)

Supports recording demonstrations (rosbags) for TP-GMM learning across different
robots (Franka Panda, Kinova Gen 3, UR10), tasks (e.g., 'franka_pick_cube', 'pick', 'place'),
and environments (Gazebo Sim / Real Robot).

Features:
- MoveGroup action client for planning and execution.
- Forward Kinematics (/compute_fk) to extract end-effector Cartesian trajectories (PoseArray).
- Interactive user prompt after each trial: accept/save ([y]), discard/skip ([n]), retry ([r]), or quit ([q]).
- Writes ROS 2 rosbags with start_pose, goal_pose, planned_trajectory/posearray, and move_group/result.
- Automatic indexing and resuming (demon_1, demon_2, ...).
- Gazebo Sim integration: randomizes object poses in Gazebo and updates MoveIt Planning Scene.
"""

import sys
import os
import time
import math
import random
import shutil
import re
import argparse
import subprocess
import numpy as np
from pathlib import Path
from copy import deepcopy

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from scipy.spatial.transform import Rotation as R

from geometry_msgs.msg import PoseStamped, Pose, PoseArray, Quaternion, Point
from moveit_msgs.msg import (
    BoundingVolume,
    Constraints,
    PositionConstraint,
    OrientationConstraint,
    RobotState,
    CollisionObject,
    PlanningScene,
    RobotTrajectory
)
from moveit_msgs.action import MoveGroup
from moveit_msgs.srv import GetPositionFK, ApplyPlanningScene
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Header
from control_msgs.action import ParallelGripperCommand

try:
    from rosbags.rosbag2 import Writer
    from rosbags.typesys import get_typestore, Stores
    from rclpy.serialization import serialize_message
    ROSBAGS_AVAILABLE = True
except ImportError:
    ROSBAGS_AVAILABLE = False
    print("WARNING: 'rosbags' package is not installed. Please install via: pip install rosbags")

# Directory helpers
from ament_index_python.packages import get_package_share_directory
try:
    pkg_share = get_package_share_directory('tp_gmm')
    sys.path.append(os.path.join(pkg_share, 'scripts'))
    sys.path.append(os.path.join(pkg_share, 'include'))
except Exception:
    pass

try:
    from dir_paths import get_paths, get_demonstrations_dir
    DEMONS_BASE_DIR = get_demonstrations_dir()
except Exception:
    DEMONS_BASE_DIR = Path('/home/zizo/the_folder/Trajectory_Data_Collection/Demons')


ROBOT_CONFIGS = {
    "franka_panda": {
        "frame_id": "panda_link0",
        "group_name": "panda_arm",
        "ee_link": "panda_hand",
        "gripper_group": "hand",
        "gripper_action": "/panda_hand_controller/gripper_cmd",
        # Panda native downward orientation (gripper pointing down along -Z of panda_link0)
        # Quaternion [x, y, z, w] = [1.0, 0.0, 0.0, 0.0]
        "default_down_quat": (1.0, 0.0, 0.0, 0.0),
    },
    "kinova_gen3": {
        "frame_id": "base_link",
        "group_name": "manipulator",
        "ee_link": "end_effector_link",
        "gripper_group": "gripper",
        "gripper_action": "/robotiq_gripper_controller/gripper_cmd",
        "default_down_quat": (0.0, 1.0, 0.0, 0.0),
    },
    "ur10": {
        "frame_id": "base_link",
        "group_name": "manipulator",
        "ee_link": "tool0",
        "gripper_group": "gripper",
        "gripper_action": "/gripper_controller/gripper_cmd",
        "default_down_quat": (0.0, -0.7071, 0.0, -0.7071),
    },
}


class DemonstrationCollector(Node):
    def __init__(
        self,
        robot_type: str = "franka_panda",
        task_name: str = "franka_pick_cube",
        namespace: str = "",
        save_dir: Path | None = None,
        use_gazebo: bool = True,
        auto_save: bool = False,
    ):
        super().__init__('demonstration_collector')

        self.robot_type = robot_type
        self.task_name = task_name
        self.namespace = namespace.rstrip('/')
        self.use_gazebo = use_gazebo
        self.auto_save = auto_save

        cfg = ROBOT_CONFIGS.get(robot_type, ROBOT_CONFIGS["franka_panda"])
        self.frame_id = cfg["frame_id"]
        self.group_name = cfg["group_name"]
        self.ee_link = cfg["ee_link"]
        self.default_down_quat = cfg["default_down_quat"]
        self.gripper_action_name = cfg.get("gripper_action", "")

        # Save directory for this task
        if save_dir is not None:
            self.task_demons_dir = Path(save_dir) / self.task_name
        else:
            self.task_demons_dir = DEMONS_BASE_DIR / self.task_name

        self.task_demons_dir.mkdir(parents=True, exist_ok=True)
        self.get_logger().info(f"Demonstrations directory: {self.task_demons_dir}")

        # Action clients
        move_action_name = f"{self.namespace}/move_action" if self.namespace else "/move_action"
        self.get_logger().info(f"Connecting to MoveGroup Action Server at '{move_action_name}'...")
        self.move_action_client = ActionClient(self, MoveGroup, move_action_name)
        if not self.move_action_client.wait_for_server(timeout_sec=15.0):
            self.get_logger().error("MoveGroup action server not available! Make sure MoveIt is running.")
            sys.exit(1)

        gripper_topic = f"{self.namespace}{self.gripper_action_name}" if self.namespace else self.gripper_action_name
        self.gripper_action_client = ActionClient(self, ParallelGripperCommand, gripper_topic)

        # Service clients
        fk_srv_name = f"{self.namespace}/compute_fk" if self.namespace else "/compute_fk"
        self.fk_client = self.create_client(GetPositionFK, fk_srv_name)

        scene_srv_name = f"{self.namespace}/apply_planning_scene" if self.namespace else "/apply_planning_scene"
        self.apply_planning_scene_client = self.create_client(ApplyPlanningScene, scene_srv_name)

        # Publishers for visualization
        self.start_pose_pub = self.create_publisher(PoseStamped, f"{self.namespace}/start_pose" if self.namespace else "/start_pose", 1)
        self.goal_pose_pub = self.create_publisher(PoseStamped, f"{self.namespace}/goal_pose" if self.namespace else "/goal_pose", 1)
        self.posearray_pub = self.create_publisher(PoseArray, f"{self.namespace}/planned_trajectory/posearray" if self.namespace else "/planned_trajectory/posearray", 1)
        self.planning_scene_pub = self.create_publisher(PlanningScene, f"{self.namespace}/planning_scene" if self.namespace else "/planning_scene", 10)

        # State storage
        self.start_pose: PoseStamped | None = None
        self.goal_pose: PoseStamped | None = None
        self.last_robot_trajectory: RobotTrajectory | None = None
        self.last_posearray: PoseArray | None = None

        # Determine starting demonstration number by checking existing demons
        self.demon_num = self._get_next_demon_index()
        self.get_logger().info(f"Demonstration Collector initialized for robot '{self.robot_type}', task '{self.task_name}'. Next index: demon_{self.demon_num}")

    def _get_next_demon_index(self) -> int:
        """Finds the highest existing demon_<N> index in task_demons_dir and returns N + 1."""
        if not self.task_demons_dir.exists():
            return 1

        indices = []
        for item in self.task_demons_dir.iterdir():
            match = re.search(r'demon_(\d+)', item.name)
            if match:
                indices.append(int(match.group(1)))

        return max(indices) + 1 if indices else 1

    def euler_to_quat(self, yaw: float, pitch: float, roll: float) -> Quaternion:
        """Convert Euler angles (radians) in ZYX order to geometry_msgs Quaternion."""
        rot = R.from_euler('ZYX', [yaw, pitch, roll], degrees=False)
        q = rot.as_quat()
        quat = Quaternion()
        quat.x, quat.y, quat.z, quat.w = float(q[0]), float(q[1]), float(q[2]), float(q[3])
        return quat

    def set_gripper(self, open_gripper: bool = True, effort: float = 20.0):
        """Controls the robot gripper (opens or closes)."""
        target_pos = 0.04 if open_gripper else 0.0
        self.get_logger().info(f"Gripper -> {'OPEN' if open_gripper else 'CLOSE'} (pos={target_pos})")

        if self.gripper_action_client.wait_for_server(timeout_sec=2.0):
            goal = ParallelGripperCommand.Goal()
            goal.command.name = ["panda_finger_joint1"]
            goal.command.position = [target_pos]
            goal.command.effort = [effort]
            future = self.gripper_action_client.send_goal_async(goal)
            rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
            goal_handle = future.result()
            if goal_handle and goal_handle.accepted:
                res_future = goal_handle.get_result_async()
                rclpy.spin_until_future_complete(self, res_future, timeout_sec=3.0)
                return True

        # Fallback to MoveGroup hand joint
        req = MoveGroup.Goal()
        req.request.group_name = "hand" if self.robot_type == "franka_panda" else "gripper"
        req.request.num_planning_attempts = 2
        req.request.allowed_planning_time = 2.0
        c = Constraints()
        from moveit_msgs.msg import JointConstraint
        jc = JointConstraint()
        jc.joint_name = "panda_finger_joint1" if self.robot_type == "franka_panda" else "finger_joint"
        jc.position = target_pos
        jc.tolerance_above = 0.01
        jc.tolerance_below = 0.01
        jc.weight = 1.0
        c.joint_constraints.append(jc)
        req.request.goal_constraints.append(c)
        future = self.move_action_client.send_goal_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
        return True

    def set_gazebo_pose(self, entity_name: str, x: float, y: float, z: float, qx: float = 0.0, qy: float = 0.0, qz: float = 0.0, qw: float = 1.0):
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
            self.get_logger().warn(f"Could not set Gazebo pose for {entity_name}: {e}")

    def update_moveit_planning_scene(self, table_x: float = 0.6, table_y: float = 0.0, table_z: float = 0.125):
        """Adds table collision object into MoveIt planning scene to ensure collision-free paths."""
        planning_scene = PlanningScene()
        planning_scene.is_diff = True
        planning_scene.robot_state.is_diff = True

        table_co = CollisionObject()
        table_co.header.frame_id = self.frame_id
        table_co.id = "table"
        table_co.operation = CollisionObject.ADD

        table_box = SolidPrimitive()
        table_box.type = SolidPrimitive.BOX
        table_box.dimensions = [0.75, 1.0, 0.25]

        table_pose = Pose()
        table_pose.position.x = table_x
        table_pose.position.y = table_y
        table_pose.position.z = table_z
        table_pose.orientation.w = 1.0

        table_co.primitives.append(table_box)
        table_co.primitive_poses.append(table_pose)
        planning_scene.world.collision_objects.append(table_co)

        self.planning_scene_pub.publish(planning_scene)
        if self.apply_planning_scene_client.wait_for_service(timeout_sec=1.0):
            req = ApplyPlanningScene.Request()
            req.scene = planning_scene
            future = self.apply_planning_scene_client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)

    def sample_task_poses(self) -> tuple[Pose, Pose]:
        """
        Samples randomized (start_pose, goal_pose) tailored for the specified task and robot.
        For 'franka_pick_cube':
        - Start pose: Above the table in the elevated approach workspace.
        - Goal pose: Pre-grasp pose right above the randomized cube on the table.
        """
        start = Pose()
        goal = Pose()

        if self.task_name in ["franka_pick_cube", "pick_cube", "pick"]:
            if self.robot_type == "franka_panda":
                # Start pose in workspace above table
                start.position.x = float(random.uniform(0.22, 0.35))
                start.position.y = float(random.uniform(-0.18, 0.18))
                start.position.z = float(random.uniform(0.58, 0.68))

                # Slight random yaw variation around downward orientation
                start_yaw_var = math.radians(random.uniform(-30.0, 30.0))
                rot = R.from_euler('ZYX', [start_yaw_var, 0.0, math.pi], degrees=False) # roll=pi gives downward pointing EE
                q = rot.as_quat()
                start.orientation.x = float(q[0])
                start.orientation.y = float(q[1])
                start.orientation.z = float(q[2])
                start.orientation.w = float(q[3])

                # Goal pose: pre-grasp pose above the cube on the table
                # Table top is at z = 0.25m, cube center is at z = 0.275m
                cube_x = float(random.uniform(0.30, 0.64))
                cube_y = float(random.uniform(-0.50, 0.50))
                cube_z = 0.275

                # Pre-grasp height: cube top (0.30m) + 0.05m clearance + 0.10m flange-to-fingers = 0.45m
                pre_grasp_z = cube_z + 0.025 + 0.05 + 0.070

                goal.position.x = cube_x
                goal.position.y = cube_y
                goal.position.z = pre_grasp_z

                # Goal orientation matches downward pointing grasp orientation
                goal_yaw_var = math.radians(random.uniform(-45.0, 45.0))
                goal_rot = R.from_euler('ZYX', [goal_yaw_var, 0.0, math.pi], degrees=False)
                gq = goal_rot.as_quat()
                goal.orientation.x = float(gq[0])
                goal.orientation.y = float(gq[1])
                goal.orientation.z = float(gq[2])
                goal.orientation.w = float(gq[3])

                # If Gazebo is running, physically update the cube pose in the simulator
                if self.use_gazebo:
                    self.set_gazebo_pose("cube1", cube_x, cube_y, cube_z, gq[0], gq[1], gq[2], gq[3])

            else:
                # Kinova Gen3 or UR10 default pick workspace
                start.position.x = float(random.uniform(0.25, 0.40))
                start.position.y = float(random.uniform(-0.30, 0.30))
                start.position.z = float(random.uniform(0.40, 0.50))
                start.orientation = self.euler_to_quat(math.radians(random.randint(-130, -70)), math.radians(180), 0.0)

                goal.position.x = float(random.uniform(0.35, 0.65))
                goal.position.y = float(random.uniform(-0.35, 0.35))
                goal.position.z = float(random.uniform(0.10, 0.20))
                goal.orientation = self.euler_to_quat(math.radians(random.randint(-130, -70)), math.radians(180), 0.0)

        elif self.task_name == "place":
            objects = [[0.6, 0.2, 0.5], [0.5, 0.3, 0.5], [0.7, -0.1, 0.6]]
            idx = (self.demon_num - 1) % len(objects)
            start.position.x, start.position.y, start.position.z = objects[idx]
            start.orientation = self.euler_to_quat(0.0, math.radians(90), 0.0)

            goal.position.x, goal.position.y, goal.position.z = 0.55, -0.16, 0.83
            goal.orientation = self.euler_to_quat(0.0, math.radians(90), math.radians(-45))

        else:
            # Generic default random workspace
            start.position.x = float(random.uniform(0.3, 0.5))
            start.position.y = float(random.uniform(-0.2, 0.2))
            start.position.z = float(random.uniform(0.4, 0.6))
            start.orientation.w = 1.0

            goal.position.x = float(random.uniform(0.4, 0.6))
            goal.position.y = float(random.uniform(-0.2, 0.2))
            goal.position.z = float(random.uniform(0.2, 0.4))
            goal.orientation.w = 1.0

        return start, goal

    def construct_goal_constraints(self, target_pose: PoseStamped, pos_tol: float = 0.01, ori_tol: float = 0.01) -> Constraints:
        """Constructs MoveIt position and orientation goal constraints."""
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

    def plan_and_execute(self, target_pose: PoseStamped, planning_time: float = 5.0) -> bool:
        """Plans and executes motion to target_pose using MoveGroup."""
        req = MoveGroup.Goal()
        req.request.group_name = self.group_name
        req.request.num_planning_attempts = 5
        req.request.allowed_planning_time = planning_time
        req.request.max_velocity_scaling_factor = 0.8
        req.request.max_acceleration_scaling_factor = 0.8

        goal_constraint = self.construct_goal_constraints(target_pose)
        req.request.goal_constraints.append(goal_constraint)
        req.planning_options.plan_only = False

        future = self.move_action_client.send_goal_async(req)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()

        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error("MoveGroup Goal rejected!")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        res = result_future.result().result

        if res.error_code.val == res.error_code.SUCCESS:
            self.last_robot_trajectory = res.planned_trajectory
            return True
        else:
            self.get_logger().warn(f"MoveGroup execution failed with code: {res.error_code.val}")
            return False

    def compute_cartesian_trajectory(self, trajectory: RobotTrajectory) -> PoseArray | None:
        """
        Computes forward kinematics for every joint waypoint in trajectory
        to build a geometry_msgs/PoseArray of end-effector poses.
        """
        if not self.fk_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn("/compute_fk service not available, cannot extract PoseArray.")
            return None

        joint_names = trajectory.joint_trajectory.joint_names
        points = trajectory.joint_trajectory.points

        if not points:
            self.get_logger().warn("Empty trajectory points.")
            return None

        pose_array = PoseArray()
        pose_array.header.frame_id = self.frame_id

        for pt in points:
            fk_req = GetPositionFK.Request()
            fk_req.header.frame_id = self.frame_id
            fk_req.fk_link_names = [self.ee_link]
            fk_req.robot_state.joint_state.name = joint_names
            fk_req.robot_state.joint_state.position = list(pt.positions)

            future = self.fk_client.call_async(fk_req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
            res = future.result()

            if res is not None and res.error_code.val == 1 and res.pose_stamped:
                pose_array.poses.append(res.pose_stamped[0].pose)

        self.get_logger().info(f"Extracted {len(pose_array.poses)} Cartesian waypoints for EE '{self.ee_link}'.")
        return pose_array

    def record_rosbag(self, demon_index: int) -> Path | None:
        """Writes the demonstration (start_pose, goal_pose, posearray, trajectory) to a ROS 2 bag."""
        if not ROSBAGS_AVAILABLE:
            self.get_logger().error("rosbags package not available, cannot save bag!")
            return None

        bag_path = self.task_demons_dir / f"demon_{demon_index}"
        if bag_path.exists():
            if bag_path.is_dir():
                shutil.rmtree(bag_path)
            else:
                bag_path.unlink()

        typestore = get_typestore(Stores.ROS2_JAZZY)

        # Register moveit_msgs/msg/RobotTrajectory custom type if not present in default typestore
        if "moveit_msgs/msg/RobotTrajectory" not in typestore.types:
            try:
                from rosbags.typesys.types import generate_typesdict
                robot_traj_def = """
                moveit_msgs/msg/RobotTrajectory
                    trajectory_msgs/msg/JointTrajectory joint_trajectory
                    trajectory_msgs/msg/MultiDOFJointTrajectory multi_dof_joint_trajectory
                """
                typestore.register(generate_typesdict(robot_traj_def))
            except Exception as e:
                self.get_logger().debug(f"Could not register moveit_msgs/msg/RobotTrajectory in typestore: {e}")

        self.get_logger().info(f"Writing demonstration to bag: {bag_path}...")
        ns = self.namespace

        with Writer(bag_path, version=8) as writer:
            start_conn = writer.add_connection(
                f"{ns}/start_pose" if ns else "/start_pose",
                "geometry_msgs/msg/PoseStamped",
                typestore=typestore
            )
            goal_conn = writer.add_connection(
                f"{ns}/goal_pose" if ns else "/goal_pose",
                "geometry_msgs/msg/PoseStamped",
                typestore=typestore
            )
            posearray_conn = writer.add_connection(
                f"{ns}/planned_trajectory/posearray" if ns else "/planned_trajectory/posearray",
                "geometry_msgs/msg/PoseArray",
                typestore=typestore
            )

            # Optional RobotTrajectory connection if registered
            traj_conn = None
            if "moveit_msgs/msg/RobotTrajectory" in typestore.types:
                try:
                    traj_conn = writer.add_connection(
                        f"{ns}/move_group/result" if ns else "/move_group/result",
                        "moveit_msgs/msg/RobotTrajectory",
                        typestore=typestore
                    )
                except Exception:
                    traj_conn = None

            ts = int(time.time() * 1e9)
            if self.start_pose:
                writer.write(start_conn, ts, serialize_message(self.start_pose))
            if self.goal_pose:
                writer.write(goal_conn, ts, serialize_message(self.goal_pose))
            if self.last_posearray:
                writer.write(posearray_conn, ts, serialize_message(self.last_posearray))
            if traj_conn and self.last_robot_trajectory:
                writer.write(traj_conn, ts, serialize_message(self.last_robot_trajectory))

        self.get_logger().info(f"[SAVED] Successfully recorded 'demon_{demon_index}' into {bag_path}")
        return bag_path

    def collect_single_demonstration(self) -> bool:
        """
        Runs a single demonstration trial:
        1. Samples start and goal poses.
        2. Moves robot to start pose (with open gripper).
        3. Plans and executes trajectory to goal pose.
        4. Extracts Cartesian EE path (PoseArray) via /compute_fk.
        5. Visualizes and prompts user whether to save.
        """
        self.get_logger().info(f"\n{'='*60}\n>>> PREPARING DEMONSTRATION TRIAL (Target index: demon_{self.demon_num}) <<<\n{'='*60}")

        # Ensure environment collision objects (e.g. table) are up to date
        if self.use_gazebo:
            self.update_moveit_planning_scene()

        # Step 1: Sample task poses
        start_raw, goal_raw = self.sample_task_poses()

        self.start_pose = PoseStamped()
        self.start_pose.header.frame_id = self.frame_id
        self.start_pose.pose = start_raw
        self.start_pose_pub.publish(self.start_pose)

        self.goal_pose = PoseStamped()
        self.goal_pose.header.frame_id = self.frame_id
        self.goal_pose.pose = goal_raw
        self.goal_pose_pub.publish(self.goal_pose)

        self.get_logger().info(
            f"Sampled Poses:\n"
            f"  - Start: ({start_raw.position.x:.3f}, {start_raw.position.y:.3f}, {start_raw.position.z:.3f})\n"
            f"  - Goal:  ({goal_raw.position.x:.3f}, {goal_raw.position.y:.3f}, {goal_raw.position.z:.3f})"
        )

        # Step 2: Open gripper and move to start pose
        self.set_gripper(open_gripper=True)
        time.sleep(0.3)

        self.get_logger().info("Navigating to start pose...")
        if not self.plan_and_execute(self.start_pose, planning_time=5.0):
            self.get_logger().warn("Failed to reach start pose. Retrying...")
            return False

        time.sleep(0.5)

        # Step 3: Plan and execute trajectory from start to goal pose
        self.get_logger().info("Executing demonstration trajectory to goal pose...")
        if not self.plan_and_execute(self.goal_pose, planning_time=5.0):
            self.get_logger().warn("Motion plan to goal pose failed.")
            return False

        # Step 4: Extract Cartesian PoseArray from trajectory
        if self.last_robot_trajectory:
            self.last_posearray = self.compute_cartesian_trajectory(self.last_robot_trajectory)
            if self.last_posearray:
                self.posearray_pub.publish(self.last_posearray)

        # Step 5: User Confirmation to save or discard
        if self.auto_save:
            self.record_rosbag(self.demon_num)
            self.demon_num += 1
            return True

        while True:
            choice = input(f"\n[?] Demonstration executed ({len(self.last_posearray.poses if self.last_posearray else 0)} points). Save as 'demon_{self.demon_num}'? [y: Save / n: Skip / r: Retry / q: Quit]: ").strip().lower()
            if choice in ['y', 'yes']:
                self.record_rosbag(self.demon_num)
                self.demon_num += 1
                return True
            elif choice in ['n', 'no']:
                self.get_logger().info("[DISCARDED] Demonstration skipped by user.")
                return False
            elif choice in ['r', 'retry']:
                self.get_logger().info("[RETRY] Retrying trial...")
                return False
            elif choice in ['q', 'quit']:
                self.get_logger().info("Collection session terminated by user.")
                raise KeyboardInterrupt
            else:
                print("Invalid input. Please enter 'y', 'n', 'r', or 'q'.")

    def run_collection(self, target_count: int = 10):
        """Runs the collection loop until target_count demonstrations are successfully recorded."""
        self.get_logger().info(f"Starting demonstration collection loop (Target: {target_count} demons for '{self.task_name}')...")
        collected_this_session = 0

        while collected_this_session < target_count:
            try:
                saved = self.collect_single_demonstration()
                if saved:
                    collected_this_session += 1
                    self.get_logger().info(f"Progress: {collected_this_session}/{target_count} collected in this session (Total saved: demon_{self.demon_num - 1}).")
                time.sleep(0.5)
            except KeyboardInterrupt:
                break
            except Exception as e:
                self.get_logger().error(f"Encountered error during trial: {e}")
                time.sleep(1.0)

        self.get_logger().info(f"\n{'='*60}\nDemonstration Collection Complete: {collected_this_session} demons recorded for '{self.task_name}'.\nDirectory: {self.task_demons_dir}\n{'='*60}")


def main(args=None):
    parser = argparse.ArgumentParser(description="Modular Demonstration Collector for Robot Manipulation (ROS 2)")
    parser.add_argument('--task', '-t', type=str, default='franka_pick_cube', help="Task name (default: franka_pick_cube)")
    parser.add_argument('--robot', '-r', type=str, default='franka_panda', help="Robot type (franka_panda, kinova_gen3, ur10)")
    parser.add_argument('--num-demons', '-n', type=int, default=10, help="Number of demonstrations to collect (default: 10)")
    parser.add_argument('--namespace', '-ns', type=str, default='', help="ROS namespace")
    parser.add_argument('--auto', action='store_true', default=False, help="Automatically save valid demonstrations without prompting")
    parser.add_argument('--no-gazebo', dest='use_gazebo', action='store_false', help="Disable Gazebo entity updates")
    parser.add_argument('--save-dir', type=str, default=None, help="Custom root save directory for demonstrations")

    parsed_args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args if ros_args else None)

    collector = DemonstrationCollector(
        robot_type=parsed_args.robot,
        task_name=parsed_args.task,
        namespace=parsed_args.namespace,
        save_dir=Path(parsed_args.save_dir) if parsed_args.save_dir else None,
        use_gazebo=parsed_args.use_gazebo,
        auto_save=parsed_args.auto,
    )

    try:
        collector.run_collection(target_count=parsed_args.num_demons)
    except KeyboardInterrupt:
        collector.get_logger().info("Demonstration collection interrupted by user.")
    finally:
        collector.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
