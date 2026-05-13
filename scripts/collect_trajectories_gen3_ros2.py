#!/usr/bin/env python3

"""
Trajectory Data Collection Script for Kinova Gen 3 (ROS 2 Migration)

This script records trajectories (demonstrations) for tp_gmm training
using Kinova Gen 3 in a ROS 2 simulation environment. 

Enhancements from ROS 1 version:
- Migrated from `moveit_commander` (ROS 1) to `MoveGroup` Action Client (ROS 2), bypassing the need for complex MoveItPy launch files.
- Migrated from `rosbag` to `rosbags` package for robust ROS 2 bag writing.
- Uses native `/compute_fk` service.
"""

import sys
import time
import math
import random
import numpy as np
from pathlib import Path
from copy import deepcopy

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from scipy.spatial.transform import Rotation as R

from geometry_msgs.msg import PoseStamped, Pose, PoseArray, Quaternion
from moveit_msgs.msg import BoundingVolume, Constraints, PositionConstraint, DisplayTrajectory, OrientationConstraint, MotionPlanRequest
from moveit_msgs.action import MoveGroup
from moveit_msgs.srv import GetPositionFK
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Header
from tp_gmm.srv import ReproduceTPGMM

try:
    from rosbags.rosbag2 import Writer
    from rclpy.serialization import serialize_message
except ImportError:
    print("WARNING: rosbags is not installed. Please install it via: pip install rosbags")


class TrajectoryDataCollector(Node):
    def __init__(self, namespace="", frame_id="base_link", group_name="manipulator"):
        super().__init__('collect_trajectories_gen3')
        
        self.namespace = namespace
        self.frame_id = frame_id
        self.group_name = group_name
        self.ee_link = f"{self.namespace[1:]}_end_effector_link" if namespace.startswith("/") else "end_effector_link"
        
        self.save_dir = Path("/home/zizo/the_folder/Trajectory_Data_Collection/")
        self.save_dir.mkdir(parents=True, exist_ok=True)
        
        self.get_logger().info("Initializing MoveGroup Action Client...")
        
        self.move_action_client = ActionClient(self, MoveGroup, f'{self.namespace}/move_action')
        if not self.move_action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("MoveGroup action server not available!")
            sys.exit(1)
            
        self.fk_client = self.create_client(GetPositionFK, f'{self.namespace}/compute_fk')
        
        from tp_gmm.srv import ReproduceTPGMM
        self.reproduce_tpgmm_client = self.create_client(ReproduceTPGMM, f"{self.namespace}/ReproduceTPGMM_service")
        
        self.start_pose_pub = self.create_publisher(PoseStamped, f"{self.namespace}/start_pose", 1)
        self.goal_pose_pub = self.create_publisher(PoseStamped, f"{self.namespace}/goal_pose", 1)
        self.start_pose_gmm_pub = self.create_publisher(PoseStamped, f"{self.namespace}/start_pose_gmm", 1)
        self.goal_pose_gmm_pub = self.create_publisher(PoseStamped, f"{self.namespace}/goal_pose_gmm", 1)
        self.posearray_pub = self.create_publisher(PoseArray, f"{self.namespace}/planned_trajectory/posearray", 1)
        
        self.create_subscription(BoundingVolume, f"{self.namespace}/gmm_moveit", self.gmm_constraint_cb, 10)
        self.create_subscription(DisplayTrajectory, f"{self.namespace}/move_group/display_planned_path", self.display_path_cb, 10)
        
        self.start_pose = None
        self.goal_pose = None
        self.plan_result = None
        self.gmm_bounding_volume = None
        
        self.demon_num = 1
        self.get_logger().info("Trajectory Data Collector Node Initialized Successfully!")

    def euler_to_quat(self, yaw, pitch, roll):
        rot = R.from_euler('ZYX', [yaw, pitch, roll], degrees=False) # ZYX around current frame
        # rot = R.from_euler('xyz', [roll, pitch, yaw], degrees=False) # xyz around reference frame
        q = rot.as_quat()
        quat = Quaternion()
        quat.x, quat.y, quat.z, quat.w = q[0], q[1], q[2], q[3]
        return quat

    def get_task_random_poses(self, task_name):
        start = Pose()
        goal = Pose()
        
        if task_name == "pick":
            start.position.x = random.uniform(0.35, 0.55)
            start.position.y = random.uniform(-0.47, 0.47)
            start.position.z = random.uniform(0.4, 0.5)
            # start.orientation = self.euler_to_quat(0.0, math.radians(90), math.radians(random.randint(-70, 70)))
            start.orientation = self.euler_to_quat(math.radians(random.randint(-130, -70)), math.radians(180), math.radians(0.0)) # ZYX around current frame
            # start.orientation = self.euler_to_quat(math.radians(-90.0), math.radians(180), math.radians(0.0)) # zyx (reads from right to left x-y-z) around reference frame

            goal.position.x = random.uniform(0.35, 0.65)
            goal.position.y = random.uniform(-0.47, 0.47)
            goal.position.z = random.uniform(0.05, 0.1)
            # goal.orientation = self.euler_to_quat(0.0, math.radians(90), math.radians(random.randint(-70, 70)))
            goal.orientation = self.euler_to_quat(math.radians(random.randint(-130, -70)), math.radians(180), math.radians(0.0)) # ZYX around current frame
            
        elif task_name == "place":
            objects = [[0.6, 0.2, 0.5], [0.5, 0.3, 0.5], [0.7, -0.1, 0.6]]
            idx = (self.demon_num - 1) % len(objects)
            start.position.x, start.position.y, start.position.z = objects[idx]
            start.orientation = self.euler_to_quat(0.0, math.radians(90), 0.0)
            
            goal.position.x, goal.position.y, goal.position.z = 0.55, -0.16, 0.83
            goal.orientation = self.euler_to_quat(0.0, math.radians(90), math.radians(-45))
            
        return start, goal

    def setup_task(self, task_name):
        task_start, task_goal = self.get_task_random_poses(task_name)
        
        self.start_pose = PoseStamped()
        self.start_pose.header.frame_id = self.frame_id
        self.start_pose.pose = task_start
        self.start_pose_pub.publish(self.start_pose)
        
        self.goal_pose = PoseStamped()
        self.goal_pose.header.frame_id = self.frame_id
        self.goal_pose.pose = task_goal
        self.goal_pose_pub.publish(self.goal_pose)
        
        self.get_logger().info(f"Task poses set for '{task_name}'.")
        return self.start_pose, self.goal_pose

    def construct_goal_constraints(self, target_pose):
        goal_constraint = Constraints()
        
        pos_constraint = PositionConstraint()
        pos_constraint.header = target_pose.header
        pos_constraint.link_name = self.ee_link
        
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.SPHERE
        primitive.dimensions = [0.01]
        
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
        ori_constraint.absolute_x_axis_tolerance = 0.05
        ori_constraint.absolute_y_axis_tolerance = 0.05
        ori_constraint.absolute_z_axis_tolerance = 0.05
        ori_constraint.weight = 1.0
        
        goal_constraint.position_constraints.append(pos_constraint)
        goal_constraint.orientation_constraints.append(ori_constraint)
        return goal_constraint

    def plan_and_execute(self, target_pose, apply_constraints=False):
        self.get_logger().info("Sending goal to MoveGroup Action Server...")
        
        req = MoveGroup.Goal()
        req.request.group_name = self.group_name
        req.request.num_planning_attempts = 3
        req.request.allowed_planning_time = 5.0
        req.request.max_velocity_scaling_factor = 1.0
        req.request.max_acceleration_scaling_factor = 1.0
        
        goal_constraint = self.construct_goal_constraints(target_pose)
        req.request.goal_constraints.append(goal_constraint)
        
        if apply_constraints and self.gmm_bounding_volume:
            self.get_logger().info("Applying GMM Path Constraints...")
            path_constraint = Constraints()
            path_constraint.name = "position_constraint"
            p_const = PositionConstraint()
            p_const.header.frame_id = self.frame_id
            p_const.link_name = self.ee_link
            p_const.constraint_region = self.gmm_bounding_volume
            p_const.weight = 1.0
            path_constraint.position_constraints.append(p_const)
            req.request.path_constraints = path_constraint

        # Execute as well
        req.planning_options.plan_only = False
        
        future = self.move_action_client.send_goal_async(req)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error("MoveGroup Goal rejected")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        res = result_future.result().result

        if res.error_code.val == res.error_code.SUCCESS:
            self.get_logger().info("Execution succeeded.")
            self.plan_result = res.planned_trajectory
            return True
        else:
            self.get_logger().error(f"MoveGroup failed with error code: {res.error_code.val}")
            return False

    def record_rosbag(self):
        bag_path = self.save_dir / f"demon_{self.demon_num}"
        if bag_path.exists():
            import shutil
            shutil.rmtree(bag_path)
            
        from rosbags.typesys import get_typestore, Stores
        typestore = get_typestore(Stores.ROS2_JAZZY)
        
        self.get_logger().info(f"Writing bag to {bag_path}...")
        with Writer(bag_path, version=8) as writer:
            start_conn = writer.add_connection(f"{self.namespace}/start_pose", "geometry_msgs/msg/PoseStamped", typestore=typestore)
            goal_conn = writer.add_connection(f"{self.namespace}/goal_pose", "geometry_msgs/msg/PoseStamped", typestore=typestore)
            traj_conn = writer.add_connection(f"{self.namespace}/move_group/result", "moveit_msgs/msg/RobotTrajectory", typestore=typestore)
            
            ts = int(time.time() * 1e9)
            writer.write(start_conn, ts, serialize_message(self.start_pose))
            writer.write(goal_conn, ts, serialize_message(self.goal_pose))
            
            if self.plan_result:
                writer.write(traj_conn, ts, serialize_message(self.plan_result))
                
        self.get_logger().info(f"Demonstration {self.demon_num} saved.")

    def gmm_constraint_cb(self, msg):
        self.gmm_bounding_volume = msg

    def display_path_cb(self, msg):
        pass # Forward kinematics is handled differently if needed, skipping for brevity unless required.

    # This function is to adjust the orienation of the Kinova to that of UR10 to match the demonstrations recorded with the UR10.
    # No need for this function if demonstrations are recorded with the Kinova.
    # -90 around Z, -90 around Y, 0 around X - around current frame
    def adjust_orientation(self, pose):

        kinova_orienation = R.from_quat([pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w])
        adjustment_rotation = R.from_euler('ZYX', [-90, -90, 0], degrees=True)
        adjusted_rotation = kinova_orienation * adjustment_rotation
        adjusted_pose = deepcopy(pose)
        adjusted_pose.pose.orientation.x = adjusted_rotation.as_quat()[0]
        adjusted_pose.pose.orientation.y = adjusted_rotation.as_quat()[1]
        adjusted_pose.pose.orientation.z = adjusted_rotation.as_quat()[2]
        adjusted_pose.pose.orientation.w = adjusted_rotation.as_quat()[3]
        
        return adjusted_pose
        

    def call_tpgmm_service(self, task, start_pose, target_pose):
        self.get_logger().info("Calling ReproduceTPGMM service...")
        if not self.reproduce_tpgmm_client.wait_for_service(timeout_sec=2.0):
            return False

        req = ReproduceTPGMM.Request()
        req.task_name = task
        req.frame_id = self.frame_id
        req.start_pose = self.adjust_orientation(start_pose) # start_pose
        req.goal_pose = self.adjust_orientation(target_pose) # target_pose

        ## for debugging
        self.start_pose_gmm_pub.publish(req.start_pose)
        self.goal_pose_gmm_pub.publish(req.goal_pose)
        ##

        future = self.reproduce_tpgmm_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info("Successfully reproduced TPGMM.")
            for _ in range(20):
                rclpy.spin_once(self, timeout_sec=0.1)
                if self.gmm_bounding_volume is not None:
                    break
            return True
        return False

    def test_tpgmm(self, task_name, num_experiments=10):
        exp = 1
        self.get_logger().info("Testing TPGMM Reproduction...")
        while exp <= num_experiments:
            self.get_logger().info(f"--- TPGMM Test Experiment No. {exp} ---")
            self.gmm_bounding_volume = None
            time.sleep(1.0)

            start_pose, goal_pose = self.setup_task(task_name)

            ans = input("Proceed with these Task Parameters? [y/n]: ")
            if ans.lower() == 'y':
                if self.plan_and_execute(start_pose):
                    time.sleep(1.0)
                    ##
                    if self.call_tpgmm_service(task_name, start_pose, goal_pose):
                        ans2 = input("Execute the constrained plan? [y/n]: ")
                        if ans2.lower() == 'y':
                            self.plan_and_execute(goal_pose, apply_constraints=True)

                    # self.plan_and_execute(goal_pose, apply_constraints=True)
                    ##
                exp += 1
            
            time.sleep(0.5)

def main(args=None):
    rclpy.init(args=args)
    namespace = ""
    frame_id = "base_link"
    group_name = "manipulator" 
    task_name = "pick"
    
    node = TrajectoryDataCollector(namespace, frame_id, group_name)
    try:
        node.test_tpgmm(task_name, num_experiments=10)
        node.get_logger().info("Execution complete.")
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
