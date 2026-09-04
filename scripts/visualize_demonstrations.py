#!/usr/bin/env python3

"""
Demonstrations Visualizer for RViz
=================================
Publishes demonstration trajectories before and after Dynamic Time Warping (DTW)
alignment, with dedicated highlighting for the reference demonstration (ref_demon).

Published Topics:
  PoseArrays:
    - /demons/raw/poses         (geometry_msgs/msg/PoseArray) - All raw demonstration poses
    - /demons/aligned/poses     (geometry_msgs/msg/PoseArray) - All DTW-aligned demonstration poses
    - /demons/reference/poses   (geometry_msgs/msg/PoseArray) - Reference demonstration poses only

  Visual Markers (Continuous 3D Line Strips, Waypoints & Text Labels):
    - /demons/raw/markers       (visualization_msgs/msg/MarkerArray) - Raw trajectories (Cyan)
    - /demons/aligned/markers   (visualization_msgs/msg/MarkerArray) - DTW-aligned trajectories (Emerald Green)
    - /demons/reference/markers (visualization_msgs/msg/MarkerArray) - Bold reference demonstration (Gold)
    - /demons/inspect/markers   (visualization_msgs/msg/MarkerArray) - Currently inspected demonstration (when cycling)
"""

import os
import sys
import re
import time
import argparse
from pathlib import Path
from copy import deepcopy
import numpy as np

# Resolve directory paths
SCRIPTS_DIR = Path(__file__).resolve().parent
INCLUDE_DIR = SCRIPTS_DIR.parent / "include"
if str(SCRIPTS_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_DIR))
if str(INCLUDE_DIR) not in sys.path:
    sys.path.insert(0, str(INCLUDE_DIR))

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import Pose, PoseArray, Point, Quaternion, Vector3
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from dtw import dtw, warp
from dir_paths import get_demonstrations_dir
from demons_to_samples import discover_demonstrations, RosbagWrapper


def to_ros_pose(raw_p) -> Pose:
    """Converts a deserialized rosbag pose into a native ROS 2 geometry_msgs/Pose."""
    p = Pose()
    p.position = Point(
        x=float(raw_p.position.x),
        y=float(raw_p.position.y),
        z=float(raw_p.position.z)
    )
    p.orientation = Quaternion(
        x=float(raw_p.orientation.x),
        y=float(raw_p.orientation.y),
        z=float(raw_p.orientation.z),
        w=float(raw_p.orientation.w)
    )
    return p


class DemonstrationVisualizer(Node):
    def __init__(self, task_name: str = "franka_pick_cube", frame_id: str | None = None,
                 publish_rate: float = 1.0, cycle_interval: float = 0.0,
                 inspect_demon: str | None = None):
        super().__init__("demonstrations_visualizer")

        self.task_name = task_name
        self.publish_rate = max(publish_rate, 0.1)
        self.cycle_interval = cycle_interval
        self.target_inspect_demon = inspect_demon

        # Load and align demonstrations
        self.get_logger().info(f"Loading demonstrations for task '{self.task_name}'...")
        self.demon_data = self._load_and_align_demonstrations()

        if frame_id:
            self.frame_id = frame_id
        elif self.demon_data["detected_frame_id"]:
            self.frame_id = self.demon_data["detected_frame_id"]
        else:
            self.frame_id = "panda_link0"

        self.get_logger().info(f"Using visualization frame_id: '{self.frame_id}'")

        # QoS with TRANSIENT_LOCAL so RViz receives messages immediately upon opening
        latch_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE
        )

        # PoseArray publishers
        self.pub_raw_poses = self.create_publisher(PoseArray, "/demons/raw/poses", latch_qos)
        self.pub_aligned_poses = self.create_publisher(PoseArray, "/demons/aligned/poses", latch_qos)
        self.pub_ref_poses = self.create_publisher(PoseArray, "/demons/reference/poses", latch_qos)

        # MarkerArray publishers (3D LineStrips, Spheres, Labels)
        self.pub_raw_markers = self.create_publisher(MarkerArray, "/demons/raw/markers", latch_qos)
        self.pub_aligned_markers = self.create_publisher(MarkerArray, "/demons/aligned/markers", latch_qos)
        self.pub_ref_markers = self.create_publisher(MarkerArray, "/demons/reference/markers", latch_qos)
        self.pub_inspect_markers = self.create_publisher(MarkerArray, "/demons/inspect/markers", latch_qos)

        # Pre-build static messages for efficiency
        self.raw_pose_array_msg = self._build_pose_array(is_aligned=False)
        self.aligned_pose_array_msg = self._build_pose_array(is_aligned=True)
        self.ref_pose_array_msg = self._build_ref_pose_array()

        self.raw_markers_msg = self._build_trajectory_markers(is_aligned=False)
        self.aligned_markers_msg = self._build_trajectory_markers(is_aligned=True)
        self.ref_markers_msg = self._build_reference_markers()

        # Cycling index for single-demonstration inspection
        self.demon_keys = list(self.demon_data["demons"].keys())
        if self.target_inspect_demon and self.target_inspect_demon in self.demon_keys:
            self.current_inspect_idx = self.demon_keys.index(self.target_inspect_demon)
        else:
            self.current_inspect_idx = 0
        self.last_cycle_time = time.time()

        # Publish initial batch
        self.publish_all()

        # Main timer loop
        self.timer = self.create_timer(1.0 / self.publish_rate, self.timer_callback)

        self.get_logger().info("\n" + "=" * 68)
        self.get_logger().info(f" Demonstrations Visualizer Ready for '{self.task_name}'")
        self.get_logger().info(f" Frame ID:               '{self.frame_id}'")
        self.get_logger().info(f" Total Demonstrations:   {len(self.demon_keys)}")
        self.get_logger().info(f" Reference Demonstration: '{self.demon_data['ref_num']}' ({self.demon_data['ref_nbpoints']} points)")
        self.get_logger().info(f" Raw Trajectory Length:  {self.demon_data['min_len']} -> {self.demon_data['max_len']} points")
        self.get_logger().info(" Aligned Trajectory:     All warped to uniform 24 points")
        self.get_logger().info("-" * 68)
        self.get_logger().info(" Topics Published:")
        self.get_logger().info("   PoseArrays:")
        self.get_logger().info("     • /demons/raw/poses")
        self.get_logger().info("     • /demons/aligned/poses")
        self.get_logger().info("     • /demons/reference/poses")
        self.get_logger().info("   Visual Markers (Continuous 3D Paths):")
        self.get_logger().info("     • /demons/raw/markers        [Cyan line strips]")
        self.get_logger().info("     • /demons/aligned/markers    [Emerald Green line strips]")
        self.get_logger().info("     • /demons/reference/markers  [Thick Gold path + Waypoints]")
        if self.cycle_interval > 0.0:
            self.get_logger().info(f"   Inspection Cycling: /demons/inspect/markers (every {self.cycle_interval:.1f}s)")
        self.get_logger().info("=" * 68 + "\n")

    def _load_and_align_demonstrations(self) -> dict:
        """Reads rosbags, extracts poses, and computes DTW alignments against reference."""
        demons_dir = get_demonstrations_dir(self.task_name)
        paths = discover_demonstrations(demons_dir)

        if not paths:
            raise FileNotFoundError(f"No demonstration bags found in {demons_dir}")

        raw_demons = {}
        detected_frame_id = None
        ref_num = None
        max_points = -1

        for p in paths:
            d_name = p.name
            m = re.search(r'demon_\d+', d_name)
            d_id = m.group(0) if m else p.stem

            bag = RosbagWrapper(p)
            _, _, posearray_topic = bag.find_topics()
            if not posearray_topic:
                posearray_topic = "/planned_trajectory/posearray"

            poses = []
            for _, msg, _ in bag.read_messages(topics=[posearray_topic]):
                if hasattr(msg, "header") and msg.header.frame_id and not detected_frame_id:
                    detected_frame_id = msg.header.frame_id
                for raw_p in msg.poses:
                    poses.append(to_ros_pose(raw_p))
            bag.close()

            if not poses:
                continue

            pts = np.array([[pose.position.x, pose.position.y, pose.position.z] for pose in poses])
            raw_demons[d_id] = {
                "poses": poses,
                "pts": pts,
                "nb_points": len(poses),
            }

            if len(poses) > max_points:
                max_points = len(poses)
                ref_num = d_id

        if not raw_demons or not ref_num:
            raise RuntimeError(f"Failed to extract valid demonstrations from {demons_dir}")

        # DTW Alignment against reference demonstration
        ref_pts = raw_demons[ref_num]["pts"]
        aligned_demons = {}
        lens = [v["nb_points"] for v in raw_demons.values()]

        for d_id, data in raw_demons.items():
            if d_id == ref_num:
                aligned_demons[d_id] = {
                    "poses": deepcopy(data["poses"]),
                    "pts": deepcopy(data["pts"]),
                    "is_ref": True
                }
            else:
                alignment = dtw(data["pts"], ref_pts)
                wq = warp(alignment, index_reference=False)
                warped_poses = [deepcopy(data["poses"][idx]) for idx in wq]
                warped_pts = np.array([[p.position.x, p.position.y, p.position.z] for p in warped_poses])
                aligned_demons[d_id] = {
                    "poses": warped_poses,
                    "pts": warped_pts,
                    "is_ref": False
                }

        return {
            "demons": raw_demons,
            "aligned": aligned_demons,
            "ref_num": ref_num,
            "ref_nbpoints": max_points,
            "min_len": min(lens),
            "max_len": max(lens),
            "detected_frame_id": detected_frame_id
        }

    def _build_pose_array(self, is_aligned: bool = False) -> PoseArray:
        """Constructs a PoseArray containing poses of all demonstrations."""
        pa = PoseArray()
        pa.header.frame_id = self.frame_id
        pa.header.stamp = self.get_clock().now().to_msg()

        dataset = self.demon_data["aligned"] if is_aligned else self.demon_data["demons"]
        for d_id, data in dataset.items():
            for p in data["poses"]:
                pa.poses.append(deepcopy(p))
        return pa

    def _build_ref_pose_array(self) -> PoseArray:
        """Constructs a PoseArray containing only the reference demonstration poses."""
        pa = PoseArray()
        pa.header.frame_id = self.frame_id
        pa.header.stamp = self.get_clock().now().to_msg()

        ref_num = self.demon_data["ref_num"]
        ref_poses = self.demon_data["demons"][ref_num]["poses"]
        for p in ref_poses:
            pa.poses.append(deepcopy(p))
        return pa

    def _build_trajectory_markers(self, is_aligned: bool = False) -> MarkerArray:
        """Builds continuous 3D line strips for each demonstration."""
        ma = MarkerArray()
        now = self.get_clock().now().to_msg()

        dataset = self.demon_data["aligned"] if is_aligned else self.demon_data["demons"]
        ref_num = self.demon_data["ref_num"]
        ns_prefix = "aligned_demons" if is_aligned else "raw_demons"

        # Color palette:
        # Raw: Cyan / Sky-blue (RGBA: 0.0, 0.75, 0.95, 0.65)
        # Aligned: Emerald Green (RGBA: 0.1, 0.85, 0.4, 0.65)
        if is_aligned:
            base_color = ColorRGBA(r=0.1, g=0.85, b=0.4, a=0.65)
        else:
            base_color = ColorRGBA(r=0.0, g=0.75, b=0.95, a=0.65)

        marker_id = 0
        for d_id, data in dataset.items():
            is_ref = (d_id == ref_num)
            poses = data["poses"]
            if len(poses) < 2:
                continue

            # Line strip for trajectory path
            line_marker = Marker()
            line_marker.header.frame_id = self.frame_id
            line_marker.header.stamp = now
            line_marker.ns = f"{ns_prefix}_lines"
            line_marker.id = marker_id
            marker_id += 1
            line_marker.type = Marker.LINE_STRIP
            line_marker.action = Marker.ADD
            line_marker.scale.x = 0.006 if is_ref else 0.0025  # Line width in meters
            line_marker.pose.orientation.w = 1.0

            if is_ref:
                # Reference highlighted in radiant golden-yellow
                line_marker.color = ColorRGBA(r=1.0, g=0.84, b=0.0, a=0.95)
            else:
                line_marker.color = base_color

            for pose in poses:
                line_marker.points.append(Point(
                    x=float(pose.position.x),
                    y=float(pose.position.y),
                    z=float(pose.position.z)
                ))
            ma.markers.append(line_marker)

            # Start point sphere
            start_sphere = Marker()
            start_sphere.header.frame_id = self.frame_id
            start_sphere.header.stamp = now
            start_sphere.ns = f"{ns_prefix}_endpoints"
            start_sphere.id = marker_id
            marker_id += 1
            start_sphere.type = Marker.SPHERE
            start_sphere.action = Marker.ADD
            start_sphere.pose.position = Point(
                x=float(poses[0].position.x),
                y=float(poses[0].position.y),
                z=float(poses[0].position.z)
            )
            start_sphere.pose.orientation.w = 1.0
            start_sphere.scale = Vector3(x=0.008, y=0.008, z=0.008)
            start_sphere.color = ColorRGBA(r=0.2, g=0.9, b=0.2, a=0.7)  # Light Green
            ma.markers.append(start_sphere)

            # End point sphere
            end_sphere = Marker()
            end_sphere.header.frame_id = self.frame_id
            end_sphere.header.stamp = now
            end_sphere.ns = f"{ns_prefix}_endpoints"
            end_sphere.id = marker_id
            marker_id += 1
            end_sphere.type = Marker.SPHERE
            end_sphere.action = Marker.ADD
            end_sphere.pose.position = Point(
                x=float(poses[-1].position.x),
                y=float(poses[-1].position.y),
                z=float(poses[-1].position.z)
            )
            end_sphere.pose.orientation.w = 1.0
            end_sphere.scale = Vector3(x=0.008, y=0.008, z=0.008)
            end_sphere.color = ColorRGBA(r=0.9, g=0.2, b=0.2, a=0.7)  # Light Red
            ma.markers.append(end_sphere)

        return ma

    def _build_reference_markers(self) -> MarkerArray:
        """Constructs rich, bold visualization specifically for the reference demonstration."""
        ma = MarkerArray()
        now = self.get_clock().now().to_msg()
        ref_num = self.demon_data["ref_num"]
        ref_poses = self.demon_data["demons"][ref_num]["poses"]

        # 1. Bold golden Line Strip
        line = Marker()
        line.header.frame_id = self.frame_id
        line.header.stamp = now
        line.ns = "reference_demon"
        line.id = 0
        line.type = Marker.LINE_STRIP
        line.action = Marker.ADD
        line.scale.x = 0.009  # 9mm thick bold ribbon
        line.pose.orientation.w = 1.0
        line.color = ColorRGBA(r=1.0, g=0.82, b=0.0, a=1.0)  # Pure Gold
        for p in ref_poses:
            line.points.append(Point(
                x=float(p.position.x),
                y=float(p.position.y),
                z=float(p.position.z)
            ))
        ma.markers.append(line)

        # 2. Waypoint Spheres along reference trajectory
        spheres = Marker()
        spheres.header.frame_id = self.frame_id
        spheres.header.stamp = now
        spheres.ns = "reference_demon"
        spheres.id = 1
        spheres.type = Marker.SPHERE_LIST
        spheres.action = Marker.ADD
        spheres.scale = Vector3(x=0.012, y=0.012, z=0.012)
        spheres.pose.orientation.w = 1.0
        spheres.color = ColorRGBA(r=1.0, g=0.6, b=0.0, a=0.9)  # Amber
        for p in ref_poses:
            spheres.points.append(Point(
                x=float(p.position.x),
                y=float(p.position.y),
                z=float(p.position.z)
            ))
        ma.markers.append(spheres)

        # 3. Start Marker (Green Cube)
        start_m = Marker()
        start_m.header.frame_id = self.frame_id
        start_m.header.stamp = now
        start_m.ns = "reference_demon"
        start_m.id = 2
        start_m.type = Marker.CUBE
        start_m.action = Marker.ADD
        start_m.pose.position = Point(
            x=float(ref_poses[0].position.x),
            y=float(ref_poses[0].position.y),
            z=float(ref_poses[0].position.z)
        )
        start_m.pose.orientation.w = 1.0
        start_m.scale = Vector3(x=0.02, y=0.02, z=0.02)
        start_m.color = ColorRGBA(r=0.0, g=1.0, b=0.3, a=0.95)  # Bright Green
        ma.markers.append(start_m)

        # 4. Goal Marker (Red Cube)
        goal_m = Marker()
        goal_m.header.frame_id = self.frame_id
        goal_m.header.stamp = now
        goal_m.ns = "reference_demon"
        goal_m.id = 3
        goal_m.type = Marker.CUBE
        goal_m.action = Marker.ADD
        goal_m.pose.position = Point(
            x=float(ref_poses[-1].position.x),
            y=float(ref_poses[-1].position.y),
            z=float(ref_poses[-1].position.z)
        )
        goal_m.pose.orientation.w = 1.0
        goal_m.scale = Vector3(x=0.02, y=0.02, z=0.02)
        goal_m.color = ColorRGBA(r=1.0, g=0.1, b=0.1, a=0.95)  # Crimson Red
        ma.markers.append(goal_m)

        # 5. Floating Text Annotation
        mid_idx = len(ref_poses) // 2
        mid_pos = ref_poses[mid_idx].position
        text_m = Marker()
        text_m.header.frame_id = self.frame_id
        text_m.header.stamp = now
        text_m.ns = "reference_demon"
        text_m.id = 4
        text_m.type = Marker.TEXT_VIEW_FACING
        text_m.action = Marker.ADD
        text_m.pose.position = Point(x=float(mid_pos.x), y=float(mid_pos.y), z=float(mid_pos.z) + 0.05)
        text_m.pose.orientation.w = 1.0
        text_m.scale.z = 0.035  # Text height
        text_m.color = ColorRGBA(r=1.0, g=0.95, b=0.2, a=1.0)
        text_m.text = f"REF: {ref_num} ({len(ref_poses)} pts)"
        ma.markers.append(text_m)

        return ma

    def _build_inspect_markers(self, demon_id: str) -> MarkerArray:
        """Builds side-by-side comparison for a single inspected demonstration."""
        ma = MarkerArray()
        now = self.get_clock().now().to_msg()

        raw_poses = self.demon_data["demons"][demon_id]["poses"]
        aligned_poses = self.demon_data["aligned"][demon_id]["poses"]
        ref_num = self.demon_data["ref_num"]
        is_ref = (demon_id == ref_num)

        # 1. Raw trajectory (Cyan)
        raw_line = Marker()
        raw_line.header.frame_id = self.frame_id
        raw_line.header.stamp = now
        raw_line.ns = "inspect_single"
        raw_line.id = 10
        raw_line.type = Marker.LINE_STRIP
        raw_line.action = Marker.ADD
        raw_line.scale.x = 0.005
        raw_line.pose.orientation.w = 1.0
        raw_line.color = ColorRGBA(r=0.0, g=0.8, b=1.0, a=0.9)  # Cyan
        for p in raw_poses:
            raw_line.points.append(Point(
                x=float(p.position.x),
                y=float(p.position.y),
                z=float(p.position.z)
            ))
        ma.markers.append(raw_line)

        # 2. Aligned trajectory (Emerald Green)
        aligned_line = Marker()
        aligned_line.header.frame_id = self.frame_id
        aligned_line.header.stamp = now
        aligned_line.ns = "inspect_single"
        aligned_line.id = 11
        aligned_line.type = Marker.LINE_STRIP
        aligned_line.action = Marker.ADD
        aligned_line.scale.x = 0.005
        aligned_line.pose.orientation.w = 1.0
        aligned_line.color = ColorRGBA(r=0.1, g=1.0, b=0.4, a=0.9)  # Green
        for p in aligned_poses:
            aligned_line.points.append(Point(
                x=float(p.position.x),
                y=float(p.position.y),
                z=float(p.position.z)
            ))
        ma.markers.append(aligned_line)

        # 3. Label Marker
        mid_idx = len(raw_poses) // 2
        mid_pos = raw_poses[mid_idx].position
        txt = Marker()
        txt.header.frame_id = self.frame_id
        txt.header.stamp = now
        txt.ns = "inspect_single"
        txt.id = 12
        txt.type = Marker.TEXT_VIEW_FACING
        txt.action = Marker.ADD
        txt.pose.position = Point(x=float(mid_pos.x), y=float(mid_pos.y), z=float(mid_pos.z) + 0.08)
        txt.pose.orientation.w = 1.0
        txt.scale.z = 0.03
        txt.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
        tag = " [REFERENCE]" if is_ref else ""
        txt.text = f"{demon_id}{tag} (Raw: {len(raw_poses)} -> Aligned: {len(aligned_poses)} pts)"
        ma.markers.append(txt)

        return ma

    def publish_all(self):
        """Publishes all PoseArrays and MarkerArrays with updated timestamps."""
        now = self.get_clock().now().to_msg()

        # Update timestamps
        self.raw_pose_array_msg.header.stamp = now
        self.aligned_pose_array_msg.header.stamp = now
        self.ref_pose_array_msg.header.stamp = now

        for m in self.raw_markers_msg.markers:
            m.header.stamp = now
        for m in self.aligned_markers_msg.markers:
            m.header.stamp = now
        for m in self.ref_markers_msg.markers:
            m.header.stamp = now

        # Publish PoseArrays
        self.pub_raw_poses.publish(self.raw_pose_array_msg)
        self.pub_aligned_poses.publish(self.aligned_pose_array_msg)
        self.pub_ref_poses.publish(self.ref_pose_array_msg)

        # Publish MarkerArrays
        self.pub_raw_markers.publish(self.raw_markers_msg)
        self.pub_aligned_markers.publish(self.aligned_markers_msg)
        self.pub_ref_markers.publish(self.ref_markers_msg)

        # If inspecting single demon
        curr_id = self.demon_keys[self.current_inspect_idx]
        inspect_msg = self._build_inspect_markers(curr_id)
        self.pub_inspect_markers.publish(inspect_msg)

    def timer_callback(self):
        # Handle cycling inspection if enabled
        if self.cycle_interval > 0.0:
            if time.time() - self.last_cycle_time >= self.cycle_interval:
                self.current_inspect_idx = (self.current_inspect_idx + 1) % len(self.demon_keys)
                self.last_cycle_time = time.time()
                curr_id = self.demon_keys[self.current_inspect_idx]
                self.get_logger().info(f"Inspecting [{self.current_inspect_idx + 1}/{len(self.demon_keys)}]: {curr_id}")

        self.publish_all()


def main():
    parser = argparse.ArgumentParser(description="Demonstrations RViz Visualizer (Before & After DTW)")
    parser.add_argument("--task", "-t", type=str, default="franka_pick_cube",
                        help="Task name (e.g., franka_pick_cube, pick, place_ur10_1). Default: franka_pick_cube")
    parser.add_argument("--frame-id", "-f", type=str, default=None,
                        help="Visualization TF frame_id (default: auto-detected from bag, e.g. panda_link0)")
    parser.add_argument("--rate", "-r", type=float, default=1.0,
                        help="Publishing rate in Hz (default: 1.0)")
    parser.add_argument("--cycle", "-c", type=float, default=0.0,
                        help="Interval in seconds to cycle through individual demonstrations in inspection topic (default: 0 = disabled)")
    parser.add_argument("--demon", "-d", type=str, default=None,
                        help="Specific demonstration to inspect (e.g. demon_5)")

    args = parser.parse_args()

    rclpy.init()
    node = DemonstrationVisualizer(
        task_name=args.task,
        frame_id=args.frame_id,
        publish_rate=args.rate,
        cycle_interval=args.cycle,
        inspect_demon=args.demon
    )

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
