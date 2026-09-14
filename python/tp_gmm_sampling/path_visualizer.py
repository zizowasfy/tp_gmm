"""Optional end-effector path display, independent of sampling and diagnostics."""
from copy import deepcopy
import math
import time

import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy
from moveit_msgs.srv import GetPositionFK
from visualization_msgs.msg import Marker, MarkerArray

from .client import MODES


class PathVisualizer:
    """Retain the latest successful planned path per mode while this node lives.

    Call publish() only after a successful MoveGroup result, outside callbacks
    spinning this node. FK work happens after planning; failure only logs a warning.
    This visualizes the planned link origin, not measured execution or tool geometry.
    """

    def __init__(self, node, namespace=''):
        self.node = node
        self.fk = node.create_client(GetPositionFK, f'{namespace}/compute_fk')
        self.publisher = node.create_publisher(
            MarkerArray, f'{namespace}/gmm_sampling/paths',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.paths = {}

    def publish(self, trajectory, start_state, link, frame, mode, timeout=10.0):
        """Publish a complete path or leave the previous display intact on failure."""
        try:
            deadline = time.monotonic() + timeout
            if mode not in MODES:
                raise ValueError(f'Unknown sampling mode: {mode}')
            if trajectory.multi_dof_joint_trajectory.points:
                raise ValueError('Path visualization currently supports fixed-base joint trajectories')
            joints = trajectory.joint_trajectory
            if not joints.joint_names or len(joints.points) < 2:
                raise ValueError('No planned path to visualize')
            configurations = [list(p.positions) for p in joints.points]
            if any(len(q) != len(joints.joint_names) or not all(map(math.isfinite, q))
                   for q in configurations):
                raise ValueError('Invalid trajectory positions')
            steps = [max(1, math.ceil(math.dist(a, b) / .02))
                     for a, b in zip(configurations[:-1], configurations[1:])]
            if 1 + sum(steps) > 2000:
                raise ValueError('Path exceeds visualization budget of 2000 FK points')
            if not self.fk.wait_for_service(timeout_sec=min(1.0, timeout)):
                raise RuntimeError('MoveIt compute_fk service unavailable')
            request = GetPositionFK.Request()
            request.header.frame_id = frame
            request.fk_link_names = [link]
            request.robot_state = deepcopy(start_state)
            positions = dict(zip(start_state.joint_state.name, start_state.joint_state.position))
            positions.update(zip(joints.joint_names, configurations[0]))
            request.robot_state.joint_state.name = list(positions)
            request.robot_state.joint_state.velocity = []
            request.robot_state.joint_state.effort = []
            request.robot_state.is_diff = False
            marker = Marker()
            marker.header.frame_id = frame
            marker.ns, marker.id = mode, 0
            marker.type, marker.action = Marker.LINE_STRIP, Marker.ADD
            marker.pose.orientation.w = 1.0
            marker.scale.x = .005
            marker.color.r, marker.color.g, marker.color.b = (
                (.1, 1., .1) if mode == 'cartesian_ik' else (.1, .3, 1.))
            marker.color.a = 1.0

            def append_fk(q):
                positions.update(zip(joints.joint_names, q))
                request.robot_state.joint_state.position = list(positions.values())
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise TimeoutError('Path visualization exceeded its time budget')
                future = self.fk.call_async(request)
                rclpy.spin_until_future_complete(self.node, future, timeout_sec=remaining)
                if not future.done():
                    future.cancel()
                    raise TimeoutError('Path FK timed out')
                response = future.result()
                if (response is None or response.error_code.val != response.error_code.SUCCESS
                        or len(response.pose_stamped) != 1):
                    raise RuntimeError('MoveIt could not compute path FK')
                pose = response.pose_stamped[0]
                point = pose.pose.position
                if pose.header.frame_id != frame or not all(map(math.isfinite, (point.x, point.y, point.z))):
                    raise RuntimeError('Invalid FK frame or coordinates')
                marker.points.append(deepcopy(point))

            append_fk(configurations[0])
            for a, b, count in zip(configurations[:-1], configurations[1:], steps):
                for i in range(1, count + 1):
                    append_fk([x + (y - x) * i / count for x, y in zip(a, b)])
            self.paths[mode] = marker
            self.publisher.publish(MarkerArray(markers=list(self.paths.values())))
            return True
        except Exception as error:
            self.node.get_logger().warning(f'Successful path visualization skipped: {error}')
            return False
