"""Plan-only sequential reference IK baseline, using public MoveIt services.

This is independent of the GMM sampler. It joins the actual start and fixed joint
 goal, audits interpolated joint states, and never sends an execution action.
"""
from copy import deepcopy
import math
import time
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import GetPositionIK, GetPositionFK, GetStateValidity


def resample_reference(reference, step=.01):
    points = np.array([[p.position.x,p.position.y,p.position.z] for p in reference.poses])
    if len(points) < 2 or not np.isfinite(points).all() or step <= 0:
        raise ValueError('A finite reference polyline and positive step are required')
    lengths = np.linalg.norm(np.diff(points, axis=0), axis=1)
    keep = np.r_[True, lengths > 1e-9]
    points = points[keep]
    arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(points, axis=0), axis=1))]
    if arc[-1] <= 0 or arc[-1]/step > 2000:
        raise ValueError('Degenerate or oversized reference path')
    samples = np.linspace(0., arc[-1], max(2,math.ceil(arc[-1]/step)+1))
    return np.stack([np.interp(samples,arc,points[:,i]) for i in range(3)],axis=1)


class ReferenceIKBaseline:
    def __init__(self, node, namespace=''):
        self.node = node
        self.ik = node.create_client(GetPositionIK, f'{namespace}/compute_ik')
        self.fk = node.create_client(GetPositionFK, f'{namespace}/compute_fk')
        self.validity = node.create_client(GetStateValidity, f'{namespace}/check_state_validity')

    def call(self, client, request, deadline):
        remaining = deadline-time.monotonic()
        if remaining <= 0:
            raise TimeoutError('Reference baseline budget exhausted')
        if not client.wait_for_service(timeout_sec=min(1., remaining)):
            raise RuntimeError(f'Unavailable baseline service {client.srv_name}')
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=max(0.,deadline-time.monotonic()))
        if not future.done():
            future.cancel()
            raise TimeoutError('Reference baseline service timed out')
        return future.result()

    def plan(self, reference, start, goal, joint_names, group, link, orientation, constraints, budget=3.):
        begin = time.monotonic(); deadline = begin + budget
        states = [deepcopy(start)]
        checked = 0
        try:
            for xyz in resample_reference(reference):
                pose = PoseStamped(header=deepcopy(reference.header))
                pose.pose.orientation = deepcopy(orientation)
                pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = map(float,xyz)
                req = GetPositionIK.Request()
                req.ik_request.group_name, req.ik_request.ik_link_name = group, link
                req.ik_request.robot_state = states[-1]
                req.ik_request.pose_stamped = pose
                req.ik_request.avoid_collisions = True
                req.ik_request.timeout.nanosec = 20_000_000
                response = self.call(self.ik,req,deadline)
                if response.error_code.val != response.error_code.SUCCESS:
                    return dict(success=False, failure_stage='reference_ik', action_wall_s=time.monotonic()-begin)
                states.append(response.solution)
            states.append(deepcopy(goal))
            def q(state):
                positions = dict(zip(state.joint_state.name,state.joint_state.position))
                return np.array([positions[n] for n in joint_names])
            configurations = [q(state) for state in states]
            ee = []; length = 0.
            for a,b in zip(configurations[:-1],configurations[1:]):
                length += float(np.linalg.norm(b-a))
                for fraction in np.linspace(0.,1.,max(2,math.ceil(np.linalg.norm(b-a)/.02)+1)):
                    state = deepcopy(start)
                    positions = dict(zip(state.joint_state.name,state.joint_state.position))
                    positions.update(zip(joint_names, map(float,a+(b-a)*fraction)))
                    state.joint_state.name, state.joint_state.position = list(positions), list(positions.values())
                    response = self.call(self.validity, GetStateValidity.Request(robot_state=state, group_name=group, constraints=constraints), deadline)
                    checked += 1
                    if not response.valid:
                        return dict(success=False, failure_stage='reference_edge_invalid', action_wall_s=time.monotonic()-begin,
                                    sampling=dict(path_validation_samples=checked, path_invalid_samples=1))
                    request = GetPositionFK.Request(robot_state=state, fk_link_names=[link])
                    request.header.frame_id = reference.header.frame_id
                    response = self.call(self.fk,request,deadline)
                    if response.error_code.val != response.error_code.SUCCESS or not response.pose_stamped:
                        raise RuntimeError('Reference baseline FK failed')
                    p=response.pose_stamped[0].pose.position
                    ee.append([p.x,p.y,p.z])
            return dict(success=True, planner_success=True, path_audit_passed=True, failure_stage=None,
                        action_wall_s=time.monotonic()-begin, ee_points=ee,
                        sampling=dict(joint_path_length=length, ee_path_length_m=float(np.linalg.norm(np.diff(ee,axis=0),axis=1).sum()),
                                      path_validation_samples=checked, path_invalid_samples=0))
        except TimeoutError:
            return dict(success=False, failure_stage='reference_timeout', action_wall_s=time.monotonic()-begin)
