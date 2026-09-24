#!/usr/bin/env python3
"""Controller-free Panda integration check; four functional plans, no comparison analysis.

Start: ros2 launch tp_gmm gmm_sampling.launch.py lfd:=false rviz:=false
Run this script in a second terminal on the same isolated ROS domain.
"""
from copy import deepcopy
from pathlib import Path
import sys
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import PoseStamped, Quaternion, PoseArray, Pose
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import RobotState, Constraints, JointConstraint
from moveit_msgs.srv import GetPositionIK, GetStateValidity
from tp_gmm.msg import Gaussian, GaussianMixture
from visualization_msgs.msg import MarkerArray
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from sampling_client import SamplingClient
from tp_gmm_sampling import PathVisualizer


def wait(node, future, timeout=30):
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
    if not future.done() or future.result() is None:
        raise RuntimeError('ROS test call timed out')
    return future.result()


def main():
    rclpy.init()
    node = Node('gmm_runtime_test')
    sampling = SamplingClient(node)
    paths = PathVisualizer(node)
    ik = node.create_client(GetPositionIK, '/compute_ik')
    validity = node.create_client(GetStateValidity, '/check_state_validity')
    action = ActionClient(node, MoveGroup, '/move_action')
    markers = {}
    def collect(message):
        for marker in message.markers:
            markers[(marker.ns,marker.id)] = marker
    node.create_subscription(MarkerArray, '/gmm_sampling/markers', collect,
                             QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    try:
        assert ik.wait_for_service(timeout_sec=15) and validity.wait_for_service(timeout_sec=15)
        assert action.wait_for_server(timeout_sec=15)
        def solve(y):
            target = PoseStamped()
            target.header.frame_id = 'panda_link0'
            target.pose.position.x, target.pose.position.y, target.pose.position.z = .45, y, .55
            target.pose.orientation.x, target.pose.orientation.w = 1., 0.
            state = RobotState()
            state.joint_state.name = [f'panda_joint{i}' for i in range(1,8)] + ['panda_finger_joint1','panda_finger_joint2']
            state.joint_state.position = [0.,-.785,0.,-2.356,0.,1.571,.785,.04,.04]
            request = GetPositionIK.Request()
            request.ik_request.group_name = 'panda_arm'
            request.ik_request.ik_link_name = 'panda_hand'
            request.ik_request.robot_state = state
            request.ik_request.pose_stamped = target
            request.ik_request.avoid_collisions = True
            request.ik_request.timeout.sec = 2
            response = wait(node,ik.call_async(request))
            assert response.error_code.val == response.error_code.SUCCESS
            return response.solution
        start, goal = solve(-.03), solve(.03)
        model = GaussianMixture()
        model.header.frame_id = 'panda_link0'
        model.weights = [1.]
        model.gaussians = [Gaussian(means=[0.,.45,0.,.55],covariances=np.diag([1.,.0016,.0025,.0016]).ravel().tolist())]
        reference = PoseArray()
        reference.header.frame_id = model.header.frame_id
        for y in [-.03, .03]:
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = .45, y, .55
            reference.poses.append(pose)
        for mode, proposal in [('cartesian_ik','gmm'), ('joint_projected','gmm'),
                               ('cartesian_ik','gmr_path'), ('cartesian_ik','hybrid')]:
            key = mode if proposal == 'gmm' else mode + '/' + proposal
            prepared = sampling.prepare(model,'panda_arm','panda_hand',Quaternion(x=1.,w=0.), mode,
                                        seed=42, visualize=True, uniform_fraction=0., cartesian_fraction=0.,
                                        proposal=proposal, reference_path=reference)
            try:
                request = MoveGroup.Goal()
                request.request.group_name = 'panda_arm'
                request.request.pipeline_id = 'ompl'
                request.request.planner_id = 'RRTConnectkConfigDefault'
                request.request.allowed_planning_time = 5.
                request.request.num_planning_attempts = 1
                request.request.start_state = deepcopy(start)
                request.request.path_constraints = prepared.constraints
                constraints = Constraints()
                for name,value in zip(goal.joint_state.name,goal.joint_state.position):
                    if name.startswith('panda_joint'):
                        constraints.joint_constraints.append(JointConstraint(joint_name=name,position=value,
                            tolerance_above=.001,tolerance_below=.001,weight=1.))
                request.request.goal_constraints.append(constraints)
                request.planning_options.plan_only = True
                handle = wait(node,action.send_goal_async(request))
                assert handle.accepted
                result_future = handle.get_result_async()
                try:
                    result = wait(node,result_future).result
                except RuntimeError:
                    wait(node,handle.cancel_goal_async())
                    wait(node,result_future)
                    raise
                assert result.error_code.val == result.error_code.SUCCESS, result.error_code.val
                assert paths.publish(result.planned_trajectory, result.trajectory_start,
                                     'panda_hand', 'panda_link0', mode, proposal=proposal)
                path = paths.paths[key]
                assert len(path.points) >= len(result.planned_trajectory.joint_trajectory.points)
                assert np.allclose([path.points[0].x,path.points[0].y,path.points[0].z], [.45,-.03,.55], atol=.002)
                assert np.allclose([path.points[-1].x,path.points[-1].y,path.points[-1].z], [.45,.03,.55], atol=.002)
                stats = sampling.report(prepared.request_id,release=False)
                assert stats['sampler_active'] and stats['valid_samples'] > 0
                assert stats['gmr_attempts' if proposal == 'gmr_path' else
                             'cartesian_attempts' if mode == 'cartesian_ik' else 'projected_attempts'] > 0 or stats['gmr_attempts'] > 0
                # Validate joint interpolation against the exact request corridor and scene.
                trajectory = result.planned_trajectory.joint_trajectory
                state = deepcopy(result.trajectory_start)
                indices = [list(state.joint_state.name).index(n) for n in trajectory.joint_names]
                configurations = [np.array(p.positions) for p in trajectory.points]
                for a,b in zip(configurations[:-1],configurations[1:]):
                    for fraction in np.linspace(0,1,max(2,int(np.ceil(np.linalg.norm(b-a)/.02))+1)):
                        positions = list(state.joint_state.position)
                        for i,value in zip(indices,a+(b-a)*fraction): positions[i] = float(value)
                        state.joint_state.position = positions
                        response = wait(node,validity.call_async(GetStateValidity.Request(robot_state=state,
                            group_name='panda_arm',constraints=prepared.constraints)))
                        assert response.valid, 'Returned path violates the request corridor or scene'
                cloud = 'gmr_proposals' if stats['gmr_draws'] > 0 else 'cartesian_proposals'
                deadline = time.monotonic()+3
                while time.monotonic()<deadline and not markers.get((prepared.request_id+'/'+cloud,0)):
                    rclpy.spin_once(node,timeout_sec=.1)
                clouds = [m for (ns,_),m in markers.items() if ns==prepared.request_id+'/'+cloud]
                assert clouds and clouds[0].points
                for point in clouds[0].points:
                    if cloud == 'gmr_proposals':
                        nearest = np.array([.45, np.clip(point.y, -.03, .03), .55])
                        assert np.linalg.norm(np.array([point.x,point.y,point.z])-nearest) <= .020001
                    else:
                        local = (np.array([point.x,point.y,point.z])-[.45,0.,.55])/np.sqrt([.0016,.0025,.0016])
                        assert local@local <= 4+1e-6, 'Raw proposal outside covariance cutoff'
                print(f'PASS: {key} allocated, planned, respected the corridor and published contained proposals')
            finally:
                sampling.release(prepared.request_id)
        # A late RViz subscriber receives both retained successful paths.
        received = []
        node.create_subscription(MarkerArray, '/gmm_sampling/paths', received.append,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        deadline = time.monotonic() + 3
        while not received and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
        assert received and {m.ns for m in received[-1].markers} == {'cartesian_ik', 'joint_projected', 'cartesian_ik/gmr_path', 'cartesian_ik/hybrid'}
        # Invalid visualizations must neither fail the caller nor replace successful paths.
        before = dict(paths.paths)
        from moveit_msgs.msg import RobotTrajectory
        assert not paths.publish(RobotTrajectory(), start, 'panda_hand', 'panda_link0', 'cartesian_ik')
        assert paths.paths == before
        print('PASS: successful FK paths, late-subscriber retention and nonfatal visualization failure')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__=='__main__':
    main()
