#!/usr/bin/env python3
"""Run against sampling_demo.launch.py: malformed input, atomic registration, identity, release and RViz QoS."""
import sys
import time
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import Quaternion, PoseArray, Pose
from copy import deepcopy
from tp_gmm.msg import Gaussian, GaussianMixture
from visualization_msgs.msg import MarkerArray
from sampling_client import SamplingClient

def main():
    rclpy.init()
    node = Node('sampling_protocol_test')
    client = SamplingClient(node)
    model = GaussianMixture()
    model.header.frame_id = 'panda_link0'
    model.gaussians = [Gaussian(means=[0.45, 0., 0.55], covariances=[0.01,0.,0.,0.,0.005,0.,0.,0.,0.002])]
    model.weights = [1.0]
    orientation = Quaternion(x=1.0, w=0.0)
    ids = []
    try:
        first = client.prepare(model, 'panda_arm', 'panda_hand', orientation, 'cartesian_ik', visualize=True)
        ids.append(first.request_id)
        for covariance in ([], [1.0], [float('nan')]*9, [-1.,0.,0.,0.,1.,0.,0.,0.,1.]):
            bad = GaussianMixture()
            bad.header = model.header
            bad.gaussians = [model.gaussians[0], Gaussian(means=[0.5,0.,0.5], covariances=covariance)]
            bad.weights = [0.5,0.5]
            try:
                client.prepare(bad, 'panda_arm', 'panda_hand', orientation, 'joint_projected')
            except RuntimeError:
                pass
            else:
                raise AssertionError('Malformed second component was accepted')
        second = client.prepare(model, 'panda_arm', 'panda_hand', orientation, 'joint_projected', visualize=True)
        ids.append(second.request_id)
        assert first.request_id != second.request_id
        assert first.constraints.position_constraints == second.constraints.position_constraints
        assert client.report(first.request_id, release=False)['mode'] == 'cartesian_ik'
        assert client.report(second.request_id, release=False)['mode'] == 'joint_projected'
        reference = PoseArray()
        reference.header.frame_id = model.header.frame_id
        for y in [-.03,.03]:
            p = Pose(); p.position.x,p.position.y,p.position.z = .45,y,.55
            reference.poses.append(p)
        third = client.prepare(model,'panda_arm','panda_hand',orientation,'cartesian_ik',
                               proposal='gmr_path',reference_path=reference,visualize=True)
        ids.append(third.request_id)
        assert first.constraints.position_constraints == third.constraints.position_constraints
        reference.poses[0].position.x = 100.
        assert abs(client.report(third.request_id,release=False)['reference_length_m']-.06) < 1e-6
        reference.poses[0].position.x = .45
        for alteration in ('frame','nan','duplicate','empty'):
            bad = deepcopy(reference)
            if alteration == 'frame': bad.header.frame_id = 'another_frame'
            if alteration == 'nan': bad.poses[0].position.x = float('nan')
            if alteration == 'duplicate': bad.poses = [bad.poses[0],bad.poses[0]]
            if alteration == 'empty': bad.poses = []
            try:
                client.prepare(model,'panda_arm','panda_hand',orientation,'cartesian_ik',proposal='gmr_path',reference_path=bad)
            except (ValueError,RuntimeError): pass
            else: raise AssertionError('Invalid reference accepted: '+alteration)

        received = []
        node.create_subscription(MarkerArray, '/gmm_sampling/markers', received.append,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        limit = time.monotonic()+5
        while time.monotonic() < limit and not received:
            rclpy.spin_once(node, timeout_sec=.1)
        assert received and any(m.type == m.SPHERE for m in received[-1].markers)
        client.report(ids.pop(0))
        try:
            client.report(first.request_id)
        except RuntimeError:
            pass
        else:
            raise AssertionError('Released request remained accessible')
        print('PASS: malformed arrays/NaN/indefinite covariance rejected transactionally; IDs isolated; released ID rejected; transient RViz support received')
    finally:
        for request_id in ids:
            client.report(request_id)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
