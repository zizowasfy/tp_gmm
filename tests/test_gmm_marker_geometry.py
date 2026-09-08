#!/usr/bin/env python3
"""Compare the real converter's ellipsoids with PrepareSampling's corridor geometry.

Run against sampling_demo.launch.py lfd:=false rviz:=false on an isolated ROS domain.
The converter is started on private test topics; no planning or robot execution occurs.
"""
import math
from pathlib import Path
import subprocess
import sys
import tempfile
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from rcl_interfaces.srv import SetParameters
from geometry_msgs.msg import Quaternion
from tp_gmm.msg import Gaussian, GaussianMixture
from visualization_msgs.msg import MarkerArray

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from sampling_client import SamplingClient


def wait(node, future, timeout=10):
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
    assert future.done() and future.result() is not None, 'Service call timed out'
    return future.result()


def main():
    rclpy.init()
    node = Node('gmm_geometry_test')
    sampling = SamplingClient(node)
    received = []
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(MarkerArray, '/gmm_geometry_test/markers', received.append, qos)
    publisher = node.create_publisher(GaussianMixture, '/gmm_geometry_test/model', 1)
    parameters = node.create_client(SetParameters, '/gmm_geometry_converter/set_parameters')
    log = tempfile.TemporaryFile(mode='w+')
    process = subprocess.Popen([
        'ros2', 'run', 'tp_gmm', 'gmm_rviz_converter_node', '--ros-args',
        '-r', '__node:=gmm_geometry_converter',
        '-p', 'input_topic:=/gmm_geometry_test/model',
        '-p', 'output_topic:=/gmm_geometry_test/markers',
    ], stdout=log, stderr=subprocess.STDOUT)
    ids = []
    try:
        assert parameters.wait_for_service(timeout_sec=15), 'Converter did not start'
        deadline = time.monotonic() + 5
        while publisher.get_subscription_count() == 0 and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        assert publisher.get_subscription_count() > 0
        angle = 0.6
        rotation = np.array([[math.cos(angle), -math.sin(angle), 0],
                             [math.sin(angle), math.cos(angle), 0], [0, 0, 1.]])
        covariance = np.eye(4)
        covariance[1:, 1:] = rotation @ np.diag([0.0001, 0.0004, 1e-12]) @ rotation.T
        model = GaussianMixture()
        model.header.frame_id = 'panda_link0'
        model.gaussians = [Gaussian(means=[0., 0.4+i*0.1, 0., 0.6],
                                    covariances=covariance.ravel().tolist()) for i in range(2)]
        model.weights = [0.1, 0.9]

        def publish_model():
            received.clear()
            publisher.publish(model)
            deadline = time.monotonic() + 5
            while not received and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.1)
            assert received, 'No converter output'
            return [m for m in received[-1].markers if m.action == m.ADD]

        def set_parameters(**values):
            request = SetParameters.Request(parameters=[Parameter(name=name,
                value=(ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=value) if isinstance(value, bool)
                       else ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=value)))
                for name, value in values.items()])
            response = wait(node, parameters.call_async(request))
            assert all(result.successful for result in response.results)

        for mode, cutoff, floor in [('legacy_weighted', 3., 1e-8), ('covariance', 2., 1e-7)]:
            model.weights = [0.1, 0.9]
            set_parameters(cutoff=cutoff, covariance_floor=floor, legacy_weighted_scale=mode == 'legacy_weighted')
            prepared = sampling.prepare(model, 'panda_arm', 'panda_hand', Quaternion(x=1., w=0.),
                                        'cartesian_ik', cutoff=cutoff, covariance_floor=floor, corridor_mode=mode)
            ids.append(prepared.request_id)
            markers = publish_model()
            region = prepared.constraints.position_constraints[0].constraint_region
            assert len(markers) == 2
            for marker, box, pose in zip(markers, region.primitives, region.primitive_poses):
                assert marker.header.frame_id == model.header.frame_id
                np.testing.assert_allclose([marker.scale.x, marker.scale.y, marker.scale.z], box.dimensions,
                                           rtol=1e-12, atol=1e-12)
                assert marker.pose == pose, 'Ellipsoid axes or centre differ from sampler'
            widths = lambda m: np.array([m.scale.x, m.scale.y, m.scale.z])
            np.testing.assert_allclose(widths(markers[1]) / widths(markers[0]),
                                       9. if mode == 'legacy_weighted' else 1.)
            model.weights = [0.8, 0.2]
            changed = publish_model()
            for i, (before, after) in enumerate(zip(markers, changed)):
                ratio = model.weights[i] / [0.1, 0.9][i] if mode == 'legacy_weighted' else 1.
                np.testing.assert_allclose(widths(after), widths(before) * ratio)
                assert after.pose == before.pose

        # A late RViz subscriber receives the retained geometry without a new model.
        retained = []
        node.create_subscription(MarkerArray, '/gmm_geometry_test/markers', retained.append, qos)
        deadline = time.monotonic() + 5
        while not retained and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        assert retained
        print('PASS: rotated 4-D spatial covariance, unequal/changing priors, covariance floors, '
              'historical/covariance corridors and late-join visualization all match sampler geometry')
    finally:
        for identity in ids:
            sampling.report(identity)
        process.terminate()
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill(); process.wait(timeout=5)
        if process.returncode not in (0, -15):
            log.seek(0); print(log.read())
        log.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
