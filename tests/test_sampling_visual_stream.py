#!/usr/bin/env python3
"""Plan-only integration check of actual planner sample markers and path displays."""
import argparse
import json
import os
import subprocess
import sys
import time
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from visualization_msgs.msg import MarkerArray


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    rclpy.init()
    node = Node('sampling_visual_stream_test')
    collected, paths = {}, {}
    def markers(message):
        for marker in message.markers:
            collected[(marker.ns, marker.id)] = marker
    def path_markers(message):
        for marker in message.markers:
            paths[marker.ns] = marker
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(MarkerArray, '/gmm_sampling/markers', markers, qos)
    node.create_subscription(MarkerArray, '/gmm_sampling/paths', path_markers, qos)
    script = Path(__file__).resolve().parents[1]/'scripts/compare_sampling_approaches.py'
    args.output.mkdir(parents=True, exist_ok=True)
    log = (args.output/'run.log').open('w')
    process = subprocess.Popen([sys.executable, str(script), '--visualize', '--repeats', '1',
                                '--warmup', '0', '--planning-time', '3', '--output', str(args.output)],
                               stdout=log, stderr=subprocess.STDOUT)
    try:
        deadline = time.monotonic()+120
        while process.poll() is None and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
        if process.poll() is None:
            process.terminate(); process.wait(timeout=10)
            raise TimeoutError('Visual planning test timed out')
        assert process.returncode == 0, (args.output/'run.log').read_text()
        for _ in range(5): rclpy.spin_once(node, timeout_sec=.1)
        checked = 0
        for (namespace, _), marker in collected.items():
            if not namespace.endswith('/cartesian_proposals'):
                continue
            identity = namespace.rsplit('/', 1)[0]
            ellipsoids = [m for (ns, _), m in collected.items() if ns == identity+'/support']
            assert ellipsoids, 'Sample marker has no matching support ellipsoids'
            for p in marker.points:
                x = np.array([p.x, p.y, p.z])
                radii = []
                for m in ellipsoids:
                    q = m.pose.orientation
                    axes = Rotation.from_quat([q.x,q.y,q.z,q.w]).as_matrix()
                    c = m.pose.position
                    local = axes.T@(x-np.array([c.x,c.y,c.z]))
                    radii.append(np.sum((local/(np.array([m.scale.x,m.scale.y,m.scale.z])/2))**2))
                assert min(radii) <= 1+1e-6, 'Raw proposal outside matching GMM support'
                checked += 1
        assert checked > 0, 'No actual planner proposals received'
        assert any(m.points for (ns, _), m in collected.items() if ns.endswith('/valid_fk'))
        assert all(mode in paths and len(paths[mode].points)>1 for mode in ('cartesian_ik','joint_projected','uniform'))
        result = dict(raw_proposals_checked=checked, request_stage_markers=len(collected),
                      paths_received=list(paths), support_check='PASS', valid_fk_received=True)
        (args.output/'visual_stream_checks.json').write_text(json.dumps(result,indent=2))
        print(json.dumps(result))
    finally:
        if process.poll() is None: process.terminate(); process.wait(timeout=10)
        log.close(); node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__':
    main()
