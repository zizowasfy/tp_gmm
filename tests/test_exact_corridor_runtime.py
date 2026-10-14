#!/usr/bin/env python3
"""Live regression: a box corner is invalid in OMPL and in the offline audit."""
import argparse
import json
from pathlib import Path
import sys
import numpy as np
import rclpy
from moveit_msgs.msg import RobotTrajectory
from moveit_msgs.srv import GetStateValidity
from trajectory_msgs.msg import JointTrajectoryPoint
from tp_gmm.msg import Gaussian, GaussianMixture
from tp_gmm.srv import AuditTrajectory
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from run_sampling_study import StudyRunner
from compare_sampling_approaches import pose


class NativePreflightRunner(StudyRunner):
    def endpoint_valid(self, state, constraints, corridor_json=''):
        # Deliberately bypass the Python exact preflight to exercise OMPL itself.
        return self.service(self.validity_client, GetStateValidity.Request(
            robot_state=state, group_name=self.group_name, constraints=constraints)).valid


def main():
    rclpy.init()
    node = NativePreflightRunner(argparse.Namespace(task='franka_pick_cube', ref_robot='native',
                                clearance=.7, visualize=False, sampler_config=None, cutoff=3.))
    saved = False
    try:
        node.check_configuration(); node.save_scene(); saved = True
        case = dict(start=[.45,0.,.55], goal=[.45,0.,.55], obstacle=[.8,.4,.2], radius=.03,height=.2)
        node.set_case_scene(case)
        start = node.solve_state(pose(case['start'], node.frame_id))
        x = node.endpoint_fk(start).position
        model = GaussianMixture(weights=[1.]); model.header.frame_id = node.frame_id
        model.gaussians = [Gaussian(means=[0.,x.x-.05,x.y-.05,x.z],
            covariances=np.diag([1.,.0004,.0004,.0004]).ravel().tolist())]
        node.deformed_model, node.deformed_reference = model, None
        scene = node.scene_snapshot()
        prepared = node.prepare_for_plan(start, start, case, 'cartesian_ik', 4)
        try:
            assert node.endpoint_valid(start, prepared.constraints), 'Fixture must be inside native box'
            trajectory = RobotTrajectory()
            trajectory.joint_trajectory.joint_names = [n for n in start.joint_state.name if n.startswith('panda_joint')]
            q = dict(zip(start.joint_state.name, start.joint_state.position))
            trajectory.joint_trajectory.points = [JointTrajectoryPoint(positions=[q[n] for n in trajectory.joint_trajectory.joint_names]) for _ in range(2)]
            audited = node.service(node.audit_client, AuditTrajectory.Request(scene=scene, trajectory_start=start,
                trajectory=trajectory, constraints=prepared.constraints, corridor_json=prepared.corridor_json,
                group_name=node.group_name, link_name=node.ee_link, joint_step=.005))
            assert audited.success, audited.message
            audit = json.loads(audited.json)
            assert audit['audit_invalid_samples'] == audit['audit_samples'] == audit['exact_support_violations'], audit
        finally:
            node.sampling.release(prepared.request_id)
        row = node.study_plan(start, start, case, 'cartesian_ik', 4, .5, scene)
        assert not row['planner_success'] and row['sampling']['exact_support_rejections'] > 0, row
        print(json.dumps(dict(result='PASS', error_code=row['error_code'],
            exact_support_checks=row['sampling']['exact_support_checks'],
            exact_support_rejections=row['sampling']['exact_support_rejections'], audit=audit)))
    finally:
        if saved: node.restore_scene()
        node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()
