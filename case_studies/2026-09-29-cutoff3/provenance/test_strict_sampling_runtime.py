#!/usr/bin/env python3
"""Requires isolated sampling_demo.launch.py; deliberately exercises sampler exhaustion."""
import argparse
import json
from pathlib import Path
import sys
import numpy as np
import rclpy
from tp_gmm.msg import Gaussian, GaussianMixture
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from run_sampling_study import StudyRunner
from compare_sampling_approaches import pose


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cutoff',type=float,default=2.)
    args,ros_args=parser.parse_known_args()
    rclpy.init(args=ros_args)
    node = StudyRunner(argparse.Namespace(task='franka_pick_cube',ref_robot='native',clearance=.2,
                                         visualize=False,sampler_config=None,cutoff=args.cutoff))
    try:
        node.check_configuration()
        case = dict(start=[.45,-.03,.55],goal=[.45,.03,.55],obstacle=[.8,.4,.2],radius=.03,height=.2)
        start,goal = [node.solve_state(pose(case[k],node.frame_id)) for k in ('start','goal')]
        # Both endpoints fit inside cutoff=2, but the sole mean at x=10 m is
        # unreachable, so joint projection has exactly zero anchors/proposals.
        model = GaussianMixture(weights=[1.])
        model.header.frame_id = node.frame_id
        model.gaussians = [Gaussian(means=[0.,10.,0.,.55],
            covariances=np.diag([1.,25.,.25,.25]).ravel().tolist())]
        node.deformed_model = model
        node.deformed_reference = None
        row = node.study_plan(start,goal,case,'joint_projected',7,.5,node.scene_snapshot())
        stats = row['sampling']
        assert not row['success'] and row['error_code'] in (-6, 99999), (row['success'],row.get('error_code'),stats)
        assert stats['anchors'] == 0 and stats['valid_samples'] == 0
        assert stats['missing_anchor_rejections'] > 0 and stats['sampler_calls'] > 3
        assert stats['uniform_attempts'] == 0 and stats['online_ik_calls'] == 0
        assert .4 < row['action_wall_s'] < 5, row['action_wall_s']
        print(json.dumps(dict(result='PASS',cutoff=args.cutoff,action_wall_s=row['action_wall_s'],error_code=row['error_code'],
                              sampler_calls=stats['sampler_calls'],attempts=stats['attempts'],
                              uniform_attempts=stats['uniform_attempts'],anchors=stats['anchors'])))
    finally:
        node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()
