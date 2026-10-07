#!/usr/bin/env python3
"""Requires isolated laboratory: unreachable GMR tube must exhaust, never fall back."""
import argparse
import json
from pathlib import Path
import sys
import numpy as np
import rclpy
from geometry_msgs.msg import PoseArray, Pose
from tp_gmm.msg import Gaussian, GaussianMixture
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from run_gmr_study import GMRStudyRunner
from compare_sampling_approaches import pose

def main():
    rclpy.init()
    node=GMRStudyRunner(argparse.Namespace(task='franka_pick_cube',ref_robot='native',clearance=.7,
                                         visualize=False,sampler_config=None,cutoff=3.))
    try:
        node.check_configuration()
        case=dict(start=[.45,-.03,.55],goal=[.45,.03,.55],obstacle=[.8,.4,.2],radius=.03,height=.2)
        start,goal=[node.solve_state(pose(case[k],node.frame_id)) for k in ('start','goal')]
        model=GaussianMixture(weights=[1.]);model.header.frame_id=node.frame_id
        model.gaussians=[Gaussian(means=[0.,10.,0.,.55],covariances=np.diag([1.,25.,.25,.25]).ravel().tolist())]
        node.deformed_model=model
        reference=PoseArray();reference.header.frame_id=node.frame_id
        for x in (10.,10.1):
            p=Pose();p.position.x=x;p.position.z=.55;p.orientation.w=1.;reference.poses.append(p)
        node.deformed_reference=reference
        row=node.compare_plan(start,goal,case,'gmr_30mm',71,.5,node.scene_snapshot())
        s=row['sampling']
        assert not row['success'] and row['error_code'] in (-6,99999),row
        assert s['attempts']==s['gmr_attempts'] and s['attempts']>3
        assert s['gmm_attempts']==s['uniform_attempts']==s['valid_samples']==0
        assert .4<row['action_wall_s']<5
        print(json.dumps(dict(result='PASS',action_wall_s=row['action_wall_s'],error_code=row['error_code'],
            attempts=s['attempts'],gmr_attempts=s['gmr_attempts'],gmm_attempts=s['gmm_attempts'],
            uniform_attempts=s['uniform_attempts'],valid_samples=s['valid_samples'])))
    finally:
        node.destroy_node();rclpy.shutdown()


if __name__ == '__main__':
    main()
