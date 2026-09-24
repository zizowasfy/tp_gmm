#!/usr/bin/env python3
"""Paired plan-only experiment: GMM, GMR neighborhoods, hybrid and direct-reference IK.

Use an isolated sampling_demo.launch.py. No Gazebo entity or execution commands.
"""
import argparse
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import random
import time
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from rosidl_runtime_py.convert import message_to_ordereddict
from tp_gmm_sampling import DEFAULTS
from tp_gmm_sampling.reference_baseline import ReferenceIKBaseline, resample_reference
from compare_sampling_approaches import ComparisonRunner, pose, save_results
from clearance_analysis import cylinder_distance


def variants(widths, fraction, direct=True):
    items = [('gmm',dict(proposal='gmm'))]
    for width in widths:
        for proposal in ('gmr_path','hybrid'):
            items.append((f'{proposal}/{width:g}m', dict(proposal=proposal, gmr_stddev=width, gmr_fraction=fraction)))
    if direct:
        items.append(('reference_ik', None))
    return items


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cases',type=Path,default=Path(get_package_share_directory('tp_gmm'))/'config/comparison_cases.json')
    parser.add_argument('--sampler-config',type=Path)
    parser.add_argument('--gmr-stddevs',nargs='+',type=float,default=[.005,.015,.03])
    parser.add_argument('--gmr-fraction',type=float,default=.8)
    parser.add_argument('--repeats',type=int,default=3)
    parser.add_argument('--warmup',type=int,default=1)
    parser.add_argument('--seed',type=int,default=42)
    parser.add_argument('--planning-time',type=float,default=3.)
    parser.add_argument('--task',default='franka_pick_cube')
    parser.add_argument('--ref-robot',default='native')
    parser.add_argument('--clearance',type=float,default=.5)
    parser.add_argument('--visualize',action='store_true')
    parser.add_argument('--skip-direct',action='store_true')
    parser.add_argument('--allow-no-policy',action='store_true')
    parser.add_argument('--output',type=Path,default=Path('sampling_results')/time.strftime('gmr-%Y%m%d-%H%M%S'))
    args,ros_args=parser.parse_known_args()
    if (args.repeats<1 or args.warmup<0 or not np.isfinite(args.planning_time) or args.planning_time<=0
        or not 0<=args.gmr_fraction<=1 or any(not np.isfinite(x) or not 0<=x<=.5 for x in args.gmr_stddevs)
        or len(set(args.gmr_stddevs))!=len(args.gmr_stddevs)):
        parser.error('Invalid repeat count, budget, unique widths or mixing fraction')
    if (args.output/'results.json').exists():
        parser.error('Output already contains results; select a fresh directory')
    if args.seed < 0 or args.seed >= 2**31 or not np.isfinite(args.clearance) or not 0<=args.clearance<=1:
        parser.error('Seed must be in [0,2^31) and clearance in [0,1]')
    cases=json.loads(args.cases.read_text())
    from clearance_analysis import validate_case
    if not cases or len({case['name'] for case in cases}) != len(cases):
        parser.error('Provide nonempty cases with unique names')
    for case in cases: validate_case(case)
    selections=variants(args.gmr_stddevs,args.gmr_fraction,not args.skip_direct)
    labels=[label for label,_ in selections]
    rclpy.init(args=ros_args)
    runner=ComparisonRunner(args)
    baseline=ReferenceIKBaseline(runner)
    # Fair ablation: these three proposals have no extra constrained-uniform draws.
    options=runner.sampler_options | dict(uniform_fraction=0.,cartesian_fraction=0.)
    runner.sampler_options=options
    metadata=dict(task=args.task, seed=args.seed, planning_time=args.planning_time, modes=labels, cases=cases,
                  settings=DEFAULTS|options, variants=dict(selections), models={}, references={}, endpoint_states={},
                  visualize=args.visualize, timing_note='Shared TP-GMM/RL/DSGMR per paired repeat; sampler seeds paired; independent OMPL/IK RNGs remain',
                  baseline_note='Sequential reference IK includes start/goal connectors and fixed goal joint state. Public service audit overhead included; no time parameterization/execution.')
    rows=[]; rng=random.Random(args.seed); saved=False
    try:
        runner.save_scene(); saved=True
        for case_index,case in enumerate(cases):
            runner.set_case_scene(case)
            start_pose,goal_pose=pose(case['start'],runner.frame_id),pose(case['goal'],runner.frame_id)
            obstacle_top=list(case['obstacle']); obstacle_top[2]+=case['height']/2
            obstacle_pose=pose(obstacle_top,runner.frame_id,False)
            runner.obstacle_radius=case['radius']
            try:
                start,goal=runner.solve_state(start_pose),runner.solve_state(goal_pose)
            except RuntimeError as error:
                for repeat in range(args.repeats):
                    for label in labels:
                        rows.append(dict(case=case['name'], repeat=repeat, mode=label, warmup=False, success=False,
                                         failure_stage='endpoint_ik',error=str(error)))
                save_results(args.output,rows,metadata,labels)
                continue
            metadata['endpoint_states'][case['name']]=dict(start=message_to_ordereddict(start),goal=message_to_ordereddict(goal))
            for repeat in range(-args.warmup,args.repeats):
                if not runner.call_deform_tpgmm_service(start_pose,goal_pose,obstacle_pose):
                    raise RuntimeError('Deformation service failed')
                if not runner.last_pipeline_metrics['policy_applied'] and not args.allow_no_policy:
                    raise RuntimeError('RL policy was not applied')
                model=message_to_ordereddict(runner.deformed_model)
                reference=message_to_ordereddict(runner.deformed_reference)
                digest=hashlib.sha256(json.dumps(model,sort_keys=True).encode()).hexdigest()
                ref_digest=hashlib.sha256(json.dumps(reference,sort_keys=True).encode()).hexdigest()
                metadata['models'][digest]=model; metadata['references'][ref_digest]=reference
                metadata['policy']=runner.last_policy_metadata
                ref_points=resample_reference(runner.deformed_reference,.002)
                reference_clearance=float(cylinder_distance(ref_points,case['obstacle'],case['radius'],case['height']).min())
                endpoint_errors=dict(reference_start_error_m=float(np.linalg.norm(ref_points[0]-case['start'])),
                                     reference_goal_error_m=float(np.linalg.norm(ref_points[-1]-case['goal'])))
                order=list(selections); rng.shuffle(order)
                seed=args.seed+(repeat+args.warmup)*101+case_index*10007
                for label,settings in order:
                    runner.sampler_options=options | (settings or dict(proposal='gmm'))
                    row=dict(case=case['name'],repeat=repeat,warmup=repeat<0,mode=label,mapping='cartesian_ik',seed=seed,
                             model_sha256=digest,reference_sha256=ref_digest,reference_min_obstacle_clearance_m=reference_clearance,
                             pipeline=deepcopy(runner.last_pipeline_metrics))
                    row.update(endpoint_errors)
                    if settings is None:
                        prepare_begin=time.perf_counter()
                        prepared=runner.prepare_sampling(goal_pose, mode='cartesian_ik', seed=seed, proposal='gmm')
                        preparation_wall=time.perf_counter()-prepare_begin
                        try:
                            joint_names=[name for name in goal.joint_state.name if name.startswith('panda_joint')]
                            result=baseline.plan(runner.deformed_reference,start,goal,joint_names,runner.group_name,
                                                 runner.ee_link,goal_pose.pose.orientation,prepared.constraints,args.planning_time)
                            if result.get('ee_points'):
                                result['sampling']['ee_min_obstacle_clearance_m']=float(cylinder_distance(result['ee_points'],case['obstacle'],case['radius'],case['height']).min())
                            row.update(result)
                            row['prepare_service_wall_s']=preparation_wall
                        finally:
                            runner.sampling.release(prepared.request_id)
                    else:
                        row.update(runner.plan(start,goal,goal_pose,'cartesian_ik',seed,args.planning_time))
                    ee=row.get('sampling',{}).get('ee_path',row.get('ee_points',[]))
                    if ee:
                        row['sampling']['ee_min_obstacle_clearance_m']=float(cylinder_distance(ee,case['obstacle'],case['radius'],case['height']).min())
                    row['end_to_end_wall_s']=(row['pipeline']['deform_service_wall_s']+
                        row.get('preflight_wall_s',row.get('prepare_service_wall_s',0))+row.get('action_wall_s',0))
                    rows.append(row); save_results(args.output,rows,metadata,labels)
                    runner.get_logger().info(f"{case['name']} repeat={repeat} {label}: success={row['success']} stage={row.get('failure_stage')}")
        from plot_sampling_comparison import plot
        plot(args.output/'results.json')
        from plot_gmr_comparison import plot as plot_gmr
        plot_gmr(args.output/'results.json')
        print((args.output/'summary.md').read_text())
    finally:
        if saved: runner.restore_scene()
        runner.destroy_node(); rclpy.shutdown()

if __name__=='__main__':
    main()
