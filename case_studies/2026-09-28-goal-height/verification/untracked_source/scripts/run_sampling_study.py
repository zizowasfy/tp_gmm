#!/usr/bin/env python3
"""Fixed-design, paired, plan-only comparison; stops on purity/provenance errors."""
import argparse
from copy import deepcopy
import hashlib
import json
import os
from pathlib import Path
import platform
import subprocess
import time
import numpy as np
import rclpy
from geometry_msgs.msg import PoseArray
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, PlanningSceneComponents, RobotState
from moveit_msgs.srv import GetPlanningScene, GetStateValidity
from rcl_interfaces.srv import GetParameters
from rosidl_runtime_py.convert import message_to_ordereddict as asdict
from rosidl_runtime_py.set_message import set_message_fields
from tp_gmm.srv import AuditTrajectory
from run_clearance_sweep import ClearanceRunner, xyz
from compare_sampling_approaches import pose, wait
from clearance_analysis import DEFAULT_ENVIRONMENT, random_environment, curve_metrics
from sampling_client import DEFAULTS

MODES = ('cartesian_ik', 'joint_projected', 'ompl_uniform')
PURE = DEFAULTS | dict(corridor_mode='covariance', cutoff=2., uniform_fraction=0., cartesian_fraction=0., proposal='gmm')


def save(path, obj):
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix+'.tmp')
    temporary.write_text(json.dumps(obj, indent=2, allow_nan=False)+'\n')
    temporary.replace(path)


def digest(obj):
    return hashlib.sha256(json.dumps(obj, sort_keys=True).encode()).hexdigest()


def message(kind, fields):
    result = kind()
    # rosidl serializes byte-valued CollisionObject.operation as a one-character
    # string; its setter requires bytes when restoring a frozen scene.
    def restore_bytes(value):
        if isinstance(value, dict):
            return {k:(bytes([ord(v)]) if k == 'operation' and isinstance(v,str) and len(v) == 1 else restore_bytes(v))
                    for k,v in value.items()}
        if isinstance(value,list):
            return [restore_bytes(v) for v in value]
        return value
    set_message_fields(result, restore_bytes(fields))
    return result


def assert_pure(mode, stats):
    if mode == 'ompl_uniform':
        assert not stats, 'Unrestricted baseline unexpectedly used GMM plugin'
        return
    for key in ('uniform_attempts', 'missing_anchor_fallbacks', 'gmr_attempts'):
        if stats.get(key, 0) != 0:
            raise RuntimeError(f'Purity violation: {mode}: {key}')
    forbidden = 'online_ik_calls' if mode == 'joint_projected' else 'projected_attempts'
    if stats.get(forbidden, 0) != 0:
        raise RuntimeError(f'Purity violation: {mode}: {forbidden}')
    if stats.get('proposal') != 'gmm' or any(c != 2. for c in stats['component_cutoffs']):
        raise RuntimeError('Proposal or cutoff changed')


class StudyRunner(ClearanceRunner):
    def __init__(self, args):
        super().__init__(args)
        self.sampler_options = dict(PURE)
        self.audit_client = self.create_client(AuditTrajectory, '/sampling_study/audit')
        self.parameter_client = self.create_client(GetParameters, '/move_group/get_parameters')

    def check_configuration(self):
        names = ['ompl.panda_arm.allow_constraint_sampler_fallback',
                 'ompl.panda_arm.enforce_joint_model_state_space',
                 'ompl.panda_arm.longest_valid_segment_fraction',
                 'ompl.planner_configs.RRTConnectkConfigDefault.simplify_solutions']
        values = self.service(self.parameter_client, GetParameters.Request(names=names)).values
        if values[0].type != 1 or values[0].bool_value or values[1].type != 1 or not values[1].bool_value:
            raise RuntimeError('Requires strict fallback=false and enforced joint model state space; use sampling_demo.launch.py')
        return dict(zip(names, [asdict(v) for v in values]))

    def scene_snapshot(self):
        req = GetPlanningScene.Request()
        req.components.components = 1023  # All documented PlanningSceneComponents bits.
        scene = self.service(self.scene_client, req).scene
        if scene.is_diff:
            raise RuntimeError('Expected complete scene snapshot')
        return scene

    def study_plan(self, start, goal, case, mode, seed, budget, scene):
        prepared = None
        begin = time.perf_counter()
        row = dict(success=False, planner_success=False, sampling={}, action_wall_s=0., moveit_planning_s=0.)
        try:
            constraints = Constraints()
            if mode != 'ompl_uniform':
                prepared = self.prepare_sampling(pose(case['goal'], self.frame_id), mode=mode, seed=seed)
                constraints = prepared.constraints
            row['prepare_service_wall_s'] = time.perf_counter()-begin
            row['path_constraints_sha256'] = digest(asdict(constraints))
            # Invalid corridor endpoints are recorded as failures, never excluded from study.
            for label, state in [('start',start), ('goal',goal)]:
                validity = self.service(self.validity_client, GetStateValidity.Request(
                    robot_state=state, group_name=self.group_name, constraints=constraints))
                if not validity.valid:
                    row['failure_stage'] = label+'_invalid_in_corridor' if prepared else label+'_invalid'
                    return row
            request = MoveGroup.Goal()
            motion = request.request
            motion.group_name, motion.pipeline_id, motion.planner_id = self.group_name, 'ompl', 'RRTConnectkConfigDefault'
            motion.num_planning_attempts = 1
            motion.allowed_planning_time = float(budget)
            motion.max_velocity_scaling_factor = motion.max_acceleration_scaling_factor = .8
            motion.start_state = deepcopy(start)
            motion.start_state.is_diff = False
            motion.path_constraints = constraints
            target = Constraints()
            for name, q in zip(goal.joint_state.name, goal.joint_state.position):
                if name.startswith('panda_joint'):
                    target.joint_constraints.append(JointConstraint(joint_name=name, position=q,
                        tolerance_above=.001, tolerance_below=.001, weight=1.))
            motion.goal_constraints.append(target)
            request.planning_options.plan_only = True
            request.planning_options.replan = False
            row['preflight_wall_s'] = time.perf_counter()-begin
            action_begin = time.perf_counter()
            handle = wait(self, self.move_action_client.send_goal_async(request), 10)
            if not handle.accepted:
                raise RuntimeError('Planning action rejected')
            future = handle.get_result_async()
            try:
                result = wait(self, future, budget+30).result
            except TimeoutError:
                wait(self, handle.cancel_goal_async(), 5)
                wait(self, future, 15)
                raise
            row.update(action_wall_s=time.perf_counter()-action_begin, moveit_planning_s=result.planning_time,
                       error_code=result.error_code.val, planner_success=result.error_code.val == 1,
                       trajectory=asdict(result.planned_trajectory), trajectory_start=asdict(result.trajectory_start))
            row['planning_pipeline_wall_s'] = time.perf_counter()-begin
            if row['planner_success']:
                audit_begin = time.perf_counter()
                audited = self.service(self.audit_client, AuditTrajectory.Request(scene=scene,
                    trajectory_start=result.trajectory_start, trajectory=result.planned_trajectory,
                    constraints=constraints, group_name=self.group_name, link_name=self.ee_link, joint_step=.01), timeout=60)
                if not audited.success:
                    raise RuntimeError('Audit failed: '+audited.message)
                row['audit'] = json.loads(audited.json)
                row['audit_wall_s'] = time.perf_counter()-audit_begin
                row['success'] = row['audit']['audit_invalid_samples'] == 0
                row['curve'] = {k:v for k,v in curve_metrics(row['audit']['ee_path'], case, .002, .1).items() if not isinstance(v,list)}
            row['failure_stage'] = None if row['success'] else ('path_audit' if row['planner_success'] else 'planning')
            return row
        finally:
            if prepared:
                row['sampling'] = self.sampling.report(prepared.request_id, release=True)
            assert_pure(mode, row['sampling'])
            row.setdefault('planning_pipeline_wall_s', time.perf_counter()-begin)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--environments-per-stratum', type=int, default=10)
    parser.add_argument('--repeats', type=int, default=10)
    parser.add_argument('--planning-time', type=float, default=3.)
    parser.add_argument('--seed', type=int, default=20260926)
    parser.add_argument('--task', default='franka_pick_cube')
    parser.add_argument('--study-phase', choices=['primary','path_quality_confirmation'], default='primary')
    parser.add_argument('--resume', action='store_true')
    parser.add_argument('--design-only', action='store_true')
    args, ros_args = parser.parse_known_args()
    if min(args.environments_per_stratum,args.repeats) < 1 or not 0 < args.planning_time <= 60 or not 0 <= args.seed < 2**31:
        parser.error('Invalid sample size/budget/seed')
    args.ref_robot, args.clearance, args.visualize, args.sampler_config = 'native', .2, False, None
    args.output.mkdir(parents=True,exist_ok=True)
    manifest_path = args.output/'manifest.json'
    if manifest_path.exists() and not args.resume:
        parser.error('Output already has a manifest; use --resume or another directory')
    rclpy.init(args=ros_args)
    runner = StudyRunner(args)
    rows_path = args.output/'results.jsonl'
    old_rows = [json.loads(line) for line in rows_path.read_text().splitlines()] if rows_path.exists() else []
    completed = {(r['environment'],r['repeat'],r['mode']) for r in old_rows}
    if len(completed) != len(old_rows):
        raise ValueError('Duplicate trial keys')
    saved_scene = False
    try:
        runtime = runner.check_configuration()
        runner.save_scene(); saved_scene = True
        if manifest_path.exists():
            manifest = json.loads(manifest_path.read_text())
            expected = dict(seed=args.seed, repeats=args.repeats, environments_per_stratum=args.environments_per_stratum, planning_time=args.planning_time)
            if manifest.get('study_phase','primary') != args.study_phase:
                raise ValueError('Resume study phase differs from manifest')
            if manifest['design'] != expected or manifest['settings'] != PURE or manifest['runtime'] != runtime:
                raise ValueError('Resume settings differ from frozen manifest')
        else:
            design_rng = np.random.default_rng(args.seed)
            manifest = dict(schema=1, status='designing', created_utc=time.strftime('%Y-%m-%dT%H:%M:%SZ',time.gmtime()),
                design=dict(seed=args.seed,repeats=args.repeats,environments_per_stratum=args.environments_per_stratum,planning_time=args.planning_time),
                settings=PURE,runtime=runtime,modes=MODES,environment_ranges=DEFAULT_ENVIRONMENT,
                environments=[],rejected=[],host=dict(platform=platform.platform(),processor=platform.processor(),cpus=os.cpu_count()),
                seed_note='Environment, order and custom proposals seeded; OMPL and IK plugin have independent RNG streams. Repeats are paired by environment, not common OMPL random numbers.',
                analysis=dict(independent_unit='environment',bootstrap=20000,permutations=100000,alpha=.05,
                    primary_family='success and PAR2, all three pairwise contrasts, Holm six tests',
                    success_margin=.05,minimum_runtime_improvement=.10,
                    stopping='Fixed sample size; no significance-based stopping or outcome-based exclusions'))
            manifest['study_phase'] = args.study_phase
            if args.study_phase == 'path_quality_confirmation':
                manifest['analysis']['primary_family'] = 'Cartesian minus projected matched-success joint and EE path length; two one-sided environment sign tests, Holm correction'
                manifest['analysis']['secondary'] = 'Reliability, runtime and baseline contrasts; kept separate from original primary study'
                manifest['analysis']['confirmation_hypothesis'] = 'Projected method produces shorter paths on independent new environments; positive Cartesian-minus-projected differences'
            for repo in ('tp_gmm','moveit2','moveit_resources'):
                root = Path(__file__).resolve().parents[2]/repo
                if (root/'.git').exists():
                    manifest.setdefault('git',{})[repo] = dict(head=subprocess.check_output(['git','rev-parse','HEAD'],cwd=root,text=True).strip(),
                        branch=subprocess.check_output(['git','branch','--show-current'],cwd=root,text=True).strip(),
                        diff_sha256=hashlib.sha256(subprocess.check_output(['git','diff','HEAD'],cwd=root)).hexdigest())
            source_root = Path(__file__).resolve().parents[1]
            manifest['source_sha256'] = {str(p.relative_to(source_root)):hashlib.sha256(p.read_bytes()).hexdigest()
                for folder in ('src','include','scripts','python','srv','launch','config')
                for p in (source_root/folder).rglob('*') if p.is_file() and '__pycache__' not in str(p)}
            save(manifest_path,manifest)
            for layout, offset in [('central',0.),('offset',.07),('distant',.18)]:
                for clearance in (.2,.8):
                    for replicate in range(args.environments_per_stratum):
                        index = len(manifest['environments'])
                        for attempt in range(100):
                            case = random_environment(design_rng,DEFAULT_ENVIRONMENT,index)
                            case['obstacle'][1] += float(design_rng.choice([-1,1]))*offset
                            case['name'] = f'{layout}_c{clearance:.1f}_{replicate:02d}'
                            runner.set_case_scene(case)
                            try:
                                start = runner.solve_state(pose(case['start'],runner.frame_id))
                                goal = runner.solve_state(pose(case['goal'],runner.frame_id))
                            except RuntimeError as error:
                                if not str(error).startswith('Benchmark endpoint IK failed'):
                                    raise
                                manifest['rejected'].append(dict(environment=index,attempt=attempt,case=case,reason=str(error)))
                                continue
                            break
                        else:
                            raise RuntimeError('No endpoint-feasible environment after 100 candidates')
                        response, _, deformation_wall = runner.deform(case,clearance,'top')
                        if manifest.get('policy_sha256',response.policy_sha256) != response.policy_sha256:
                            raise RuntimeError('Policy changed during design')
                        manifest['policy_sha256'] = response.policy_sha256
                        pipeline = {k:getattr(response,k) for k in ('model_load_s','reproduction_s','rl_deformation_s','deformed_regression_s','total_s')}
                        content = dict(case=case,start=asdict(start),goal=asdict(goal),scene=asdict(runner.scene_snapshot()),
                            response=asdict(response),pipeline=pipeline,deformation_wall_s=deformation_wall)
                        relative = f'environments/{index:03d}.json'
                        save(args.output/relative,content)
                        manifest['environments'].append(dict(id=index,stratum=f'{layout}_c{clearance:.1f}',case=case,
                            clearance=clearance,file=relative,sha256=digest(content)))
                        save(manifest_path,manifest)
                        print(f'DESIGN {index+1}/{6*args.environments_per_stratum}: {case["name"]}',flush=True)
            order_rng = np.random.default_rng(args.seed+1)
            schedule = []
            for index in order_rng.permutation(len(manifest['environments'])):
                for repeat in range(args.repeats):
                    for mode in order_rng.permutation(MODES):
                        schedule.append(dict(environment=int(index),repeat=repeat,mode=str(mode),seed=int(args.seed+int(index)*10007+repeat*101)))
            manifest.update(status='frozen',schedule=schedule)
            save(manifest_path,manifest)
        if len(manifest['environments']) != 6*args.environments_per_stratum or 'schedule' not in manifest:
            raise RuntimeError('Incomplete design; use a new output directory')
        if args.design_only:
            return
        from tp_gmm.srv import DeformTPGMM
        from moveit_msgs.msg import PlanningScene
        active = None
        for trial in manifest['schedule']:
            key = (trial['environment'],trial['repeat'],trial['mode'])
            if key in completed:
                continue
            if active != trial['environment']:
                active = trial['environment']
                environment = manifest['environments'][active]
                content = json.loads((args.output/environment['file']).read_text())
                if digest(content) != environment['sha256']:
                    raise RuntimeError('Frozen environment changed')
                runner.set_case_scene(content['case'])
                response = message(DeformTPGMM.Response,content['response'])
                runner.original_model,runner.deformed_model = response.original_gmm,response.deformed_gmm
                runner.original_reference,runner.deformed_reference = response.original_trajectory,response.deformed_trajectory
                start,goal = message(RobotState,content['start']),message(RobotState,content['goal'])
                scene = message(PlanningScene,content['scene'])
            result = runner.study_plan(start,goal,content['case'],trial['mode'],trial['seed'],args.planning_time,scene)
            row = trial | dict(stratum=environment['stratum'],clearance=environment['clearance'],pipeline=content['pipeline'],
                environment_sha256=environment['sha256']) | result
            row['end_to_end_wall_s'] = row['planning_pipeline_wall_s'] + (content['deformation_wall_s'] if trial['mode'] != 'ompl_uniform' else 0)
            # PAR2 uses observed action wall time capped at the common budget; all failures cost twice the budget.
            row['par2_s'] = min(row['action_wall_s'],args.planning_time) if row['success'] else 2*args.planning_time
            with rows_path.open('a') as output:
                output.write(json.dumps(row,allow_nan=False)+'\n'); output.flush(); os.fsync(output.fileno())
            completed.add(key)
            print(f'PLAN {len(completed)}/{len(manifest["schedule"])} env={active} repeat={trial["repeat"]} {trial["mode"]} success={row["success"]} wall={row["action_wall_s"]:.3f}',flush=True)
        manifest['status'] = 'complete'
        manifest['completed_utc'] = time.strftime('%Y-%m-%dT%H:%M:%SZ',time.gmtime())
        save(manifest_path,manifest)
    finally:
        if saved_scene:
            runner.restore_scene()
        runner.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
