#!/usr/bin/env python3
"""Paired, plan-only sampler comparison. Never commands robot motion or Gazebo entities."""
import argparse
import csv
import hashlib
import json
import math
import random
import time
from copy import deepcopy
from pathlib import Path
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Pose
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (PlanningScene, CollisionObject, JointConstraint, Constraints,
                             RobotState, PlanningSceneComponents)
from moveit_msgs.srv import GetPositionIK, GetPlanningScene, ApplyPlanningScene, GetStateValidity
from shape_msgs.msg import SolidPrimitive
from rosidl_runtime_py.convert import message_to_ordereddict
from ament_index_python.packages import get_package_share_directory
from run_experiments import GazeboExperimentRunner
from sampling_client import MODES, DEFAULTS


def pose(xyz, frame, downward=True):
    value = PoseStamped()
    value.header.frame_id = frame
    value.pose.position.x, value.pose.position.y, value.pose.position.z = map(float, xyz)
    if downward:
        value.pose.orientation.x = 1.0
        value.pose.orientation.w = 0.0
    else:
        value.pose.orientation.w = 1.0
    return value


def wait(node, future, timeout):
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
    if not future.done():
        raise TimeoutError(f'ROS operation exceeded {timeout}s')
    result = future.result()
    if result is None:
        raise RuntimeError('Empty ROS result')
    return result


class ComparisonRunner(GazeboExperimentRunner):
    def __init__(self, args):
        super().__init__(task_name=args.task, ref_robot=args.ref_robot,
                         desired_clearance=args.clearance, visualize_samples=args.visualize,
                         sampler_options=json.loads(args.sampler_config.read_text()) if args.sampler_config else None)
        self.ik_client = self.create_client(GetPositionIK, '/compute_ik')
        self.scene_client = self.create_client(GetPlanningScene, '/get_planning_scene')
        self.validity_client = self.create_client(GetStateValidity, '/check_state_validity')
        self.saved_objects = []

    def service(self, client, request, timeout=15):
        if not client.wait_for_service(timeout_sec=5):
            raise RuntimeError(f'Service unavailable: {client.srv_name}')
        return wait(self, client.call_async(request), timeout)

    def save_scene(self):
        req = GetPlanningScene.Request()
        req.components.components = PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
        scene = self.service(self.scene_client, req).scene
        self.saved_objects = [deepcopy(obj) for obj in scene.world.collision_objects
                              if obj.id in ('tpgmm_benchmark_table', 'tpgmm_benchmark_obstacle')]

    def restore_scene(self):
        scene = PlanningScene(is_diff=True)
        scene.robot_state.is_diff = True
        for name in ('tpgmm_benchmark_table', 'tpgmm_benchmark_obstacle'):
            scene.world.collision_objects.append(CollisionObject(id=name, operation=CollisionObject.REMOVE))
        scene.world.collision_objects.extend(self.saved_objects)
        self.apply_scene(scene)

    def apply_scene(self, scene):
        result = self.service(self.apply_planning_scene_client, ApplyPlanningScene.Request(scene=scene))
        if not result.success:
            raise RuntimeError('Planning scene update rejected')

    def set_case_scene(self, case):
        scene = PlanningScene(is_diff=True)
        scene.robot_state.is_diff = True
        for name, xyz, kind, dimensions in [
            ('table', [0.6, 0, 0.125], SolidPrimitive.BOX, [0.75, 1.0, 0.25]),
            ('obstacle', case['obstacle'], SolidPrimitive.CYLINDER, [case['height'], case['radius']])]:
            obj = CollisionObject(id=f'tpgmm_benchmark_{name}', operation=CollisionObject.ADD)
            obj.header.frame_id = self.frame_id
            obj.primitives.append(SolidPrimitive(type=kind, dimensions=list(map(float, dimensions))))
            obj.primitive_poses.append(pose(xyz, self.frame_id, downward=False).pose)
            scene.world.collision_objects.append(obj)
        self.apply_scene(scene)

    def solve_state(self, target):
        req = GetPositionIK.Request()
        req.ik_request.group_name = self.group_name
        req.ik_request.ik_link_name = self.ee_link
        req.ik_request.pose_stamped = target
        req.ik_request.avoid_collisions = True
        req.ik_request.timeout.sec = 2
        seed = RobotState()
        seed.joint_state.name = [f'panda_joint{i}' for i in range(1, 8)] + ['panda_finger_joint1', 'panda_finger_joint2']
        seed.joint_state.position = [0., -0.785, 0., -2.356, 0., 1.571, 0.785, 0.04, 0.04]
        req.ik_request.robot_state = seed
        result = self.service(self.ik_client, req)
        if result.error_code.val != result.error_code.SUCCESS:
            raise RuntimeError(f'Benchmark endpoint IK failed ({result.error_code.val})')
        return result.solution

    def plan(self, start, goal, target_pose, mode, seed, planning_time):
        begin = time.perf_counter()
        prepared = self.prepare_sampling(target_pose, mode=mode, seed=seed)
        preparation_wall = time.perf_counter() - begin
        result = None
        try:
            # Reject invalid endpoints explicitly; never silently remove the corridor.
            for label, state in [('start', start), ('goal', goal)]:
                req = GetStateValidity.Request(robot_state=state, group_name=self.group_name,
                                               constraints=prepared.constraints)
                if not self.service(self.validity_client, req).valid:
                    return dict(success=False, failure_stage=f'{label}_invalid_in_corridor',
                                request_id=prepared.request_id, prepare_service_wall_s=preparation_wall)
            request = MoveGroup.Goal()
            request.request.group_name = self.group_name
            request.request.pipeline_id = 'ompl'
            request.request.planner_id = 'RRTConnectkConfigDefault'
            request.request.num_planning_attempts = 1
            request.request.allowed_planning_time = float(planning_time)
            request.request.max_velocity_scaling_factor = 0.8
            request.request.max_acceleration_scaling_factor = 0.8
            request.request.start_state = deepcopy(start)
            request.request.start_state.is_diff = False
            request.request.path_constraints = prepared.constraints
            constraints = Constraints()
            for name, value in zip(goal.joint_state.name, goal.joint_state.position):
                if name.startswith('panda_joint'):
                    constraints.joint_constraints.append(JointConstraint(
                        joint_name=name, position=value, tolerance_above=0.001,
                        tolerance_below=0.001, weight=1.0))
            request.request.goal_constraints.append(constraints)
            request.planning_options.plan_only = True
            request.planning_options.replan = False
            preflight_wall = time.perf_counter() - begin
            action_begin = time.perf_counter()
            handle = wait(self, self.move_action_client.send_goal_async(request), 10)
            if not handle.accepted:
                raise RuntimeError('MoveGroup rejected goal')
            future = handle.get_result_async()
            try:
                result = wait(self, future, planning_time + 30).result
            except TimeoutError:
                wait(self, handle.cancel_goal_async(), 5)
                # Do not run the next paired trial while a cancelled planner could still be active.
                wait(self, future, 15)
                raise
            action_wall = time.perf_counter() - action_begin
            report = self.sampling.report(prepared.request_id, result, release=False)
            planner_success = result.error_code.val == result.error_code.SUCCESS
            audit_passed = report.get('path_validation_samples', 0) > 0 and report.get('path_invalid_samples', 0) == 0
            successes = planner_success and audit_passed
            return dict(success=successes, planner_success=planner_success, path_audit_passed=audit_passed, error_code=result.error_code.val,
                        failure_stage=None if successes else ('path_audit' if planner_success else 'planning'),
                        prepare_service_wall_s=preparation_wall, preflight_wall_s=preflight_wall, action_wall_s=action_wall,
                        moveit_planning_s=result.planning_time, sampling=report,
                        trajectory=message_to_ordereddict(result.planned_trajectory),
                        trajectory_start=message_to_ordereddict(result.trajectory_start))
        finally:
            self.sampling.report(prepared.request_id, release=True)


def summarize(rows, modes):
    summary = {}
    for mode in modes:
        trials = [row for row in rows if row['mode'] == mode and not row.get('warmup')]
        successful = [row for row in trials if row.get('success')]
        item = dict(trials=len(trials), successes=len(successful),
                    planner_successes=sum(r.get('planner_success',r.get('success',False)) for r in trials),
                    path_audit_failures=sum(r.get('failure_stage') == 'path_audit' for r in trials),
                    success_rate=len(successful)/len(trials) if trials else None)
        if trials:
            # Wilson interval is meaningful even with zero observed successes.
            n, p, z = len(trials), len(successful)/len(trials), 1.96
            center = (p + z*z/(2*n))/(1+z*z/n)
            half = z*math.sqrt(p*(1-p)/n+z*z/(4*n*n))/(1+z*z/n)
            item['success_rate_wilson95'] = [center-half, center+half]
        for key in ('action_wall_s', 'moveit_planning_s', 'end_to_end_wall_s'):
            for label, selection in [('all', trials), ('successful', successful)]:
                values = [r[key] for r in selection if r.get(key) is not None]
                item[f'{key}_{label}_median'] = float(np.median(values)) if values else None
                item[f'{key}_{label}_p95'] = float(np.percentile(values, 95)) if values else None
        for key in ('model_load_s', 'reproduction_s', 'rl_deformation_s', 'deformed_regression_s'):
            values = [r['pipeline'][key] for r in trials if r.get('pipeline', {}).get(key) is not None]
            item[f'{key}_median'] = float(np.median(values)) if values else None
        for key in ('joint_path_length', 'ee_path_length_m', 'min_world_clearance_m',
                    'sampling_s', 'setup_s', 'online_ik_s', 'anchor_ik_s', 'jacobian_setup_s',
                    'fk_mapping_s', 'draw_s', 'validity_s', 'attempts', 'valid_samples'):
            selection = successful if key in ('joint_path_length', 'ee_path_length_m', 'min_world_clearance_m') else trials
            values = [r['sampling'][key] for r in selection if r.get('sampling', {}).get(key) is not None]
            item[f'{key}_median'] = float(np.median(values)) if values else None
        attempted = sum(r.get('sampling', {}).get('attempts', 0) for r in trials)
        valid = sum(r.get('sampling', {}).get('valid_samples', 0) for r in trials)
        item['valid_per_attempt'] = valid/attempted if attempted else None
        item['inactive_sampler_trials'] = sum(not r.get('sampling', {}).get('sampler_active') for r in trials)
        summary[mode] = item
    return summary


def save_results(output, rows, metadata, modes):
    output.mkdir(parents=True, exist_ok=True)
    summary = summarize(rows, modes)
    from sampling_fidelity import distribution_report
    payload = dict(metadata=metadata, summary=summary, trials=rows,
                   distribution_checks=distribution_report(rows, metadata))
    temporary = output / 'results.tmp'
    temporary.write_text(json.dumps(payload, indent=2, allow_nan=False))
    temporary.replace(output / 'results.json')
    fields = ['case', 'repeat', 'mode', 'seed', 'warmup', 'success', 'planner_success', 'path_audit_passed', 'failure_stage',
              'action_wall_s', 'moveit_planning_s', 'end_to_end_wall_s', 'error_code']
    pipeline_keys = ['model_load_s', 'reproduction_s', 'rl_deformation_s', 'deformed_regression_s', 'deform_service_wall_s']
    stage_keys = ['setup_s', 'sampling_s', 'draw_s', 'online_ik_s', 'anchor_ik_s', 'jacobian_setup_s',
                  'fk_mapping_s', 'validity_s', 'attempts', 'valid_samples', 'joint_path_length',
                  'ee_path_length_m', 'min_world_clearance_m', 'path_invalid_samples', 'path_collision_samples', 'path_constraint_violations']
    with (output / 'trials.csv').open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=fields+pipeline_keys+stage_keys)
        writer.writeheader()
        for row in rows:
            writer.writerow({key: row.get(key, row.get('sampling', {}).get(key, row.get('pipeline', {}).get(key))) for key in fields+pipeline_keys+stage_keys})
    lines = ['# Sampling comparison', '', 'Success requires both MoveIt success and the interpolated path audit. Timing runs exclude RViz sample publication unless requested.', '',
             '| Mode | Success | Action median (s), all | Action p95 (s), all | Valid/attempt |',
             '|---|---:|---:|---:|---:|']
    fmt = lambda value: '—' if value is None else f'{value:.4f}'
    for mode, s in summary.items():
        lines.append(f"| {mode} | {s['successes']}/{s['trials']} | {fmt(s['action_wall_s_all_median'])} | {fmt(s['action_wall_s_all_p95'])} | {fmt(s['valid_per_attempt'])} |")
    lines += ['', 'Median stage times (seconds; nested sub-timers are shown separately):', '',
              '| Mode | Reproduction | RL | Sampler setup | Sampling | Online IK | Anchor IK | Jacobian setup | FK mapping |',
              '|---|---:|---:|---:|---:|---:|---:|---:|---:|']
    for mode, values in summary.items():
        keys = ('reproduction_s', 'rl_deformation_s', 'setup_s', 'sampling_s', 'online_ik_s', 'anchor_ik_s', 'jacobian_setup_s', 'fk_mapping_s')
        lines.append('| '+mode+' | '+' | '.join(fmt(values.get(key+'_median')) for key in keys)+' |')
    lines += ['', 'Stage times overlap: draw/IK/FK/validity are inside sampling; anchor IK/Jacobian are inside setup. Do not add nested counters.',
              'Clearance is the whole robot to padded world geometry (ACM respected), sampled along joint interpolation; it is not a continuous safety certificate.',
              'The projected proposal is a local approximation. Valid targets and RRT tree vertices have different distributions.',
              'Seeds cover custom proposals/IK seeds and trial order. OMPL, the default sampler and the IK plugin may have independent RNGs.']
    (output / 'summary.md').write_text('\n'.join(lines)+'\n')


def main():
    share = Path(get_package_share_directory('tp_gmm'))
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cases', type=Path, default=share/'config/comparison_cases.json')
    parser.add_argument('--sampler-config', type=Path)
    parser.add_argument('--modes', nargs='+', choices=MODES, default=list(MODES))
    parser.add_argument('--repeats', type=int, default=3)
    parser.add_argument('--warmup', type=int, default=1, help='Paired warmups per case, excluded from summaries')
    parser.add_argument('--seed', type=int, default=42)
    parser.add_argument('--planning-time', type=float, default=3.0)
    parser.add_argument('--task', default='franka_pick_cube')
    parser.add_argument('--ref-robot', default='native')
    parser.add_argument('--clearance', type=float, default=0.5)
    parser.add_argument('--visualize', action='store_true', help='Enable RViz samples; introduces timing overhead')
    parser.add_argument('--allow-no-policy', action='store_true', help='Explicitly allow undeformed models for smoke tests')
    parser.add_argument('--output', type=Path, default=Path('sampling_results')/time.strftime('%Y%m%d-%H%M%S'))
    args, ros_args = parser.parse_known_args()
    if args.repeats < 1 or args.warmup < 0 or args.planning_time <= 0:
        parser.error('repeats and planning-time must be positive; warmup must be nonnegative')
    cases = json.loads(args.cases.read_text())
    rclpy.init(args=ros_args)
    runner = ComparisonRunner(args)
    rng, rows = random.Random(args.seed), []
    metadata = dict(task=args.task, seed=args.seed, planning_time=args.planning_time,
                    modes=args.modes, cases=cases, visualize=args.visualize,
                    settings=DEFAULTS | runner.sampler_options,
                    timing_note='one reproduction/deformation per pair, shared by all modes', models={})
    saved_scene = False
    try:
        runner.save_scene(); saved_scene = True
        for case in cases:
            runner.set_case_scene(case)
            start_pose, goal_pose = pose(case['start'], runner.frame_id), pose(case['goal'], runner.frame_id)
            obstacle_top = list(case['obstacle']); obstacle_top[2] += case['height']/2
            obstacle_pose = pose(obstacle_top, runner.frame_id, False)
            runner.obstacle_radius = case['radius']
            try:
                start, goal = runner.solve_state(start_pose), runner.solve_state(goal_pose)
            except RuntimeError as error:
                for repeat in range(args.repeats):
                    for mode in args.modes:
                        rows.append(dict(case=case['name'], repeat=repeat, mode=mode, seed=args.seed+repeat,
                                         warmup=False, success=False, failure_stage='endpoint_ik', error=str(error)))
                save_results(args.output, rows, metadata, args.modes)
                continue
            for repeat in range(-args.warmup, args.repeats):
                if not runner.call_deform_tpgmm_service(start_pose, goal_pose, obstacle_pose):
                    raise RuntimeError('TP-GMM service failed')
                if not runner.last_pipeline_metrics['policy_applied'] and not args.allow_no_policy:
                    raise RuntimeError('RL policy was not applied; load the checkpoint or explicitly use --allow-no-policy')
                model = message_to_ordereddict(runner.deformed_model)
                digest = hashlib.sha256(json.dumps(model, sort_keys=True).encode()).hexdigest()
                metadata['models'][digest] = model
                order = list(args.modes); rng.shuffle(order)
                seed = args.seed + (repeat + args.warmup)*101 + cases.index(case)*10007
                for mode in order:
                    row = dict(case=case['name'], repeat=repeat, warmup=repeat < 0, mode=mode,
                               seed=seed, model_sha256=digest, pipeline=deepcopy(runner.last_pipeline_metrics))
                    # Transport/configuration errors abort: continuing after a timeout could contaminate later trials.
                    row.update(runner.plan(start, goal, goal_pose, mode, seed, args.planning_time))
                    row['end_to_end_wall_s'] = (row['pipeline']['deform_service_wall_s'] +
                        row.get('preflight_wall_s', row.get('prepare_service_wall_s', 0)) + row.get('action_wall_s', 0))
                    rows.append(row)
                    save_results(args.output, rows, metadata, args.modes)
                    runner.get_logger().info(f"{case['name']} {repeat=} {mode}: success={row['success']} stage={row.get('failure_stage')}")
        from plot_sampling_comparison import plot
        plot(args.output/'results.json')
        print((args.output/'summary.md').read_text())
    finally:
        if saved_scene:
            runner.restore_scene()
        runner.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
