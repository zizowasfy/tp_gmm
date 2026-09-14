#!/usr/bin/env python3
"""Paired RL clearance generalization experiment. Plans only; never executes robot motion."""
import argparse
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import time
import numpy as np
import rclpy
from geometry_msgs.msg import PoseArray
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, RobotState, PlanningSceneComponents
from moveit_msgs.srv import GetPositionFK, GetStateValidity, GetPlanningScene
from rosidl_runtime_py.convert import message_to_ordereddict
from rosidl_runtime_py.set_message import set_message_fields
from tp_gmm.srv import DeformTPGMM
from compare_sampling_approaches import ComparisonRunner, pose, wait
from sampling_client import DEFAULTS
from clearance_analysis import (SAMPLERS, DEFAULT_ENVIRONMENT, clearances, validate_environment,
                                random_environment, validate_case, curve_metrics, policy_metrics, save)


class ClearanceRunner(ComparisonRunner):
    def __init__(self, args):
        super().__init__(args)
        self.fk_client = self.create_client(GetPositionFK, '/compute_fk')
        self.last_constraints = Constraints()

    def prepare_sampling(self, *args, **kwargs):
        prepared = super().prepare_sampling(*args, **kwargs)
        self.last_constraints = deepcopy(prepared.constraints)
        return prepared

    def deform(self, case, clearance, reference_kind):
        req = DeformTPGMM.Request(task_name=self.task_name, frame_id=self.frame_id)
        start, goal = pose(case['start'], self.frame_id), pose(case['goal'], self.frame_id)
        req.tpgmm_start_pose = self.adjust_orientation(start)
        req.tpgmm_goal_pose = self.adjust_orientation(goal)
        req.deformed_tpgmm_start_pose = req.tpgmm_start_pose
        req.deformed_tpgmm_goal_pose = req.tpgmm_goal_pose
        reference = np.array(case['obstacle'])
        if reference_kind == 'top':
            reference[2] += case['height']/2
        req.obstacle_pose = pose(reference, self.frame_id, downward=False)
        req.obstacle_radius = float(case['radius'])
        req.desired_clearance = float(clearance)
        self.start_pose_pub.publish(start)
        self.goal_pose_pub.publish(goal)
        self.obstacle_pose_pub.publish(req.obstacle_pose)
        begin = time.perf_counter()
        response = self.service(self.deform_tpgmm_client, req, timeout=120)
        elapsed = time.perf_counter()-begin
        if not response.policy_applied:
            raise RuntimeError('RL policy was not applied; load the retrained checkpoint before running this experiment')
        if not response.original_trajectory.poses or not response.deformed_trajectory.poses:
            raise RuntimeError('Missing request-matched trajectories; rebuild and restart the TP-GMM node')
        if response.original_trajectory.header.frame_id != self.frame_id or response.deformed_trajectory.header.frame_id != self.frame_id:
            raise RuntimeError('Returned trajectory frame differs from the experiment frame')
        self.original_model, self.deformed_model = response.original_gmm, response.deformed_gmm
        return response, reference, elapsed

    def uniform_plan(self, start, goal, planning_time):
        # Empty path constraints bypass the GMM allocator and MoveIt's constrained IK sampler.
        self.last_constraints = Constraints()
        request = MoveGroup.Goal()
        motion = request.request
        motion.group_name, motion.pipeline_id, motion.planner_id = self.group_name, 'ompl', 'RRTConnectkConfigDefault'
        motion.num_planning_attempts = 1
        motion.allowed_planning_time = float(planning_time)
        motion.max_velocity_scaling_factor = motion.max_acceleration_scaling_factor = 0.8
        motion.start_state = deepcopy(start)
        motion.start_state.is_diff = False
        motion.path_constraints = Constraints()
        goal_constraints = Constraints()
        for name, value in zip(goal.joint_state.name, goal.joint_state.position):
            if name.startswith('panda_joint'):
                goal_constraints.joint_constraints.append(JointConstraint(joint_name=name, position=value,
                    tolerance_above=0.001, tolerance_below=0.001, weight=1.))
        motion.goal_constraints.append(goal_constraints)
        request.planning_options.plan_only = True
        request.planning_options.replan = False
        begin = time.perf_counter()
        handle = wait(self, self.move_action_client.send_goal_async(request), 10)
        if not handle.accepted:
            raise RuntimeError('MoveGroup rejected unconstrained planning request')
        future = handle.get_result_async()
        try:
            result = wait(self, future, planning_time+30).result
        except TimeoutError:
            wait(self, handle.cancel_goal_async(), 5)
            wait(self, future, 15)
            raise
        ok = result.error_code.val == result.error_code.SUCCESS
        return dict(success=ok, planner_success=ok, error_code=result.error_code.val,
                    failure_stage=None if ok else 'planning', action_wall_s=time.perf_counter()-begin,
                    moveit_planning_s=result.planning_time, sampling={},
                    trajectory=message_to_ordereddict(result.planned_trajectory),
                    trajectory_start=message_to_ordereddict(result.trajectory_start))

    def measure_path(self, result, joint_step):
        trajectory = result['trajectory']['joint_trajectory']
        points = trajectory['points']
        if len(points) < 2:
            raise ValueError('Planner returned fewer than two trajectory points')
        q = np.array([p['positions'] for p in points])
        if not np.all(np.isfinite(q)) or q.shape[1] != len(trajectory['joint_names']):
            raise ValueError('Malformed returned joint trajectory')
        distance = np.linalg.norm(np.diff(q, axis=0), axis=1)
        counts = np.maximum(1, np.ceil(distance/joint_step).astype(int))
        if counts.sum() > 10000:
            raise ValueError('Returned path exceeds FK audit sample limit')
        joints = np.vstack([a+(b-a)*np.arange(n)[:, None]/n for a, b, n in zip(q[:-1], q[1:], counts)] + [q[-1:]])
        state = RobotState()
        set_message_fields(state, result['trajectory_start'])
        names = list(state.joint_state.name)
        indices = [names.index(n) for n in trajectory['joint_names']]
        xyz, invalid = [], 0
        for configuration in joints:
            positions = list(state.joint_state.position)
            for i, value in zip(indices, configuration):
                positions[i] = float(value)
            state.joint_state.position = positions
            state.is_diff = False
            fk_req = GetPositionFK.Request(robot_state=state, fk_link_names=[self.ee_link])
            fk_req.header.frame_id = self.frame_id
            fk = self.service(self.fk_client, fk_req)
            if fk.error_code.val != fk.error_code.SUCCESS or len(fk.pose_stamped) != 1:
                raise RuntimeError('FK analysis failed')
            p = fk.pose_stamped[0].pose.position
            xyz.append([p.x, p.y, p.z])
            valid = self.service(self.validity_client, GetStateValidity.Request(robot_state=state,
                                 group_name=self.group_name, constraints=self.last_constraints))
            invalid += not valid.valid
        return xyz, dict(audit_samples=len(joints), audit_invalid_samples=int(invalid),
                         joint_path_length_rad=float(distance.sum()))


def xyz(message: PoseArray):
    return [[p.position.x, p.position.y, p.position.z] for p in message.poses]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--trials', type=int, default=5, help='Number of random endpoint-valid environments')
    parser.add_argument('--samplers', '--sampler', nargs='+', choices=SAMPLERS, default=['cartesian_ik'])
    parser.add_argument('--include-zero', action='store_true', help='Eleven levels 0.0–1.0 instead of ten 0.1–1.0')
    parser.add_argument('--seed', type=int, default=42)
    parser.add_argument('--environment-config', type=Path, help='JSON sampling ranges (see config/clearance_environment.json)')
    parser.add_argument('--environments', type=Path, help='Replay the exact environments.json from a previous sweep')
    parser.add_argument('--max-environment-attempts', type=int, default=40)
    parser.add_argument('--planning-time', type=float, default=5.)
    parser.add_argument('--task', default='franka_pick_cube')
    parser.add_argument('--ref-robot', default='native')
    parser.add_argument('--sampler-config', type=Path)
    parser.add_argument('--visualize', action='store_true')
    parser.add_argument('--max-clearance-margin', type=float, default=0.30, help='Training normalization scale in metres')
    parser.add_argument('--base-buffer', type=float, default=0.08, help='Training reward base buffer in metres')
    parser.add_argument('--obstacle-reference', choices=['top', 'center'], default='top', help='Point sent to policy; top matches run_experiments.py')
    parser.add_argument('--joint-step', type=float, default=0.02, help='Maximum joint-vector norm between FK/validity audit states')
    parser.add_argument('--curve-step', type=float, default=0.002, help='Maximum polyline distance for obstacle analysis (metres)')
    parser.add_argument('--trim-fraction', type=float, default=0.1, help='Exclude each endpoint fraction for the additional interior metric')
    parser.add_argument('--output', type=Path, default=Path('clearance_results')/time.strftime('%Y%m%d-%H%M%S'))
    args, ros_args = parser.parse_known_args()
    if args.trials < 1 or args.max_environment_attempts < 1 or args.seed < 0 or args.seed >= 2**31:
        parser.error('Trial count/attempts must be positive and seed must be in [0, 2^31)')
    for value in (args.planning_time, args.joint_step, args.curve_step, args.max_clearance_margin):
        if not np.isfinite(value) or value <= 0:
            parser.error('Times, steps and margin scale must be finite and positive')
    if not np.isfinite(args.base_buffer) or args.base_buffer < 0 or not 0 <= args.trim_fraction < 0.5:
        parser.error('Invalid base buffer or trim fraction')
    if len(set(args.samplers)) != len(args.samplers):
        parser.error('Do not repeat a sampler')
    if (args.output/'results.json').exists():
        parser.error('Output already contains a run; use a new directory')
    config = DEFAULT_ENVIRONMENT | (json.loads(args.environment_config.read_text()) if args.environment_config else {})
    validate_environment(config)
    replay = json.loads(args.environments.read_text()) if args.environments else None
    if replay is not None and len(replay) < args.trials:
        parser.error('Replay file has fewer environments than requested trials')
    # Separate design/order RNGs: changing sampler count cannot change the environments.
    design_rng = np.random.default_rng(args.seed)
    order_rng = np.random.default_rng(args.seed+1)
    args.clearance = 0.1  # Parent runner initialization; each service request sets its own value.
    rclpy.init(args=ros_args)
    runner = ClearanceRunner(args)
    rows, environments = [], []
    metadata = dict(schema_version=1, status='running', args={k:str(v) if isinstance(v, Path) else v for k,v in vars(args).items()},
                    settings=DEFAULTS | runner.sampler_options, environment_ranges=config, environments=environments,
                    rejected_environments=[], endpoint_states=[], clearance_values=clearances(args.include_zero),
                    seed_note='Seed controls environment/order/custom sampler draws. OMPL and IK may use independent RNGs.',
                    sampler_note='ompl_uniform has no GMM/path constraints and uses joint-space bounds; it is not uniform in Cartesian volume',
                    distance_note='Signed EE-point/DSGMR-polyline to finite cylinder surface; not whole-robot clearance. FK path audit is sampled.',
                    target_note='Training mean metric excludes endpoint components: 3-D reference distance minus radius >= base_buffer + clearance*max_margin')
    saved_scene = False
    try:
        runner.save_scene(); saved_scene = True
        scene_request = GetPlanningScene.Request()
        scene_request.components.components = PlanningSceneComponents.WORLD_OBJECT_GEOMETRY | PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
        scene = runner.service(runner.scene_client, scene_request)
        metadata['initial_scene'] = message_to_ordereddict(scene.scene)
        for trial in range(args.trials):
            start = goal = None
            for attempt in range(1 if replay is not None else args.max_environment_attempts):
                case = deepcopy(replay[trial]) if replay is not None else random_environment(design_rng, config, trial)
                validate_case(case)
                runner.set_case_scene(case)
                try:
                    if np.linalg.norm(np.array(case['goal'])-case['start']) < config['minimum_endpoint_separation']:
                        raise ValueError('Endpoints too close')
                    start = runner.solve_state(pose(case['start'], runner.frame_id))
                    goal = runner.solve_state(pose(case['goal'], runner.frame_id))
                except (RuntimeError, ValueError) as error:
                    if isinstance(error, RuntimeError) and not str(error).startswith('Benchmark endpoint IK failed'):
                        raise
                    metadata['rejected_environments'].append(dict(trial=trial, attempt=attempt, environment=case, reason=str(error)))
                    save(args.output, metadata, rows)
                    start = goal = None
                    continue
                break
            if start is None or goal is None:
                raise RuntimeError(f'No endpoint-valid environment for trial {trial}; see rejected_environments')
            environments.append(case)
            metadata['endpoint_states'].append(dict(trial=trial, start=message_to_ordereddict(start), goal=message_to_ordereddict(goal)))
            (args.output).mkdir(parents=True, exist_ok=True)
            (args.output/'environments.json').write_text(json.dumps(environments, indent=2))
            # Environment/IK states remain fixed across every clearance and sampler in this trial.
            prior_fingerprint = None
            for c in order_rng.permutation(metadata['clearance_values']):
                c = float(c)
                response, reference, service_wall = runner.deform(case, c, args.obstacle_reference)
                models = {name:message_to_ordereddict(getattr(response, name+'_gmm')) for name in ('original', 'deformed')}
                current_prior = hashlib.sha256(json.dumps({k: models['original'][k] for k in ('weights','gaussians')}, sort_keys=True).encode()).hexdigest()
                if prior_fingerprint is not None and current_prior != prior_fingerprint:
                    raise RuntimeError('Prior GMM changed within the fixed-environment sweep')
                prior_fingerprint = current_prior
                if metadata.get('policy_sha256', response.policy_sha256) != response.policy_sha256:
                    raise RuntimeError('Policy changed during the experiment')
                metadata['policy_sha256'] = response.policy_sha256
                digest = hashlib.sha256(json.dumps(models, sort_keys=True).encode()).hexdigest()
                curves = {name:curve_metrics(xyz(getattr(response, name+'_trajectory')), case, args.curve_step, args.trim_fraction)
                          for name in ('original', 'deformed')}
                policy = policy_metrics(models['original'], models['deformed'], reference, case['radius'], args.base_buffer+c*args.max_clearance_margin)
                pipeline = {name:getattr(response,name) for name in ('model_load_s','reproduction_s','rl_deformation_s','deformed_regression_s','total_s','policy_applied')}
                # Save each exact model and policy provenance once, shared by samplers.
                model_dir = args.output/'models'; model_dir.mkdir(exist_ok=True)
                (model_dir/(digest+'.json')).write_text(json.dumps(models, indent=2))
                for mode in order_rng.permutation(args.samplers):
                    mode = str(mode)
                    seed = int((args.seed+trial*10007) % 2**32)
                    row = dict(trial=trial, clearance=c, sampler=mode, seed=seed, environment=case, model_sha256=digest,
                               policy_checkpoint=response.policy_checkpoint, policy_sha256=response.policy_sha256,
                               action_scale=response.action_scale, obstacle_reference=reference.tolist(),
                               target_margin_m=policy['target_margin_m'], curves=deepcopy(curves), policy=policy, pipeline=pipeline,
                               policy_violation_m=policy['training_clearance_violation_sum_m'],
                               policy_clearance_satisfied=policy['training_clearance_satisfied'],
                               deformed_interior_min_clearance_m=curves['deformed']['interior_min_clearance_m'],
                               original_min_clearance_m=curves['original']['min_clearance_m'],
                               deformed_min_clearance_m=curves['deformed']['min_clearance_m'],
                               deformation_gain_m=curves['deformed']['min_clearance_m']-curves['original']['min_clearance_m'],
                               deform_service_wall_s=service_wall)
                    result = (runner.uniform_plan(start, goal, args.planning_time) if mode == 'ompl_uniform' else
                              runner.plan(start, goal, pose(case['goal'],runner.frame_id), mode, seed, args.planning_time))
                    row.update(result)
                    if result.get('planner_success'):
                        analysis_start = time.perf_counter()
                        path, audit = runner.measure_path(result, args.joint_step)
                        row.update(audit)
                        row['analysis_wall_s'] = time.perf_counter()-analysis_start
                        row['path_audit_passed'] = row.get('path_audit_passed', True) and audit['audit_invalid_samples'] == 0
                        if audit['audit_invalid_samples']:
                            row['success'], row['failure_stage'] = False, 'path_audit'
                        # Failed audits are kept as diagnostics, excluded from successful-path summaries.
                        curve = curve_metrics(path, case, args.curve_step, args.trim_fraction)
                        row['curves']['planned'] = curve
                        if row['success']:
                            row['planned_min_clearance_m'] = curve['min_clearance_m']
                            row['planned_path_length_m'] = curve['path_length_m']
                            row['planned_interior_min_clearance_m'] = curve['interior_min_clearance_m']
                    row['end_to_end_wall_s'] = service_wall+row.get('preflight_wall_s',row.get('prepare_service_wall_s',0))+row.get('action_wall_s',0)
                    attempts = row.get('sampling', {}).get('attempts', 0)
                    row['sampling_efficiency'] = row.get('sampling', {}).get('valid_samples', 0)/attempts if attempts else None
                    for key, value in pipeline.items():
                        row[key] = value
                    for key, value in row.get('sampling', {}).items():
                        if isinstance(value, (int,float)):
                            row['sampling_'+key] = value
                    rows.append(row)
                    save(args.output, metadata, rows)
                    runner.get_logger().info(f"trial={trial} clearance={c:.1f} sampler={mode} success={row['success']} stage={row.get('failure_stage')}")
        metadata['status'] = 'complete'
    except BaseException as error:
        metadata['status'], metadata['error'] = 'interrupted' if isinstance(error, KeyboardInterrupt) else 'aborted', str(error)
        raise
    finally:
        save(args.output, metadata, rows)
        try:
            if saved_scene:
                runner.restore_scene()
        finally:
            runner.destroy_node(); rclpy.shutdown()
        from plot_clearance_analysis import plot
        plot(args.output/'results.json')
    print(f"Saved {len(rows)} plans and clearance figures in {args.output}")


if __name__ == '__main__':
    main()
