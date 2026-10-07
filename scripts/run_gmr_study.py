#!/usr/bin/env python3
"""Frozen, resumable plan-only GMM/GMR-30-mm/hybrid/reference-IK study."""
import argparse
from copy import deepcopy
import hashlib
import json
import os
from pathlib import Path
import platform
import shutil
import subprocess
import time

import numpy as np
import rclpy
from moveit_msgs.msg import PlanningScene, RobotState, RobotTrajectory
from moveit_msgs.srv import GetStateValidity
from tp_gmm.srv import AuditTrajectory, DeformTPGMM
from tp_gmm_sampling.reference_baseline import ReferenceIKBaseline
from run_sampling_study import StudyRunner, pure_settings, save, digest, message, asdict
from replay_sampling_study import restore_frozen_scene, scene_payload, model_payload
from run_clearance_sweep import xyz
from compare_sampling_approaches import pose
from clearance_analysis import DEFAULT_ENVIRONMENT, random_environment, curve_metrics

MODES = ('gmm', 'gmr_30mm', 'hybrid_30mm', 'reference_ik')
PROPOSALS = dict(gmm='gmm', gmr_30mm='gmr_path', hybrid_30mm='hybrid', reference_ik='gmm')
POLICY_SHA256 = 'd9ec51ff464b93f8ad87e4d5dc2b3bb649f89dbb5fa4155b5df46ed788d2e5ba'
SETTINGS = pure_settings(3.) | dict(gmr_stddev=.03, gmr_cutoff=3., gmr_fraction=.8)


def settings(mode):
    return SETTINGS | dict(proposal=PROPOSALS[mode])


def assert_proposal_pure(mode, stats):
    expected = settings(mode)
    if stats.get('mode') != 'cartesian_ik' or any(k not in stats for k in ('attempts','gmm_attempts','gmr_attempts','uniform_attempts')):
        raise RuntimeError('Missing proposal telemetry or incorrect mapping')
    for key in ('proposal', 'corridor_mode', 'gmr_stddev', 'gmr_cutoff', 'gmr_fraction',
                'uniform_fraction', 'cartesian_fraction'):
        if stats.get(key) != expected[key]:
            raise RuntimeError(f'Proposal configuration mismatch: {mode}/{key}: {stats.get(key)}')
    if not stats.get('component_cutoffs') or any(x != 3. for x in stats['component_cutoffs']):
        raise RuntimeError('GMM covariance cutoff differs from 3.0')
    for key in ('uniform_attempts', 'missing_anchor_fallbacks', 'projected_attempts'):
        if stats.get(key, 0) != 0:
            raise RuntimeError(f'Forbidden sampling source: {mode}/{key}')
    if mode in ('gmm', 'reference_ik') and stats.get('gmr_attempts', 0):
        raise RuntimeError('GMR attempted by GMM/reference baseline')
    if mode == 'gmr_30mm' and stats.get('gmm_attempts', 0):
        raise RuntimeError('GMM fallback attempted by pure GMR')
    if mode == 'reference_ik' and stats.get('attempts', 0):
        raise RuntimeError('Reference IK unexpectedly invoked a random sampler')
    if stats.get('attempts', 0) != stats.get('gmm_attempts', 0) + stats.get('gmr_attempts', 0):
        raise RuntimeError('Unaccounted sampling attempts')


def trial_key(row):
    return row['environment'], row['repeat'], row['mode']


def append_row(path, row):
    with path.open('a') as output:
        output.write(json.dumps(row, allow_nan=False)+'\n')
        output.flush()
        os.fsync(output.fileno())


class GMRStudyRunner(StudyRunner):
    def __init__(self, args):
        super().__init__(args)
        self.baseline = ReferenceIKBaseline(self)
        self.variant = 'gmm'

    def validate_sampling(self, mode, stats):
        assert_proposal_pure(self.variant, stats)

    def compare_plan(self, start, goal, case, mode, seed, budget, scene):
        self.variant = mode
        self.sampler_options = settings(mode)
        if mode != 'reference_ik':
            return self.study_plan(start, goal, case, 'cartesian_ik', seed, budget, scene)
        begin = time.perf_counter()
        prepared = self.prepare_sampling(pose(case['goal'], self.frame_id), mode='cartesian_ik', seed=seed)
        row = dict(success=False, planner_success=False, sampling={}, action_wall_s=0.,
                   prepare_service_wall_s=time.perf_counter()-begin,
                   path_constraints_sha256=digest(asdict(prepared.constraints)))
        try:
            # Apply identical endpoint preflight to every method.
            for label, state in [('start', start), ('goal', goal)]:
                valid = self.service(self.validity_client, GetStateValidity.Request(
                    robot_state=state, group_name=self.group_name, constraints=prepared.constraints))
                if not valid.valid:
                    row['failure_stage'] = label+'_invalid_in_corridor'
                    return row
            row['preflight_wall_s'] = time.perf_counter()-begin
            result = self.baseline.plan(self.deformed_reference, start, goal,
                [n for n in goal.joint_state.name if n.startswith('panda_joint')], self.group_name,
                self.ee_link, pose(case['goal'], self.frame_id).pose.orientation, prepared.constraints, budget)
            row.update({k:v for k,v in result.items() if k not in ('sampling', 'ee_points')})
            row['reference_baseline_metrics'] = result.get('sampling', {})
            row['planning_pipeline_wall_s'] = time.perf_counter()-begin
            if row['success']:
                audit_begin = time.perf_counter()
                audited = self.service(self.audit_client, AuditTrajectory.Request(scene=scene,
                    trajectory_start=message(RobotState,row['trajectory_start']),
                    trajectory=message(RobotTrajectory,row['trajectory']), constraints=prepared.constraints,
                    group_name=self.group_name, link_name=self.ee_link, joint_step=.01), timeout=60)
                if not audited.success:
                    raise RuntimeError('Reference audit failed: '+audited.message)
                row['audit'] = json.loads(audited.json)
                row['audit_wall_s'] = time.perf_counter()-audit_begin
                row['success'] = row['audit']['audit_invalid_samples'] == 0
                row['path_audit_passed'] = row['success']
                row['failure_stage'] = None if row['success'] else 'path_audit'
                row['curve'] = {k:v for k,v in curve_metrics(row['audit']['ee_path'],case,.002,.1).items() if not isinstance(v,list)}
            return row
        finally:
            row.setdefault('planning_pipeline_wall_s', time.perf_counter()-begin)
            row['sampling'] = self.sampling.report(prepared.request_id, release=True)
            assert_proposal_pure(mode, row['sampling'])


def runtime_fingerprint(root):
    """Hash sources and loaded artefacts; resume refuses algorithm/model changes."""
    workspace = root.parents[1]
    paths = [p for folder in ('src','include','scripts','python','srv','launch','config','tests')
             for p in (root/folder).rglob('*') if p.is_file() and '__pycache__' not in str(p)]
    paths += [root/'tasks/franka_pick_cube/TPGMM_model.pkl',root/'GMR_STUDY.md',root/'CMakeLists.txt']
    for name in ('tp_gmm/lib/libgmm_constraint_sampler.so', 'tp_gmm/lib/tp_gmm/sampling_auditor',
                 'moveit_planners_ompl/lib/libmoveit_ompl_interface.so',
                 'moveit_planners_ompl/lib/libmoveit_ompl_planner_plugin.so'):
        paths.append(workspace/'install'/name)
    return {str(p.relative_to(workspace)): hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--environments-per-layout', type=int, default=40)
    parser.add_argument('--repeats', type=int, default=5)
    parser.add_argument('--legacy-repeats', type=int, default=10)
    parser.add_argument('--planning-time', type=float, default=3.)
    parser.add_argument('--seed', type=int, default=20261005)
    parser.add_argument('--resume', action='store_true')
    parser.add_argument('--design-only', action='store_true')
    args, ros_args = parser.parse_known_args()
    if min(args.environments_per_layout,args.repeats,args.legacy_repeats) < 1 or not np.isfinite(args.planning_time) or not 0 < args.planning_time <= 60 or not 0 <= args.seed < 2**30:
        parser.error('Invalid design size, budget or seed')
    args.task,args.ref_robot,args.clearance,args.visualize,args.sampler_config,args.cutoff = 'franka_pick_cube','native',.7,False,None,3.
    args.output = args.output.resolve()
    args.output.mkdir(parents=True, exist_ok=True)
    # Source snapshots are evidence, not additional workspace packages.
    (args.output/'COLCON_IGNORE').touch(exist_ok=True)
    manifest_path = args.output/'manifest.json'
    if manifest_path.exists() and not args.resume:
        parser.error('Output already contains a study; use --resume or a new directory')
    root = Path(__file__).resolve().parents[1]
    fingerprint = runtime_fingerprint(root)
    design = {k:getattr(args,k) for k in ('environments_per_layout','repeats','legacy_repeats','planning_time','seed')}
    rclpy.init(args=ros_args)
    runner = GMRStudyRunner(args)
    saved_scene = False
    try:
        runtime = runner.check_configuration()
        segment = runtime['ompl.panda_arm.longest_valid_segment_fraction']['double_value']
        simplify = runtime['ompl.planner_configs.RRTConnectkConfigDefault.simplify_solutions']
        if segment != .0005 or simplify['type'] != 2 or simplify['integer_value'] != 0:
            raise RuntimeError(f'Unexpected edge checking or simplification: {segment}/{simplify}')
        runner.save_scene(); saved_scene = True
        if manifest_path.exists():
            manifest = json.loads(manifest_path.read_text())
            if any(manifest[k] != v for k,v in dict(design=design, settings=SETTINGS, runtime=runtime, source_sha256=fingerprint).items()):
                raise RuntimeError('Frozen settings, runtime or source changed; refusing to resume')
        else:
            rng = np.random.default_rng(args.seed)
            candidates = [dict(cohort='legacy',layout=c['name'],case=c) for c in json.loads((root/'config/comparison_cases.json').read_text())]
            for layout, offset in [('central',0.),('offset',.07),('distant',.18)]:
                for i in range(args.environments_per_layout):
                    case = random_environment(rng,DEFAULT_ENVIRONMENT,len(candidates))
                    case['obstacle'][1] += float(rng.choice([-1,1]))*offset
                    case['name'] = f'{layout}_{i:03d}'
                    candidates.append(dict(cohort='randomized',layout=layout,case=case))
            manifest = dict(schema=1,status='designing',design=design,settings=SETTINGS,clearance=.7,
                modes=list(MODES),runtime=runtime,source_sha256=fingerprint,candidates=candidates,environments=[],
                environment_ranges=DEFAULT_ENVIRONMENT,created_utc=time.strftime('%Y-%m-%dT%H:%M:%SZ',time.gmtime()),
                host=dict(platform=platform.platform(),cpus=os.cpu_count()),
                analysis=dict(independent_unit='environment',bootstrap=20000,permutations=100000,alpha=.05,
                    primary_family='All six paired method contrasts in success and PAR2; Holm correction across 12 tests',
                    quality_family='Three RRT proposal pairs, matched-success joint/EE length, world clearance and reference deviation; Holm across 12 tests',
                    legacy='Three original scenes are descriptive and never pooled with randomized environments',
                    stopping='Fixed design, no outcome-based exclusions or extensions; endpoint failures retained for all methods',
                    reference_note='Sequential reference IK includes public IK/FK/validity service costs within budget; geometric path, no time parameterization',
                    seed_note='Environment/order/custom proposal RNG seeded; OMPL and IK have independent RNG streams, paired by environment and repeat'))
            for repo in ('tp_gmm','moveit2','moveit_resources'):
                folder = root.parent/repo
                manifest.setdefault('git',{})[repo] = dict(
                    head=subprocess.check_output(['git','rev-parse','HEAD'],cwd=folder,text=True).strip(),
                    branch=subprocess.check_output(['git','branch','--show-current'],cwd=folder,text=True).strip())
            save(manifest_path,manifest)
            # Keep the exact source that produced the results, including uncommitted code.
            for relative in fingerprint:
                source = root.parents[1]/relative
                if relative.startswith('src/'):
                    destination = args.output/'provenance'/relative
                    destination.parent.mkdir(parents=True,exist_ok=True)
                    shutil.copy2(source,destination)
            for repo in ('tp_gmm','moveit2','moveit_resources'):
                (args.output/'provenance'/f'{repo}.patch').write_bytes(subprocess.check_output(['git','diff','HEAD'],cwd=root.parent/repo))
        # Candidate list was fixed before any endpoint feasibility or deformation outcome.
        for index in range(len(manifest['environments']),len(manifest['candidates'])):
            entry = manifest['candidates'][index]
            case = entry['case']
            runner.set_case_scene(case)
            start = goal = None
            endpoint_error = None
            try:
                start = runner.solve_state(pose(case['start'],runner.frame_id))
                goal = runner.solve_state(pose(case['goal'],runner.frame_id))
            except RuntimeError as error:
                if not str(error).startswith('Benchmark endpoint IK failed'):
                    raise
                endpoint_error = str(error)
            response,_,elapsed = runner.deform(case,.7,'top')
            if response.policy_sha256 != POLICY_SHA256 or response.action_scale != .15:
                raise RuntimeError('Policy differs from the original pilot checkpoint/action scale')
            scene = runner.scene_snapshot()
            verified = restore_frozen_scene(runner,scene)
            content = dict(case=case,start=asdict(start) if start else None,goal=asdict(goal) if goal else None,
                endpoint_error=endpoint_error,scene=asdict(scene),scene_verification=verified,response=asdict(response),
                pipeline={k:getattr(response,k) for k in ('model_load_s','reproduction_s','rl_deformation_s','deformed_regression_s','total_s')},
                deformation_wall_s=elapsed,
                reference_metrics={k:v for k,v in curve_metrics(xyz(response.deformed_trajectory),case).items() if not isinstance(v,list)})
            relative = f'environments/{index:03d}.json'
            save(args.output/relative,content)
            manifest['environments'].append(dict(id=index,cohort=entry['cohort'],layout=entry['layout'],case=case,
                file=relative,sha256=digest(content),model_sha256=digest(model_payload(asdict(response.deformed_gmm))),
                reference_sha256=digest(model_payload(asdict(response.deformed_trajectory)))))
            manifest['policy_sha256'] = response.policy_sha256
            save(manifest_path,manifest)
            print(f'DESIGN {index+1}/{len(manifest["candidates"])} {case["name"]} endpoint_error={endpoint_error}',flush=True)
        if 'schedule' not in manifest:
            rng = np.random.default_rng(args.seed+1)
            schedule=[]
            for env in manifest['environments']:
                if env['cohort']=='legacy':
                    for mode in rng.permutation(MODES):
                        schedule.append(dict(environment=env['id'],repeat=-1,mode=str(mode),warmup=True,
                            seed=int(args.seed+env['id']*10007)))
            for index in rng.permutation(len(manifest['environments'])):
                env=manifest['environments'][int(index)]
                repeats=args.legacy_repeats if env['cohort']=='legacy' else args.repeats
                for repeat in range(repeats):
                    for mode in rng.permutation(MODES):
                        schedule.append(dict(environment=int(index),repeat=repeat,mode=str(mode),warmup=repeat<0,
                            seed=int(args.seed+int(index)*10007+(repeat+1)*101)))
            manifest.update(status='frozen',schedule=schedule)
            save(manifest_path,manifest)
        if args.design_only:
            return
        rows_path=args.output/'results.jsonl'
        previous=[json.loads(line) for line in rows_path.read_text().splitlines()] if rows_path.exists() else []
        completed={trial_key(r) for r in previous}
        expected={trial_key(r) for r in manifest['schedule']}
        if len(completed)!=len(previous) or not completed<=expected:
            raise RuntimeError('Duplicate or unexpected trial keys')
        for row in previous:
            env=manifest['environments'][row['environment']]
            if row['environment_sha256']!=env['sha256']:
                raise RuntimeError('Result refers to different frozen environment')
            if row.get('sampling'):
                assert_proposal_pure(row['mode'],row['sampling'])
        active=None
        for trial in manifest['schedule']:
            if trial_key(trial) in completed:
                continue
            if active != trial['environment']:
                active=trial['environment']; env=manifest['environments'][active]
                content=json.loads((args.output/env['file']).read_text())
                if digest(content)!=env['sha256']:
                    raise RuntimeError('Frozen environment file changed')
                scene=message(PlanningScene,content['scene'])
                verified=restore_frozen_scene(runner,scene)
                response=message(DeformTPGMM.Response,content['response'])
                runner.original_model,runner.deformed_model=response.original_gmm,response.deformed_gmm
                runner.original_reference,runner.deformed_reference=response.original_trajectory,response.deformed_trajectory
                if not content['endpoint_error']:
                    start,goal=message(RobotState,content['start']),message(RobotState,content['goal'])
            if content['endpoint_error']:
                result=dict(success=False,planner_success=False,planning_not_attempted=True,failure_stage='endpoint_ik',
                            action_wall_s=0.,planning_pipeline_wall_s=0.,sampling={})
            else:
                result=runner.compare_plan(start,goal,content['case'],trial['mode'],trial['seed'],args.planning_time,scene)
            row=trial | dict(cohort=env['cohort'],layout=env['layout'],clearance=.7,environment_sha256=env['sha256'],
                scene_verification=verified,model_sha256=env['model_sha256'],reference_sha256=env['reference_sha256'],
                settings=settings(trial['mode']),pipeline=content['pipeline'],reference_metrics=content['reference_metrics']) | result
            row['end_to_end_wall_s']=row['planning_pipeline_wall_s']+content['deformation_wall_s']
            row['par2_s']=min(row['action_wall_s'],args.planning_time) if row['success'] else 2*args.planning_time
            append_row(rows_path,row)
            completed.add(trial_key(row))
            print(f'PLAN {len(completed)}/{len(expected)} env={active} repeat={trial["repeat"]} {trial["mode"]} success={row["success"]} stage={row.get("failure_stage")} wall={row["action_wall_s"]:.3f}',flush=True)
        manifest.update(status='complete',completed_utc=time.strftime('%Y-%m-%dT%H:%M:%SZ',time.gmtime()))
        save(manifest_path,manifest)
    finally:
        if saved_scene:
            runner.restore_scene()
        runner.destroy_node()
        rclpy.shutdown()


if __name__=='__main__':
    main()
