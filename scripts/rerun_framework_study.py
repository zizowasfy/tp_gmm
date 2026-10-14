#!/usr/bin/env python3
"""Rerun a frozen mapping/GMR study under the current benchmark-aligned protocol.

Scenes, joint endpoints, cohort membership, ordering and proposal seeds are retained.
Deformation timings are remeasured once per environment and models checked against
saved inputs. A protocol/source mismatch fails closed; historical results are read-only.
"""
import argparse
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import shutil
import time

import rclpy
from moveit_msgs.msg import PlanningScene, RobotState
from tp_gmm.srv import DeformTPGMM
from run_sampling_study import StudyRunner, pure_settings, save, digest, message, asdict, MODES as MAPPINGS
from run_gmr_study import (GMRStudyRunner, MODES as PROPOSALS, SETTINGS, POLICY_SHA256,
                           settings, runtime_fingerprint, append_row, trial_key)
from replay_sampling_study import restore_frozen_scene, model_payload


def utc():
    return time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime())


def reference_payload(reference):
    return dict(frame=reference['header']['frame_id'], poses=reference['poses'])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--study', choices=['mapping', 'gmr'], required=True)
    parser.add_argument('--source', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--resume', action='store_true')
    parser.add_argument('--design-only', action='store_true')
    args, ros_args = parser.parse_known_args()
    args.source, args.output = args.source.resolve(), args.output.resolve()
    if args.source == args.output:
        parser.error('Use a new output directory')
    source_path = args.source/'manifest.json'
    source = json.loads(source_path.read_text())
    if source['status'] != 'complete' or source['policy_sha256'] != POLICY_SHA256:
        raise ValueError('Requires completed source study and the benchmark RL checkpoint')
    modes = PROPOSALS if args.study == 'gmr' else MAPPINGS
    expected = {(e['id'], j, m) for e in source['environments']
                for j in range(source['design']['legacy_repeats'] if e.get('cohort') == 'legacy' else source['design']['repeats'])
                for m in modes}
    schedule = [t for t in source['schedule'] if t['mode'] in modes]
    measured = [t for t in schedule if not t.get('warmup')]
    if {trial_key(t) for t in measured} != expected or len(measured) != len(expected):
        raise ValueError('Source schedule is incomplete or duplicated')
    args.task, args.ref_robot, args.clearance, args.visualize = 'franka_pick_cube', 'native', .7, False
    args.sampler_config, args.cutoff = None, 3.
    root = Path(__file__).resolve().parents[1]
    fingerprint = runtime_fingerprint(root)
    identity = dict(source_manifest_sha256=hashlib.sha256(source_path.read_bytes()).hexdigest(),
                    framework='solar_ellipsoids_tube_range_auto_v1', study=args.study)
    args.output.mkdir(parents=True, exist_ok=True)
    (args.output/'COLCON_IGNORE').touch()
    manifest_path = args.output/'manifest.json'
    if manifest_path.exists() and not args.resume:
        parser.error('Output exists; use --resume or a new directory')
    rclpy.init(args=ros_args)
    runner = (GMRStudyRunner if args.study == 'gmr' else StudyRunner)(args)
    saved_scene = False
    try:
        runtime = runner.check_configuration()
        runner.save_scene(); saved_scene = True
        if manifest_path.exists():
            manifest = json.loads(manifest_path.read_text())
            if manifest['framework_replay'] != identity or manifest['runtime'] != runtime or manifest['source_sha256'] != fingerprint:
                raise RuntimeError('Source, binary, protocol or runtime changed; cannot resume')
        else:
            manifest = deepcopy(source)
            for k in ('completed_utc', 'replay', 'git'):
                manifest.pop(k, None)
            manifest.update(status='designing', created_utc=utc(), source_directory=str(args.source),
                framework_replay=identity, runtime=runtime, source_sha256=fingerprint, environments=[],
                modes=list(modes), settings=SETTINGS if args.study == 'gmr' else pure_settings(3.), schedule=schedule)
            manifest['design']['planning_time'] = 10.
            manifest['protocol'] = dict(gmm_geometry='ellipsoid_union', gmm_cutoff=3., corridor_tolerance_m=0.,
                gmr_stddev_m=.03, gmr_cutoff=3., gmr_tube_radius_m=.09, gmr_endpoint_connections='actual_fk_start_goal',
                planner='RRTConnectkConfigDefault', requested_range=0., longest_valid_segment_fraction=.00005,
                audit_step_rad=.005, response_retiming=False, default_uniform_fallback=False,
                ompl_rng='independent unseeded stream, paired environments; custom proposal seeds retained',
                deformation='remeasured once per environment, attributed to each method; excluded for unrestricted OMPL')
            manifest['analysis']['framework_note'] = 'Fresh results on saved environments; do not pool with earlier box-corridor results.'
            manifest['analysis']['replay_note'] = 'Framework-alignment rerun on saved environments; clearance and covariance cutoff are retained. This is not new independent confirmation.'
            if args.study == 'gmr':
                manifest['analysis']['primary_family'] = 'Success and PAR2, three pairwise contrasts, Holm six tests'
                manifest['analysis']['quality_family'] = 'One GMM/GMR pair, matched-success joint/EE length, world clearance and reference deviation; Holm across four tests'
            save(manifest_path, manifest)
            for relative in fingerprint:
                original = root.parents[1]/relative
                target = args.output/'provenance'/relative
                target.parent.mkdir(parents=True, exist_ok=True)
                shutil.copy2(original, target)
            shutil.copy2(source_path, args.output/'provenance/source_manifest.json')
        # Restarting an interrupted design resumes only completed, immutable inputs.
        for env in source['environments'][len(manifest['environments']):]:
            content = json.loads((args.source/env['file']).read_text())
            if digest(content) != env['sha256']:
                raise ValueError('Source frozen input changed')
            scene = message(PlanningScene, content['scene'])
            restore_frozen_scene(runner, scene)
            clearance = .7 if args.study == 'gmr' else env['clearance']
            response, _, wall = runner.deform(content['case'], clearance, 'top')
            fresh = asdict(response)
            if response.policy_sha256 != POLICY_SHA256 or response.action_scale != content['response']['action_scale']:
                raise RuntimeError('RL checkpoint/action scale mismatch')
            for name in ('original_gmm', 'deformed_gmm'):
                if model_payload(fresh[name]) != model_payload(content['response'][name]):
                    raise RuntimeError('Frozen model changed: '+name)
            for name in ('original_trajectory', 'deformed_trajectory'):
                if reference_payload(fresh[name]) != reference_payload(content['response'][name]):
                    raise RuntimeError('Frozen reference changed: '+name)
            content.update(response=fresh, deformation_wall_s=wall,
                pipeline={k:getattr(response, k) for k in ('model_load_s','reproduction_s','rl_deformation_s','deformed_regression_s','total_s')})
            copied = deepcopy(env)
            copied.update(sha256=digest(content), source_sha256=env['sha256'])
            save(args.output/copied['file'], content)
            manifest['environments'].append(copied)
            save(manifest_path, manifest)
            print(f'DESIGN {len(manifest["environments"])}/{len(source["environments"])}', flush=True)
        manifest['status'] = 'frozen'; save(manifest_path, manifest)
        if args.design_only:
            return
        rows_path = args.output/'results.jsonl'
        old_rows = [json.loads(x) for x in rows_path.read_text().splitlines()] if rows_path.exists() else []
        completed = {trial_key(r) for r in old_rows}
        if len(completed) != len(old_rows) or not completed <= {trial_key(t) for t in schedule}:
            raise ValueError('Duplicate or unexpected result rows')
        for row in old_rows:
            if row['environment_sha256'] != manifest['environments'][row['environment']]['sha256']:
                raise ValueError('Result/frozen input mismatch')
        active = None
        for trial in schedule:
            if trial_key(trial) in completed:
                continue
            if active != trial['environment']:
                if runtime_fingerprint(root) != fingerprint:
                    raise RuntimeError('Runtime changed during measured experiment')
                active = trial['environment']; env = manifest['environments'][active]
                content = json.loads((args.output/env['file']).read_text())
                if digest(content) != env['sha256']:
                    raise ValueError('Frozen replay input changed')
                scene = message(PlanningScene, content['scene'])
                verified = restore_frozen_scene(runner, scene)
                response = message(DeformTPGMM.Response, content['response'])
                runner.original_model, runner.deformed_model = response.original_gmm, response.deformed_gmm
                runner.original_reference, runner.deformed_reference = response.original_trajectory, response.deformed_trajectory
                if not content.get('endpoint_error'):
                    start, goal = message(RobotState, content['start']), message(RobotState, content['goal'])
            if content.get('endpoint_error'):
                result = dict(success=False, planner_success=False, planning_not_attempted=True, failure_stage='endpoint_ik',
                              action_wall_s=0., planning_pipeline_wall_s=0., sampling={})
            elif args.study == 'gmr':
                result = runner.compare_plan(start, goal, content['case'], trial['mode'], trial['seed'], 10., scene)
            else:
                result = runner.study_plan(start, goal, content['case'], trial['mode'], trial['seed'], 10., scene)
            row = trial | dict(environment_sha256=env['sha256'], scene_verification=verified, pipeline=content['pipeline']) | result
            if args.study == 'gmr':
                row.update(cohort=env['cohort'], layout=env['layout'], clearance=.7,
                    model_sha256=env['model_sha256'], reference_sha256=env['reference_sha256'],
                    settings=settings(trial['mode']), reference_metrics=content['reference_metrics'])
            else:
                row.update(stratum=env['stratum'], clearance=env['clearance'])
            row['end_to_end_wall_s'] = row['planning_pipeline_wall_s'] + (content['deformation_wall_s'] if trial['mode'] != 'ompl_uniform' else 0.)
            row['par2_s'] = min(row['action_wall_s'], 10.) if row['success'] else 20.
            append_row(rows_path, row); completed.add(trial_key(row))
            print(f'PLAN {len(completed)}/{len(schedule)} env={active} repeat={trial["repeat"]} {trial["mode"]} success={row["success"]} stage={row.get("failure_stage")} wall={row["action_wall_s"]:.3f}', flush=True)
        if runtime_fingerprint(root) != fingerprint:
            raise RuntimeError('Runtime changed before completion')
        manifest.update(status='complete', completed_utc=utc()); save(manifest_path, manifest)
    finally:
        if saved_scene:
            runner.restore_scene()
        runner.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()
