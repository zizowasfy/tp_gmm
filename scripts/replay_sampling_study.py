#!/usr/bin/env python3
"""Replay frozen cases and joint endpoints with explicit clearance/covariance cutoff."""
import argparse
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import os
import shutil
import time
import numpy as np
import rclpy
from moveit_msgs.msg import PlanningScene, RobotState, CollisionObject
from tp_gmm.srv import DeformTPGMM
from run_sampling_study import StudyRunner, PURE, MODES, pure_settings, save, digest, message, asdict

SCENE_RESTORE='full_then_diff_with_readback_v1'

def model_payload(model):
    return {k:v for k,v in model.items() if k != 'header'} | {'frame_id':model['header']['frame_id']}


def scene_payload(scene):
    """Collision-relevant snapshot, independent of live robot-state/TF timestamps."""
    result=deepcopy({k:scene[k] for k in ('robot_model_name','fixed_frame_transforms',
        'allowed_collision_matrix','link_padding','link_scale','world')})
    for transform in result['fixed_frame_transforms']:
        transform['header'].pop('stamp',None)
    result['fixed_frame_transforms'].sort(key=lambda t:(t['header']['frame_id'],t['child_frame_id']))
    for obj in result['world']['collision_objects']:
        obj['header'].pop('stamp',None)
    result['world']['collision_objects'].sort(key=lambda o:o['id'])
    octomap=result['world']['octomap']
    if not octomap['octomap']['data']:
        result['world']['octomap']=None
    return result


def restore_frozen_scene(runner,scene):
    # In this MoveIt build, a full-scene update can leave the monitored child
    # world's copy one update behind the parent. Explicitly synchronize the
    # child with a diff, then read it back; a successful service call is not proof.
    runner.apply_scene(scene)
    observed=runner.scene_snapshot()
    update=deepcopy(scene);update.is_diff=True
    wanted={obj.id for obj in scene.world.collision_objects}
    update.world.collision_objects.extend(CollisionObject(id=obj.id,operation=CollisionObject.REMOVE)
        for obj in observed.world.collision_objects if obj.id not in wanted)
    runner.apply_scene(update)
    actual=scene_payload(asdict(runner.scene_snapshot()))
    expected=scene_payload(asdict(scene))
    if actual!=expected:
        raise RuntimeError('Live collision scene differs from frozen input; refusing to plan')
    return dict(expected_sha256=digest(expected),actual_sha256=digest(actual),verified=True)


def clearance_mapping(source_levels, targets):
    if len(targets) not in (1,len(source_levels)) or not all(np.isfinite(x) and 0 <= x <= 1 for x in targets):
        raise ValueError('Supply one clearance for all cases or one per sorted source level, in [0,1]')
    return {float(old):float(targets[0] if len(targets)==1 else targets[i]) for i,old in enumerate(source_levels)}


def check_replay(source, runtime):
    if source['status'] != 'complete' or source['settings'] != pure_settings(source['settings']['cutoff']) or tuple(source['modes']) != MODES:
        raise ValueError('Source must be a completed study with matching pure sampler settings')
    if source['runtime'] != runtime:
        raise ValueError('Planner configuration changed from source study')
    expected={(e['id'],j,m) for e in source['environments'] for j in range(source['design']['repeats']) for m in MODES}
    actual={(r['environment'],r['repeat'],r['mode']) for r in source['schedule']}
    if actual != expected or len(actual) != len(source['schedule']):
        raise ValueError('Malformed source schedule')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source',type=Path,required=True,help='Completed source study directory')
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--clearances',type=float,nargs='+',default=[.5,.7],help='Targets for sorted old levels, or one value for every case')
    parser.add_argument('--cutoff',type=float,help='Override covariance cutoff only; defaults to the source cutoff')
    parser.add_argument('--resume',action='store_true')
    parser.add_argument('--design-only',action='store_true')
    args,ros_args=parser.parse_known_args()
    source_path=args.source.resolve()/'manifest.json'
    source=json.loads(source_path.read_text())
    args.cutoff=source['settings']['cutoff'] if args.cutoff is None else args.cutoff
    settings=pure_settings(args.cutoff)
    targets=clearance_mapping(sorted({e['clearance'] for e in source['environments']}),args.clearances)
    args.task,args.ref_robot,args.clearance,args.visualize,args.sampler_config='franka_pick_cube','native',.5,False,None
    args.output.mkdir(parents=True,exist_ok=True)
    manifest_path=args.output/'manifest.json'
    if manifest_path.exists() and not args.resume:
        parser.error('Output exists; use a new directory or --resume')
    fingerprint=dict(source_manifest_sha256=hashlib.sha256(source_path.read_bytes()).hexdigest(),
                     clearance_map={str(k):v for k,v in targets.items()})
    if args.cutoff!=source['settings']['cutoff']:
        fingerprint.update(kind='cutoff' if all(k==v for k,v in targets.items()) else 'clearance_and_cutoff',
                           source_cutoff=source['settings']['cutoff'],cutoff=args.cutoff)
    # Source artefact paths are relative to the workspace; commands are run there.
    archived_hashes=json.loads((args.source/'provenance/artifact_sha256.json').read_text())
    for name,expected in archived_hashes.items():
        p=Path(name)
        if not p.is_file() or hashlib.sha256(p.read_bytes()).hexdigest()!=expected:
            raise ValueError('Runtime/model differs from original study: '+name)
    rclpy.init(args=ros_args)
    runner=StudyRunner(args)
    saved_scene=False
    try:
        runtime=runner.check_configuration();check_replay(source,runtime)
        runner.save_scene();saved_scene=True
        if manifest_path.exists():
            manifest=json.loads(manifest_path.read_text())
            if manifest['replay'] != fingerprint or manifest['runtime'] != runtime or manifest['settings']!=settings:
                raise ValueError('Resume input, clearances or runtime changed')
            if manifest.get('scene_restore_method')!=SCENE_RESTORE or manifest['status'] not in ('frozen','complete'):
                raise ValueError('Resume requires frozen inputs prepared for verified scene restoration')
        else:
            manifest=deepcopy(source)
            for key in ('completed_utc','source_sha256','git'):
                manifest.pop(key,None)
            manifest.update(status='designing',created_utc=time.strftime('%Y-%m-%dT%H:%M:%SZ',time.gmtime()),
                            replay=fingerprint,settings=settings,environments=[],source_directory=str(args.source.resolve()),scene_restore_method=SCENE_RESTORE)
            manifest['analysis']=deepcopy(source['analysis'])
            manifest['analysis']['replay_note']='Sensitivity rerun on original environments, not independent confirmation. Changed clearance/cutoff are explicit in replay metadata; compare per source environment.'
            manifest['source_sha256']={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in (Path(__file__),Path(__file__).with_name('run_sampling_study.py'))}
            save(manifest_path,manifest)
            provenance=args.output/'provenance';provenance.mkdir(exist_ok=True)
            save(provenance/'artifact_sha256.json',archived_hashes)
            save(provenance/'source_manifest.json',source)
            for script in ('replay_sampling_study.py','run_sampling_study.py','run_clearance_sweep.py','compare_sampling_approaches.py'):
                shutil.copy2(Path(__file__).with_name(script),provenance/script)
            for environment in source['environments']:
                old=json.loads((args.source/environment['file']).read_text())
                if digest(old) != environment['sha256']:
                    raise ValueError('Source environment changed')
                restore_frozen_scene(runner,message(PlanningScene,old['scene']))
                clearance=targets[float(environment['clearance'])]
                response,_,wall=runner.deform(old['case'],clearance,'top')
                new_response=asdict(response)
                if response.policy_sha256 != source['policy_sha256'] or response.action_scale != old['response']['action_scale']:
                    raise ValueError('RL checkpoint or action scale changed')
                if model_payload(new_response['original_gmm']) != model_payload(old['response']['original_gmm']):
                    raise ValueError('Reproduced prior GMM changed for saved environment')
                if clearance==environment['clearance'] and model_payload(new_response['deformed_gmm'])!=model_payload(old['response']['deformed_gmm']):
                    raise ValueError('Deformed GMM changed despite unchanged clearance')
                content=deepcopy(old)
                content.update(response=new_response,deformation_wall_s=wall,
                    pipeline={k:getattr(response,k) for k in ('model_load_s','reproduction_s','rl_deformation_s','deformed_regression_s','total_s')})
                for key in ('case','start','goal','scene'):
                    if content[key]!=old[key]:raise ValueError('Frozen input changed: '+key)
                save(args.output/environment['file'],content)
                replay_environment=deepcopy(environment)
                replay_environment.update(clearance=clearance,source_clearance=environment['clearance'],
                    source_stratum=environment['stratum'],source_sha256=environment['sha256'],sha256=digest(content),
                    stratum=environment['stratum'].split('_c')[0]+f'_c{clearance:g}_from{environment["clearance"]:g}')
                manifest['environments'].append(replay_environment)
                save(manifest_path,manifest)
                print(f'DESIGN {len(manifest["environments"])}/{len(source["environments"])}: clearance {environment["clearance"]} -> {clearance}; exact endpoints/scene/prior verified',flush=True)
            manifest['status']='frozen';save(manifest_path,manifest)
        if len(manifest['environments']) != len(source['environments']):
            raise ValueError('Incomplete design; use a new output directory')
        if args.design_only:return
        rows_path=args.output/'results.jsonl'
        old_rows=[json.loads(s) for s in rows_path.read_text().splitlines()] if rows_path.exists() else []
        completed={(r['environment'],r['repeat'],r['mode']) for r in old_rows}
        if len(completed)!=len(old_rows):raise ValueError('Duplicate saved trial keys')
        active=None
        for trial in manifest['schedule']:
            key=(trial['environment'],trial['repeat'],trial['mode'])
            if key in completed:continue
            if active!=trial['environment']:
                active=trial['environment'];environment=manifest['environments'][active]
                content=json.loads((args.output/environment['file']).read_text())
                if digest(content)!=environment['sha256']:raise ValueError('Replay environment changed')
                scene=message(PlanningScene,content['scene'])
                verification=restore_frozen_scene(runner,scene)
                save(args.output/f'scene_verification/{active:03d}.json',verification)
                response=message(DeformTPGMM.Response,content['response'])
                runner.original_model,runner.deformed_model=response.original_gmm,response.deformed_gmm
                runner.original_reference,runner.deformed_reference=response.original_trajectory,response.deformed_trajectory
                start,goal=message(RobotState,content['start']),message(RobotState,content['goal'])
            result=runner.study_plan(start,goal,content['case'],trial['mode'],trial['seed'],manifest['design']['planning_time'],scene)
            row=trial | dict(stratum=environment['stratum'],clearance=environment['clearance'],source_clearance=environment['source_clearance'],
                pipeline=content['pipeline'],environment_sha256=environment['sha256']) | result
            row['end_to_end_wall_s']=row['planning_pipeline_wall_s']+(content['deformation_wall_s'] if trial['mode']!='ompl_uniform' else 0)
            budget=manifest['design']['planning_time']
            row['par2_s']=min(row['action_wall_s'],budget) if row['success'] else 2*budget
            with rows_path.open('a') as out:
                out.write(json.dumps(row,allow_nan=False)+'\n');out.flush();os.fsync(out.fileno())
            completed.add(key)
            print(f'PLAN {len(completed)}/{len(manifest["schedule"])} env={active} repeat={trial["repeat"]} {trial["mode"]} success={row["success"]} wall={row["action_wall_s"]:.3f}',flush=True)
        manifest.update(status='complete',completed_utc=time.strftime('%Y-%m-%dT%H:%M:%SZ',time.gmtime()))
        save(manifest_path,manifest)
    finally:
        if saved_scene:runner.restore_scene()
        runner.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
