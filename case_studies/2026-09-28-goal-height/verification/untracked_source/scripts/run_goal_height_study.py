#!/usr/bin/env python3
"""Paired original-height scenes and cloned lower-goal/lower-table scenes."""
import argparse
from copy import deepcopy
import hashlib
import json
import os
from pathlib import Path
import shutil
import time
import numpy as np
import rclpy
from moveit_msgs.msg import PlanningScene, RobotState
from moveit_msgs.srv import GetPositionIK, GetStateValidity
from tp_gmm.srv import DeformTPGMM
from run_sampling_study import StudyRunner, save, digest, message, asdict
from replay_sampling_study import check_replay, restore_frozen_scene, model_payload, SCENE_RESTORE
from compare_sampling_approaches import pose

ARMS=('control','training')


def mapped_goal_z(value,source_range,target_range):
    if not all(np.isfinite([value,*source_range,*target_range])) or source_range[0]>=source_range[1] or target_range[0]>=target_range[1]:
        raise ValueError('Invalid goal-height ranges')
    t=(value-source_range[0])/(source_range[1]-source_range[0])
    if not 0<=t<=1:raise ValueError('Source goal outside declared range')
    if tuple(source_range)==tuple(target_range):return float(value)
    return float(target_range[0]+t*(target_range[1]-target_range[0]))


def lower_table(scene,top):
    result=deepcopy(scene)
    table=[o for o in result['world']['collision_objects'] if o['id']=='tpgmm_benchmark_table']
    if len(table)!=1 or not np.isfinite(top):raise ValueError('Expected one benchmark table and finite height')
    obj=table[0]
    if len(obj['primitives'])!=1 or obj['primitives'][0]['type']!=1 or obj['pose']['orientation']!={'x':0.,'y':0.,'z':0.,'w':1.}:
        raise ValueError('Expected an axis-aligned box table')
    if obj['primitive_poses'][0]['position']!={'x':0.,'y':0.,'z':0.}:
        raise ValueError('Expected canonical table pose')
    obj['pose']['position']['z']=float(top-obj['primitives'][0]['dimensions'][2]/2)
    return result


def solve_goal(runner,case,seed):
    request=GetPositionIK.Request()
    request.ik_request.group_name=runner.group_name
    request.ik_request.ik_link_name=runner.ee_link
    request.ik_request.pose_stamped=pose(case['goal'],runner.frame_id)
    request.ik_request.avoid_collisions=True
    request.ik_request.timeout.sec=2
    request.ik_request.robot_state=seed
    return runner.service(runner.ik_client,request)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source',type=Path,required=True,help='Verified 0.5/0.7 clearance replay cohort')
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--goal-z-range',type=float,nargs=2,default=[.1,.3])
    parser.add_argument('--table-top-z',type=float,default=-.02,help='Training-height scene only; control table stays unchanged')
    parser.add_argument('--resume',action='store_true')
    parser.add_argument('--design-only',action='store_true')
    args,ros_args=parser.parse_known_args()
    args.task,args.ref_robot,args.clearance,args.visualize,args.sampler_config='franka_pick_cube','native',.5,False,None
    source_path=args.source.resolve()/'manifest.json';source=json.loads(source_path.read_text())
    if sorted({e['clearance'] for e in source['environments']})!=[.5,.7]:raise ValueError('Source clearances must be 0.5/0.7')
    ranges=source['environment_ranges'];old_range=[ranges['goal_low'][2],ranges['goal_high'][2]]
    mapped_goal_z(old_range[0],old_range,args.goal_z_range)
    if not np.isfinite(args.table_top_z):raise ValueError('Invalid table height')
    fingerprint=dict(kind='goal_height',source_manifest_sha256=hashlib.sha256(source_path.read_bytes()).hexdigest(),
        source_goal_z_range=old_range,training_goal_z_range=args.goal_z_range,table_top_z=args.table_top_z)
    hashes=json.loads((args.source/'provenance/artifact_sha256.json').read_text())
    for name,expected in hashes.items():
        if hashlib.sha256(Path(name).read_bytes()).hexdigest()!=expected:raise ValueError('Runtime/model changed: '+name)
    args.output.mkdir(parents=True,exist_ok=True);manifest_path=args.output/'manifest.json'
    if manifest_path.exists() and not args.resume:parser.error('Output exists; use --resume or another directory')
    rclpy.init(args=ros_args);runner=StudyRunner(args);saved=False
    try:
        runtime=runner.check_configuration();check_replay(source,runtime)
        runner.save_scene();saved=True
        if manifest_path.exists():
            parent=json.loads(manifest_path.read_text())
            if parent['intervention']!=fingerprint or parent['runtime']!=runtime or parent['status'] not in ('frozen','complete'):
                raise ValueError('Resume requires the same complete frozen design')
            manifests={a:json.loads((args.output/a/'manifest.json').read_text()) for a in ARMS}
        else:
            parent=dict(status='designing',intervention=fingerprint,runtime=runtime,
                created_utc=time.strftime('%Y-%m-%dT%H:%M:%SZ',time.gmtime()),
                note='Control scene and endpoints unchanged. Training scene clones source and lowers only table and goal z. Goal IK is necessarily recomputed. Endpoint infeasibility remains in the denominator. This is a combined height/table intervention.')
            manifests={}
            for arm in ARMS:
                m=deepcopy(source)
                for key in ('completed_utc','invalidated_attempt','preparation_note','source_sha256','git'):m.pop(key,None)
                m.update(status='designing',created_utc=parent['created_utc'],replay=fingerprint|dict(arm=arm),
                    source_directory=str(args.source.resolve()),environments=[],scene_restore_method=SCENE_RESTORE)
                m['analysis']['replay_note']='Paired height sensitivity on reused environments; not independent confirmation.'
                m['environment_ranges']['goal_low'][2],m['environment_ranges']['goal_high'][2]=old_range if arm=='control' else args.goal_z_range
                provenance=args.output/arm/'provenance';provenance.mkdir(parents=True,exist_ok=True)
                save(provenance/'source_manifest.json',source);save(provenance/'artifact_sha256.json',hashes)
                scripts=('run_goal_height_study.py','run_sampling_study.py','replay_sampling_study.py','run_clearance_sweep.py','compare_sampling_approaches.py')
                m['source_sha256']={}
                for name in scripts:
                    path=Path(__file__).with_name(name);shutil.copy2(path,provenance/name)
                    m['source_sha256'][name]=hashlib.sha256(path.read_bytes()).hexdigest()
                manifests[arm]=m;save(args.output/arm/'manifest.json',m)
            save(manifest_path,parent)
            for env in source['environments']:
                original=json.loads((args.source/env['file']).read_text())
                if digest(original)!=env['sha256']:raise ValueError('Source environment changed')
                for arm in ARMS:
                    scene=message(PlanningScene,original['scene'] if arm=='control' else lower_table(original['scene'],args.table_top_z))
                    restore_frozen_scene(runner,scene)
                    case=deepcopy(original['case'])
                    if arm=='training':case['goal'][2]=mapped_goal_z(case['goal'][2],old_range,args.goal_z_range)
                    start=message(RobotState,original['start'])
                    goal=message(RobotState,original['goal'])
                    status=dict(ik_error=1,feasible=True)
                    if arm=='training':
                        ik=solve_goal(runner,case,goal);goal=ik.solution;status['ik_error']=ik.error_code.val
                        if ik.error_code.val!=1:status.update(feasible=False,failure_stage='goal_ik_design')
                    for label,state in [('start',start),('goal',goal)]:
                        if not status['feasible']:break
                        valid=runner.service(runner.validity_client,GetStateValidity.Request(robot_state=state,group_name=runner.group_name))
                        status[label+'_validity']=asdict(valid)
                        if not valid.valid:status.update(feasible=False,failure_stage=label+'_invalid_design')
                    response,_,wall=runner.deform(case,env['clearance'],'top')
                    encoded=asdict(response)
                    if response.policy_sha256!=source['policy_sha256'] or response.action_scale!=original['response']['action_scale']:
                        raise ValueError('Policy or action scale changed')
                    if arm=='control' and model_payload(encoded['original_gmm'])!=model_payload(original['response']['original_gmm']):
                        raise ValueError('Control prior changed')
                    if arm=='control' and model_payload(encoded['deformed_gmm'])!=model_payload(original['response']['deformed_gmm']):
                        raise ValueError('Control deformation changed')
                    content=dict(case=case,start=asdict(start),goal=asdict(goal),scene=asdict(scene),response=encoded,
                        endpoint_status=status,deformation_wall_s=wall,
                        pipeline={k:getattr(response,k) for k in ('model_load_s','reproduction_s','rl_deformation_s','deformed_regression_s','total_s')})
                    save(args.output/arm/env['file'],content)
                    entry=deepcopy(env);entry.update(case=case,sha256=digest(content),source_sha256=env['sha256'],endpoint_feasible=status['feasible'])
                    manifests[arm]['environments'].append(entry);save(args.output/arm/'manifest.json',manifests[arm])
                    print(f'DESIGN {arm} {env["id"]+1}/{len(source["environments"])} z={case["goal"][2]:.4f} endpoint_feasible={status["feasible"]}',flush=True)
            rng=np.random.default_rng(source['design']['seed']+2809)
            parent['blocks']=[dict(environment=e,arm=str(a)) for e in dict.fromkeys(t['environment'] for t in source['schedule']) for a in rng.permutation(ARMS)]
            parent['status']='frozen';save(manifest_path,parent)
            for arm,m in manifests.items():m['status']='frozen';save(args.output/arm/'manifest.json',m)
        if args.design_only:return
        completed={}
        for arm in ARMS:
            path=args.output/arm/'results.jsonl'
            rows=[json.loads(s) for s in path.read_text().splitlines()] if path.exists() else []
            completed[arm]={(r['environment'],r['repeat'],r['mode']) for r in rows}
            if len(rows)!=len(completed[arm]):raise ValueError('Duplicate trial keys')
        total=2*len(source['schedule'])
        for block in parent['blocks']:
            arm,e=block['arm'],block['environment'];m=manifests[arm];env=m['environments'][e]
            trials=[t for t in m['schedule'] if t['environment']==e and (e,t['repeat'],t['mode']) not in completed[arm]]
            if not trials:continue
            content=json.loads((args.output/arm/env['file']).read_text())
            if digest(content)!=env['sha256']:raise ValueError('Frozen environment changed')
            scene=message(PlanningScene,content['scene']);verification=restore_frozen_scene(runner,scene)
            save(args.output/arm/f'scene_verification/{e:03d}.json',verification)
            response=message(DeformTPGMM.Response,content['response'])
            runner.original_model,runner.deformed_model=response.original_gmm,response.deformed_gmm
            runner.original_reference,runner.deformed_reference=response.original_trajectory,response.deformed_trajectory
            start,goal=message(RobotState,content['start']),message(RobotState,content['goal'])
            for t in trials:
                if content['endpoint_status']['feasible']:
                    result=runner.study_plan(start,goal,content['case'],t['mode'],t['seed'],m['design']['planning_time'],scene)
                else:
                    result=dict(success=False,planner_success=False,sampling={},action_wall_s=0.,moveit_planning_s=0.,
                        planning_pipeline_wall_s=0.,failure_stage=content['endpoint_status']['failure_stage'],planning_not_attempted=True)
                row=t|dict(stratum=env['stratum'],clearance=env['clearance'],height_arm=arm,goal_z=content['case']['goal'][2],
                    pipeline=content['pipeline'],environment_sha256=env['sha256'])|result
                row['end_to_end_wall_s']=row['planning_pipeline_wall_s']+(content['deformation_wall_s'] if t['mode']!='ompl_uniform' else 0.)
                row['par2_s']=min(row['action_wall_s'],m['design']['planning_time']) if row['success'] else 2*m['design']['planning_time']
                with (args.output/arm/'results.jsonl').open('a') as out:
                    out.write(json.dumps(row,allow_nan=False)+'\n');out.flush();os.fsync(out.fileno())
                completed[arm].add((e,t['repeat'],t['mode']))
                print(f'PLAN {sum(map(len,completed.values()))}/{total} {arm} env={e} repeat={t["repeat"]} {t["mode"]} success={row["success"]} wall={row["action_wall_s"]:.3f}',flush=True)
        for arm,m in manifests.items():
            m.update(status='complete',completed_utc=time.strftime('%Y-%m-%dT%H:%M:%SZ',time.gmtime()));save(args.output/arm/'manifest.json',m)
        parent.update(status='complete',completed_utc=time.strftime('%Y-%m-%dT%H:%M:%SZ',time.gmtime()));save(manifest_path,parent)
    finally:
        if saved:runner.restore_scene()
        runner.destroy_node();rclpy.shutdown()

if __name__=='__main__':main()
