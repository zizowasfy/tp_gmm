from pathlib import Path
from collections import Counter, defaultdict
import json, math
import argparse
parser=argparse.ArgumentParser(description='Independently check every saved request, endpoint, audit and purity counter.')
parser.add_argument('workspace', type=Path, help='Workspace or raw archive extraction root')
args=parser.parse_args()
ROOT=args.workspace.resolve()
OUT=Path(__file__).resolve().with_name('results_integrity.json')
summary={}
def consume(rows, endpoints, expected):
    modes=defaultdict(lambda: dict(requests=0,successes=0,failures=Counter(),attempts=0,valid_samples=0,
        uniform_attempts=0,missing_anchor_fallbacks=0,online_ik_calls=0,projected_attempts=0,gmm_attempts=0,gmr_attempts=0,
        exact_support_checks=0,exact_support_rejections=0,audited_samples=0,audit_invalid_samples=0,exact_support_violations=0,
        max_endpoint_error_rad=0.,warmups=0))
    seen=set();failures=[]
    for row in rows:
        mode=row.get('mode',row.get('sampler'));eid=row.get('trial',row.get('environment'))
        key=(eid,row.get('repeat',row.get('clearance')),mode)
        assert key not in seen,key
        seen.add(key);m=modes[mode];m['requests']+=1;m['successes']+=int(row['success']);m['warmups']+=int(row.get('warmup',False))
        stats=row.get('sampling',{})
        for k in ('attempts','valid_samples','uniform_attempts','missing_anchor_fallbacks','online_ik_calls',
                  'projected_attempts','gmm_attempts','gmr_attempts','exact_support_checks','exact_support_rejections'):
            m[k]+=stats.get(k,0)
        assert stats.get('uniform_attempts',0)==stats.get('missing_anchor_fallbacks',0)==0,key
        assert stats.get('uniform_fraction',0)==stats.get('cartesian_fraction',0)==0,key
        if mode=='ompl_uniform':assert not stats,key
        elif mode in ('cartesian_ik','joint_projected','gmm') and stats:
            assert stats['corridor_geometry']=='gmm_ellipsoids',key
            assert set(stats['component_cutoffs'])=={3.0},key
            assert stats['gmr_attempts']==0,key
            if mode=='joint_projected':assert stats['online_ik_calls']==0,key
            else:assert stats['projected_attempts']==0,key
            if row['success']:assert stats['exact_support_checks']>0,key
        elif mode in ('gmr_30mm','reference_ik') and stats:
            assert stats['corridor_geometry']=='gmr_tube',key
            assert stats['gmr_stddev']==.03 and stats['gmr_cutoff']==3.,key
            assert stats['gmm_attempts']==stats['projected_attempts']==0,key
        if row['success']:
            audit=row['audit'];assert audit['audit_invalid_samples']==0,key
            assert audit['audit_step_rad']==.005,key
            for k in ('audit_invalid_samples','exact_support_violations'):m[k]+=audit[k]
            m['audited_samples']+=audit['audit_samples']
            trajectory=row['trajectory']['joint_trajectory']
            for state_key,point in zip(('start','goal'),(trajectory['points'][0],trajectory['points'][-1])):
                q=endpoints[eid][state_key]['joint_state'];lookup=dict(zip(q['name'],q['position']))
                error=max(abs(value-lookup[name]) for name,value in zip(trajectory['joint_names'],point['positions']))
                assert error<=.0011,(key,state_key,error)
                m['max_endpoint_error_rad']=max(m['max_endpoint_error_rad'],error)
        else:
            m['failures'][row['failure_stage']]+=1
            failures.append(dict(environment=eid,repeat=row.get('repeat'),mode=mode,stage=row['failure_stage'],
                clearance=row.get('clearance'),layout=row.get('layout',row.get('stratum')),action_wall_s=row.get('action_wall_s')))
    assert len(seen)==expected,(len(seen),expected)
    return dict(rows=len(seen),modes=dict(modes),failures=failures)
for label,rel in [('mapping_primary','sampling_results/2026-10-13-framework-primary'),
                  ('mapping_confirmation','sampling_results/2026-10-13-framework-confirmation'),
                  ('gmr','gmr_results/2026-10-13-framework')]:
    p=ROOT/rel;manifest=json.loads((p/'manifest.json').read_text());assert manifest['status']=='complete'
    endpoints={env['id']:json.loads((p/env['file']).read_text()) for env in manifest['environments']}
    with (p/'results.jsonl').open() as f:summary[label]=consume((json.loads(line) for line in f),endpoints,len(manifest['schedule']))
    print(label,{m:(v['successes'],v['requests']) for m,v in summary[label]['modes'].items()},flush=True)
p=ROOT/'clearance_results/2026-10-13-framework/results.json';content=json.loads(p.read_text());assert content['metadata']['status']=='complete'
endpoints={e['trial']:e for e in content['metadata']['endpoint_states']}
summary['clearance']=consume(content['rows'],endpoints,150)
summary['all_checks_passed']=True
OUT.write_text(json.dumps(summary,indent=2)+'\n')
print('ALL_RESULT_INTEGRITY_CHECKS_PASSED',flush=True)
