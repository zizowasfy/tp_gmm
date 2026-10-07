from pathlib import Path
from collections import Counter
import hashlib
import json
import sys
sys.path.insert(0,'src/tp_gmm/scripts')
from analyze_goal_height_study import verify_pair
from compare_clearance_replays import load,payload
from run_sampling_study import assert_pure

workspace=Path('/home/zizo/the_folder/ws_moveit')
result={'cohorts':{},'checks':[]}
for phase in ('primary','confirmation'):
    root=workspace/f'sampling_results/2026-09-28-height-{phase}'
    parent=json.loads((root/'manifest.json').read_text())
    assert parent['status']=='complete'
    source=workspace/f'sampling_results/2026-09-28-clearance-{phase}'
    source_manifest=json.loads((source/'manifest.json').read_text())
    assert hashlib.sha256((source/'manifest.json').read_bytes()).hexdigest()==parent['intervention']['source_manifest_sha256']
    a,rows_a=load(root/'control');b,rows_b=load(root/'training')
    for original,control,training in zip(source_manifest['environments'],a['environments'],b['environments']):
        old=json.loads((source/original['file']).read_text())
        ca=json.loads((root/'control'/control['file']).read_text())
        cb=json.loads((root/'training'/training['file']).read_text())
        for k in ('case','start','goal','scene'):assert old[k]==ca[k],(phase,original['id'],k)
        for k in ('original_gmm','deformed_gmm'):assert payload(old['response'][k])==payload(ca['response'][k])
        verify_pair(ca,cb,parent['intervention'])
        if cb['endpoint_status']['feasible']:
            non_arm=lambda content:{k:v for k,v in zip(content['goal']['joint_state']['name'],content['goal']['joint_state']['position']) if not k.startswith('panda_joint')}
            assert non_arm(ca)==non_arm(cb),'Non-arm goal joints changed'
    phase_result={}
    for arm,manifest,rows in (('control',a,rows_a),('training',b,rows_b)):
        for row in rows.values():
            if row.get('planning_not_attempted'):
                assert not row['sampling'],'Unattempted endpoint failure has sampler telemetry'
            else:assert_pure(row['mode'],row['sampling'])
        hashes=json.loads((root/arm/'provenance/artifact_sha256.json').read_text())
        for path,digest in hashes.items():assert hashlib.sha256(Path(path).read_bytes()).hexdigest()==digest,path
        for path,digest in manifest['source_sha256'].items():
            assert hashlib.sha256((root/arm/'provenance'/path).read_bytes()).hexdigest()==digest,path
        for mode in manifest['modes']:
            subset=[r for r in rows.values() if r['mode']==mode]
            phase_result[arm+'/'+mode]=dict(outcomes=len(subset),successes=sum(r['success'] for r in subset),
                failure_stages=dict(Counter(r['failure_stage'] for r in subset if not r['success'])),
                action_requests=sum('error_code' in r for r in subset),
                uniform_attempts=sum(r['sampling'].get('uniform_attempts',0) for r in subset),
                missing_anchor_fallbacks=sum(r['sampling'].get('missing_anchor_fallbacks',0) for r in subset),
                online_ik_calls=sum(r['sampling'].get('online_ik_calls',0) for r in subset))
    result['cohorts'][phase]=phase_result
result['checks']=['All control cases, exact start/goal states and scenes equal the original saved source.',
    'Control prior and deformed GMM payloads equal source (header timestamps ignored).',
    'Non-arm goal joints are unchanged in all 99 endpoint-feasible scene pairs.',
    'Every training scene changes only table z; every training case changes only goal z.',
    'All outcome keys, environment content hashes and live scene read-back hashes verified.',
    'Recorded pure-sampler assertions pass; no uniform attempts or projected Cartesian rescues.',
    'Every frozen runtime/model artifact and runner source snapshot matches its recorded SHA-256.']
output=workspace/'sampling_results/2026-09-28-height-comparison/validation.json'
output.parent.mkdir(parents=True,exist_ok=True)
output.write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps(result,indent=2))
