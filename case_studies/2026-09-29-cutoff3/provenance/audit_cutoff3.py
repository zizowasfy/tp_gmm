from pathlib import Path
from collections import Counter
import hashlib
import json
import sys
sys.path.insert(0,'src/tp_gmm/scripts')
from compare_cutoff_replays import verify_controls,verify_environment,verify_purity
from compare_clearance_replays import load

workspace=Path('/home/zizo/the_folder/ws_moveit')
result=dict(cohorts={},checks=[],total_outcomes=0,live_scenes=0)
for phase,old_name in (('primary','2026-09-26-pure-study'),('confirmation','2026-09-26-path-confirmation')):
    source=workspace/'sampling_results'/old_name
    root=workspace/f'sampling_results/2026-09-29-cutoff3-{phase}'
    old,_=load(source);new,rows=load(root)
    verify_controls(old,new);verify_purity(rows.values(),3.)
    assert hashlib.sha256((source/'manifest.json').read_bytes()).hexdigest()==new['replay']['source_manifest_sha256']
    for a,b in zip(old['environments'],new['environments']):
        before=json.loads((source/a['file']).read_text());after=json.loads((root/b['file']).read_text())
        verify_environment(before,after)
        table=next(o for o in after['scene']['world']['collision_objects'] if o['id']=='tpgmm_benchmark_table')
        assert table['pose']['position']['z']+table['primitive_poses'][0]['position']['z']+table['primitives'][0]['dimensions'][2]/2==.25
        assert .45<=after['case']['goal'][2]<=.54
        assert b['clearance'] in (.2,.8) and b['clearance']==a['clearance']
    hashes=json.loads((root/'provenance/artifact_sha256.json').read_text())
    for path,digest in hashes.items():assert hashlib.sha256((workspace/path).read_bytes()).hexdigest()==digest,path
    for path,digest in new['source_sha256'].items():
        assert hashlib.sha256((root/'provenance'/Path(path).name).read_bytes()).hexdigest()==digest,path
        assert hashlib.sha256(Path(path).read_bytes()).hexdigest()==digest,path
    assert (root/'provenance/protocol.md').read_bytes()==(workspace/'src/tp_gmm/CUTOFF_STUDY.md').read_bytes()
    strict=json.loads((root/'provenance/strict_runtime_test.log').read_text().splitlines()[-1])
    assert strict['result']=='PASS' and strict['cutoff']==3. and strict['uniform_attempts']==0
    methods={}
    for mode in new['modes']:
        subset=[r for r in rows.values() if r['mode']==mode]
        methods[mode]=dict(outcomes=len(subset),successes=sum(r['success'] for r in subset),
            failures=dict(Counter(r['failure_stage'] for r in subset if not r['success'])),
            action_requests=sum('error_code' in r for r in subset),
            **{k:sum(r['sampling'].get(k,0) for r in subset) for k in
               ('attempts','valid_samples','uniform_attempts','missing_anchor_fallbacks','online_ik_calls')})
    result['cohorts'][phase]=dict(methods=methods,runtime_artifacts=hashes,strict_exhaustion_test=strict)
    result['total_outcomes']+=len(rows);result['live_scenes']+=len(new['environments'])
result['checks']=[
    'All original cases, full start/goal states and collision scenes unchanged.',
    'All prior/deformed GMMs and both GMR references identical except header timestamps.',
    'All 120 original table surfaces remain z=0.25 m; goal z remains 0.45–0.54 m; requested clearances remain 0.2/0.8.',
    'Schedule, repeats, model/checkpoint, runtime and non-cutoff settings unchanged.',
    'Both custom methods record covariance cutoff 3.0, zero uniform fallback and missing-anchor rescue; projection has zero online IK.',
    'All outcome keys, frozen-input content hashes and live scene read-back hashes verified.',
    'Runtime/model artifacts and frozen runner source snapshots match their recorded SHA-256.',
    'Protocol was frozen before measurements; cutoff-3.0 exhausted-sampler runtime regression passed.']
assert result['total_outcomes']==2700 and result['live_scenes']==120
output=workspace/'sampling_results/2026-09-29-cutoff3-comparison/validation.json'
output.parent.mkdir(parents=True,exist_ok=True)
output.write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps(result,indent=2))
