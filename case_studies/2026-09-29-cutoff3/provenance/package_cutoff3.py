from pathlib import Path
import hashlib
import json
import shutil
import subprocess
import tarfile

workspace=Path('/home/zizo/the_folder/ws_moveit')
root=workspace/'src/tp_gmm/case_studies/2026-09-29-cutoff3'
root.mkdir(parents=True,exist_ok=True)
checks=[]
for phase in ('primary','confirmation'):
    source=workspace/f'sampling_results/2026-09-29-cutoff3-{phase}'
    assert json.loads((source/'manifest.json').read_text())['status']=='complete'
    target=root/phase;target.mkdir(exist_ok=True)
    for name in ('summary.md','statistics.json','trials.csv','endpoint_audit.json','distribution_diagnostics.json','manifest.json'):
        shutil.copy2(source/name,target/name)
    for name in ('figures','scene_verification'):
        shutil.copytree(source/name,target/name,dirs_exist_ok=True)
    for name in ('analyze_sampling_study.py','compare_cutoff_replays.py','compare_clearance_replays.py','sampling_fidelity.py'):
        shutil.copy2(workspace/'src/tp_gmm/scripts'/name,source/'provenance'/name)
    shutil.copy2(f'/tmp/cutoff3-{phase}-plans.log',source/'provenance/planning_progress.log')
    archive=root/f'raw-{phase}.tar.xz'
    with tarfile.open(archive,'w:xz',preset=6) as tar:tar.add(source,arcname=phase)
    checked=0
    with tarfile.open(archive,'r:xz') as tar:
        for member in tar:
            if member.isfile():
                relative=Path(member.name).relative_to(phase)
                assert hashlib.sha256(tar.extractfile(member).read()).digest()==hashlib.sha256((source/relative).read_bytes()).digest(),member.name
                checked+=1
    checks.append(dict(archive=archive.name,bytes=archive.stat().st_size,verified_files=checked))
    print(checks[-1],flush=True)
shutil.copytree(workspace/'sampling_results/2026-09-29-cutoff3-comparison',root/'comparison',dirs_exist_ok=True)
shutil.copy2(workspace/'src/tp_gmm/CUTOFF_STUDY.md',root/'protocol.md')
(root/'archive_verification.json').write_text(json.dumps(checks,indent=2)+'\n')
provenance=root/'provenance';provenance.mkdir(exist_ok=True)
shutil.copy2('/tmp/audit_cutoff3.py',provenance/'audit_cutoff3.py')
shutil.copy2('/tmp/package_cutoff3.py',provenance/'package_cutoff3.py')
for name in ('test_cutoff_study.py','test_cutoff_replay_analysis.py','test_strict_sampling_runtime.py'):
    shutil.copy2(workspace/'src/tp_gmm/tests'/name,provenance/name)
for name in ('compare_cutoff_replays.py','analyze_sampling_study.py','compare_clearance_replays.py','sampling_fidelity.py'):
    shutil.copy2(workspace/'src/tp_gmm/scripts'/name,provenance/name)
git_state={}
for repo in ('tp_gmm','moveit2','moveit_resources'):
    location=workspace/'src'/repo
    git_state[repo]={name:subprocess.check_output(['git',*command],cwd=location,text=True).strip() for name,command in
        [('branch',['branch','--show-current']),('head',['rev-parse','HEAD']),('status',['status','--short'])]}
    (provenance/f'{repo}.patch').write_bytes(subprocess.check_output(['git','diff','HEAD','--binary'],cwd=location))
(provenance/'git_state.json').write_text(json.dumps(git_state,indent=2)+'\n')
print(root,flush=True)
