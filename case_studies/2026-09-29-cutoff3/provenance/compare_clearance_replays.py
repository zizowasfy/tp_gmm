#!/usr/bin/env python3
"""Paired before/after clearance sensitivity on reused source environments."""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from analyze_sampling_study import MODES, LABELS, bootstrap_indices, interval, sign_pvalue, holm


def load(path):
    manifest=json.loads((path/'manifest.json').read_text())
    if manifest['status']!='complete':raise ValueError('Study is incomplete: '+str(path))
    rows=[json.loads(s) for s in (path/'results.jsonl').read_text().splitlines()]
    lookup={(r['environment'],r['repeat'],r['mode']):r for r in rows}
    expected={(r['environment'],r['repeat'],r['mode']) for r in manifest['schedule']}
    if set(lookup)!=expected or len(rows)!=len(lookup):raise ValueError('Incomplete or duplicate records')
    hashes={e['id']:e['sha256'] for e in manifest['environments']}
    for environment in manifest['environments']:
        content=json.loads((path/environment['file']).read_text())
        actual=hashlib.sha256(json.dumps(content,sort_keys=True).encode()).hexdigest()
        if actual!=environment['sha256']:raise ValueError('Frozen environment content changed')
        if 'replay' in manifest:
            verification=json.loads((path/f'scene_verification/{environment["id"]:03d}.json').read_text())
            if not verification['verified'] or verification['actual_sha256']!=verification['expected_sha256']:
                raise ValueError('Live scene verification failed')
    if any(r['environment_sha256']!=hashes[r['environment']] for r in rows):
        raise ValueError('Trial used a different frozen environment')
    return manifest,lookup


def payload(model):
    return {k:v for k,v in model.items() if k!='header'} | {'frame_id':model['header']['frame_id']}


def summarize_transition(selection,old_c,new_c,rng):
    strata=[c['cluster_stratum'] for c in selection];boot=bootstrap_indices(strata,rng)
    comparisons=[]
    for mode in MODES:
        before=[r for c in selection for r in c['rows'][mode][0]]
        after=[r for c in selection for r in c['rows'][mode][1]]
        old_rates=np.array([np.mean([r['success'] for r in c['rows'][mode][0]]) for c in selection])
        new_rates=np.array([np.mean([r['success'] for r in c['rows'][mode][1]]) for c in selection])
        diff=new_rates-old_rates
        comparisons.append(dict(mode=mode,old_clearance=old_c,new_clearance=new_c,environments=len(selection),plans=len(after),
            old_successes=sum(r['success'] for r in before),new_successes=sum(r['success'] for r in after),
            old_failure_stages=dict(Counter(r['failure_stage'] for r in before if not r['success'])),
            new_failure_stages=dict(Counter(r['failure_stage'] for r in after if not r['success'])),
            recovered_pairs=sum(not a['success'] and b['success'] for a,b in zip(before,after)),
            lost_pairs=sum(a['success'] and not b['success'] for a,b in zip(before,after)),
            success_difference=float(diff.mean()),ci95=interval(diff[boot].mean(axis=1)),p=sign_pvalue(diff,rng),
            uniform_attempts=sum(r['sampling'].get('uniform_attempts',0) for r in after) if mode!='ompl_uniform' else None,
            online_ik_calls=sum(r['sampling'].get('online_ik_calls',0) for r in after) if mode=='joint_projected' else None))
    for c,p in zip(comparisons,holm([c['p'] for c in comparisons])):c['holm_p']=float(p)
    return comparisons


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--before',type=Path,nargs='+',required=True)
    parser.add_argument('--after',type=Path,nargs='+',required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    if len(args.before)!=len(args.after):parser.error('Pair each before directory with one after directory')
    cases=[];verification=[]
    for phase,(old_path,new_path) in enumerate(zip(args.before,args.after)):
        old,old_rows=load(old_path);new,new_rows=load(new_path)
        if new['replay']['source_manifest_sha256']!=hashlib.sha256((old_path/'manifest.json').read_bytes()).hexdigest():
            raise ValueError('Source manifest mismatch')
        if old['schedule']!=new['schedule'] or old['design']!=new['design'] or old['settings']!=new['settings'] or old['runtime']!=new['runtime']:
            raise ValueError('Experimental controls changed')
        if len(old['environments'])!=len(new['environments']):raise ValueError('Environment count changed')
        for original,replayed in zip(old['environments'],new['environments']):
            e=original['id']
            if e!=replayed['id']:raise ValueError('Environment IDs changed')
            a=json.loads((old_path/original['file']).read_text());b=json.loads((new_path/replayed['file']).read_text())
            for key in ('case','start','goal','scene'):
                if a[key]!=b[key]:raise ValueError(f'Changed {key}: {phase}/{e}')
            if payload(a['response']['original_gmm'])!=payload(b['response']['original_gmm']):raise ValueError('Prior changed')
            if a['response']['policy_sha256']!=b['response']['policy_sha256'] or a['response']['action_scale']!=b['response']['action_scale']:raise ValueError('Policy changed')
            verification.append(dict(phase=phase,environment=e,source_sha256=original['sha256'],new_sha256=replayed['sha256'],inputs_identical=True))
            rows={m:([old_rows[e,j,m] for j in range(old['design']['repeats'])],
                     [new_rows[e,j,m] for j in range(old['design']['repeats'])]) for m in MODES}
            cases.append(dict(phase=phase,environment=e,old_clearance=original['clearance'],new_clearance=replayed['clearance'],
                cluster_stratum=f'{phase}/'+original['stratum'].split('_c')[0],rows=rows))
    args.output.mkdir(parents=True,exist_ok=True)
    (args.output/'input_verification.json').write_text(json.dumps(verification,indent=2)+'\n')
    transitions=sorted({(c['old_clearance'],c['new_clearance']) for c in cases})
    results=dict(environments=len(cases),comparisons=[],secondary=[],failure_cases=[],by_phase={},
        note='New minus old; environment-weighted inference, raw pooled counts descriptive. Reused environments; not independent replication.')
    rng=np.random.default_rng(20260928)
    for old_c,new_c in transitions:
        selection=[c for c in cases if c['old_clearance']==old_c and c['new_clearance']==new_c]
        results['comparisons']+=summarize_transition(selection,old_c,new_c,rng)
        for phase in range(len(args.before)):
            part=[c for c in selection if c['phase']==phase]
            results['by_phase'].setdefault(str(phase),[]).extend(summarize_transition(part,old_c,new_c,rng))
        for mode in MODES:
            for metric,section in [('joint_path_length','audit'),('ee_path_length_m','audit'),('min_world_clearance_m','audit'),('min_clearance_m','curve')]:
                diffs=[];strata=[];matched=0
                for c in selection:
                    pairs=[(a,b) for a,b in zip(*c['rows'][mode]) if a['success'] and b['success']]
                    values=[b[section][metric]-a[section][metric] for a,b in pairs if a[section].get(metric) is not None and b[section].get(metric) is not None]
                    if values:
                        diffs.append(np.mean(values));strata.append(c['cluster_stratum']);matched+=len(values)
                if diffs:
                    d=np.array(diffs);boot=bootstrap_indices(strata,rng)
                    results['secondary'].append(dict(mode=mode,old_clearance=old_c,new_clearance=new_c,metric=section+'.'+metric,
                        difference=float(d.mean()),ci95=interval(d[boot].mean(axis=1)),environments=len(d),matched_pairs=matched))
    for c in cases:
        for mode in MODES:
            old,new=c['rows'][mode]
            old_fail=Counter(r['failure_stage'] for r in old if not r['success']);new_fail=Counter(r['failure_stage'] for r in new if not r['success'])
            if old_fail or new_fail:
                results['failure_cases'].append({k:c[k] for k in ('phase','environment','old_clearance','new_clearance')} |
                    dict(mode=mode,old_failures=dict(old_fail),new_failures=dict(new_fail)))
    (args.output/'clearance_comparison.json').write_text(json.dumps(results,indent=2)+'\n')
    plt.rcParams.update({'font.size':10,'axes.spines.top':False,'axes.spines.right':False,'pdf.fonttype':42})
    figures=args.output/'figures';figures.mkdir(exist_ok=True)
    fig,axes=plt.subplots(1,len(transitions),figsize=(6*len(transitions),4.5),squeeze=False,layout='constrained')
    for ax,(old_c,new_c) in zip(axes[0],transitions):
        comparisons=[r for r in results['comparisons'] if r['old_clearance']==old_c and r['new_clearance']==new_c]
        old_fail=[r['plans']-r['old_successes'] for r in comparisons];new_fail=[r['plans']-r['new_successes'] for r in comparisons]
        ax.bar(np.arange(3)-.18,old_fail,width=.36,label=f'Before: {old_c:g}',color='#777777')
        ax.bar(np.arange(3)+.18,new_fail,width=.36,label=f'Replay: {new_c:g}',color='#0072B2')
        ax.set_xticks(range(3),['Cartesian\n+ IK','Joint\nprojection','OMPL\nunrestricted']);ax.set_ylabel('Failed plans')
        ax.set_title(f'{old_c:g} → {new_c:g}: '+('reduced clearance' if new_c<old_c else 'increased clearance'))
        ax.legend();ax.set_ylim(0,max(old_fail+new_fail+[1])*1.3)
        for i,(a,b) in enumerate(zip(old_fail,new_fail)):
            ax.text(i-.18,a+.2,str(a),ha='center');ax.text(i+.18,b+.2,str(b),ha='center')
    fig.suptitle('Same scenes, joint endpoints and prior GMM; changed requested clearance')
    for ext in ('png','pdf'):fig.savefig(figures/f'failure_comparison.{ext}',dpi=200)
    plt.close(fig)
    fig,axes=plt.subplots(1,len(transitions),figsize=(6*len(transitions),4),squeeze=False,layout='constrained')
    for ax,(old_c,new_c) in zip(axes[0],transitions):
        comparisons=[r for r in results['comparisons'] if r['old_clearance']==old_c and r['new_clearance']==new_c]
        v=np.array([r['success_difference']*100 for r in comparisons]);ci=np.array([r['ci95'] for r in comparisons])*100
        ax.errorbar(v,range(3),xerr=np.maximum(0,np.array([v-ci[:,0],ci[:,1]-v])),fmt='o',capsize=4)
        ax.axvline(0,color='gray',ls=':');ax.set_yticks(range(3),LABELS);ax.invert_yaxis()
        ax.set_xlabel('Success change (percentage points)');ax.set_title(f'Clearance {old_c:g} → {new_c:g}')
    fig.suptitle('Paired environment-cluster differences and pointwise 95% intervals')
    for ext in ('png','pdf'):fig.savefig(figures/f'paired_success_change.{ext}',dpi=200)
    plt.close(fig)
    fig,axes=plt.subplots(2,len(transitions),figsize=(6*len(transitions),6.5),squeeze=False,layout='constrained')
    for column,(old_c,new_c) in enumerate(transitions):
        for row,(metric,label) in enumerate((('audit.min_world_clearance_m','Minimum robot–world distance'),
                                            ('curve.min_clearance_m','Minimum EE–cylinder distance'))):
            ax=axes[row,column]
            values=[r for mode in MODES for r in results['secondary'] if r['old_clearance']==old_c and r['new_clearance']==new_c
                    and r['mode']==mode and r['metric']==metric]
            if values:
                v=np.array([r['difference'] for r in values])*1000;ci=np.array([r['ci95'] for r in values])*1000
                ax.errorbar(v,[MODES.index(r['mode']) for r in values],
                    xerr=np.maximum(0,np.array([v-ci[:,0],ci[:,1]-v])),fmt='o',capsize=4)
            else:
                ax.text(.5,.5,'No matched successful plans',transform=ax.transAxes,ha='center')
            ax.axvline(0,color='gray',ls=':');ax.set_yticks(range(3),LABELS);ax.invert_yaxis()
            ax.set_xlabel('Replay minus previous (mm)');ax.set_title(f'{old_c:g} → {new_c:g}: {label}')
    fig.suptitle('Measured path clearance: matched successes; 95% environment-cluster intervals')
    for ext in ('png','pdf'):fig.savefig(figures/f'paired_clearance_change.{ext}',dpi=200)
    plt.close(fig)
    lines=['# Clearance sensitivity: paired replay results','',f'{len(cases)} reused environments; the requested RL clearance is the experimental change.',
        '', '| Clearance | Sampler | Previous successes | Replay successes | Success change (pp), 95% cluster CI | Holm p |',
        '|---|---|---:|---:|---:|---:|']
    for c in results['comparisons']:
        lines.append(f'| {c["old_clearance"]:g} → {c["new_clearance"]:g} | {c["mode"]} | {c["old_successes"]}/{c["plans"]} | {c["new_successes"]}/{c["plans"]} | {100*c["success_difference"]:.2f} [{100*c["ci95"][0]:.2f}, {100*c["ci95"][1]:.2f}] | {c["holm_p"]:.5g} |')
    lines+=['', 'Counts pool the two source cohorts; inferential differences give equal weight to each environment. Repeated plans are not independent observations. The two clearance transitions have separate three-sampler Holm families. Pointwise bootstrap intervals and corrected sign-test p-values need not agree, particularly with few discordant environments.',
        '', 'The 0.8→0.7 subset directly tests reduced clearance. The 0.2→0.5 subset is an increase; do not describe the complete replay as uniformly lowering clearance. Unrestricted OMPL does not use the clearance input, so its differences measure stochastic rerun variability.',
        '', '## Failure mechanisms', '', '| Clearance | Sampler | Previous | Replay |','|---|---|---|---|']
    for c in results['comparisons']:
        lines.append(f'| {c["old_clearance"]:g} → {c["new_clearance"]:g} | {c["mode"]} | {json.dumps(c["old_failure_stages"])} | {json.dumps(c["new_failure_stages"])} |')
    lines+=['','![Failure comparison](figures/failure_comparison.png)', '', '![Paired success change](figures/paired_success_change.png)',
        '', '![Matched-success clearance change](figures/paired_clearance_change.png)',
        '', 'Detailed matched-success clearance/path effects and case-level failure transitions are in `clearance_comparison.json`. The saved inputs were verified bit-for-bit for case geometry, complete joint states, scene and prior GMM payload (ignoring the model header timestamp). Policy/checkpoint and action scale were also verified. Cutoff 2.0 and strict no-fallback sampling remain in force.',
        '', 'Goal heights remain in the original case-study range, 0.45–0.54 m. The saved training configuration uses 0.10–0.30 m. A change in outcomes cannot isolate goal-height mismatch as the cause because this replay did not change goal height.']
    (args.output/'summary.md').write_text('\n'.join(lines)+'\n')
    print(json.dumps(results['comparisons'],indent=2))


if __name__=='__main__':main()
