#!/usr/bin/env python3
"""Paired covariance-cutoff sensitivity with frozen geometry, clearance and GMMs."""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from analyze_sampling_study import MODES,LABELS,bootstrap_indices,interval,sign_pvalue,holm
from compare_clearance_replays import load,payload


def verify_controls(old,new):
    for key in ('schedule','design','runtime','modes','policy_sha256','environment_ranges'):
        if old[key]!=new[key]:raise ValueError('Changed control: '+key)
    expected=old['settings'] | {'cutoff':new['settings']['cutoff']}
    if new['settings']!=expected:raise ValueError('Sampler settings changed beyond cutoff')
    if new['replay'].get('kind')!='cutoff':raise ValueError('Not a cutoff-only replay')
    if (new['replay']['source_cutoff'],new['replay']['cutoff'])!=(old['settings']['cutoff'],new['settings']['cutoff']):
        raise ValueError('Cutoff provenance mismatch')
    if any(float(k)!=v for k,v in new['replay']['clearance_map'].items()):raise ValueError('Requested clearance changed')


def verify_environment(a,b):
    for key in ('case','start','goal','scene'):
        if a[key]!=b[key]:raise ValueError('Changed frozen '+key)
    for key in ('original_gmm','deformed_gmm','original_trajectory','deformed_trajectory'):
        if payload(a['response'][key])!=payload(b['response'][key]):raise ValueError('Changed model/reference: '+key)
    for key in ('policy_sha256','action_scale'):
        if a['response'][key]!=b['response'][key]:raise ValueError('Changed policy: '+key)


def verify_purity(rows,cutoff):
    for row in rows:
        s=row['sampling'];mode=row['mode']
        if mode=='ompl_uniform':
            if s:raise ValueError('Uniform baseline used GMM plugin')
            continue
        forbidden=('uniform_attempts','missing_anchor_fallbacks','gmr_attempts',
                   'online_ik_calls' if mode=='joint_projected' else 'projected_attempts')
        if any(s.get(k,0)!=0 for k in forbidden):raise ValueError('Sampler purity violation')
        if s.get('proposal')!='gmm' or s.get('corridor_mode')!='covariance' or not s.get('component_cutoffs') or any(c!=cutoff for c in s['component_cutoffs']):
            raise ValueError('Recorded proposal/cutoff mismatch')


def summarize(cases,rng):
    boot=bootstrap_indices([c['stratum'] for c in cases],rng);result=[]
    for mode in MODES:
        before=[r for c in cases for r in c['rows'][mode][0]];after=[r for c in cases for r in c['rows'][mode][1]]
        a=np.array([np.mean([r['success'] for r in c['rows'][mode][0]]) for c in cases])
        b=np.array([np.mean([r['success'] for r in c['rows'][mode][1]]) for c in cases]);d=b-a
        result.append(dict(mode=mode,environments=len(cases),requests=len(after),
            before_successes=sum(r['success'] for r in before),after_successes=sum(r['success'] for r in after),
            before_rate=float(a.mean()),after_rate=float(b.mean()),before_ci95=interval(a[boot].mean(axis=1)),after_ci95=interval(b[boot].mean(axis=1)),
            difference=float(d.mean()),ci95=interval(d[boot].mean(axis=1)),p=sign_pvalue(d,rng),
            before_failures=dict(Counter(r['failure_stage'] for r in before if not r['success'])),
            after_failures=dict(Counter(r['failure_stage'] for r in after if not r['success'])),
            recovered_pairs=sum(not x['success'] and y['success'] for x,y in zip(before,after)),
            lost_pairs=sum(x['success'] and not y['success'] for x,y in zip(before,after))))
    for r,p in zip(result,holm([r['p'] for r in result])):r['holm_p']=float(p)
    return result


def descriptive(cases):
    return {mode:{arm:dict(requests=sum(len(c['rows'][mode][i]) for c in cases),
        successes=sum(r['success'] for c in cases for r in c['rows'][mode][i]),
        failures=dict(Counter(r['failure_stage'] for c in cases for r in c['rows'][mode][i] if not r['success'])))
        for i,arm in enumerate(('before','after'))} for mode in MODES}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--before',type=Path,nargs='+',required=True)
    parser.add_argument('--after',type=Path,nargs='+',required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    if len(args.before)!=len(args.after):parser.error('Pair each before/after cohort')
    cases=[];verified=[];cutoffs=set();purity=[]
    for phase,(before,after) in enumerate(zip(args.before,args.after)):
        old,old_rows=load(before);new,new_rows=load(after);verify_controls(old,new)
        if new['replay']['source_manifest_sha256']!=hashlib.sha256((before/'manifest.json').read_bytes()).hexdigest():raise ValueError('Wrong source manifest')
        if len(old['environments'])!=len(new['environments']):raise ValueError('Environment count changed')
        cutoffs.add((old['settings']['cutoff'],new['settings']['cutoff']))
        for arm,m,rows in (('before',old,old_rows),('after',new,new_rows)):
            verify_purity(rows.values(),m['settings']['cutoff'])
            for mode in MODES:
                selected=[r for r in rows.values() if r['mode']==mode]
                keys=('attempts','valid_samples','uniform_attempts','missing_anchor_fallbacks','online_ik_calls')
                purity.append(dict(phase=phase,arm=arm,mode=mode,cutoff=m['settings']['cutoff'],requests=len(selected),
                    action_requests=sum('error_code' in r for r in selected),**{k:sum(r['sampling'].get(k,0) for r in selected) for k in keys}))
        for a,b in zip(old['environments'],new['environments']):
            if a['id']!=b['id'] or a['clearance']!=b['clearance']:raise ValueError('Environment identity/clearance changed')
            ca=json.loads((before/a['file']).read_text());cb=json.loads((after/b['file']).read_text());verify_environment(ca,cb)
            e=a['id'];rows={m:([old_rows[e,j,m] for j in range(old['design']['repeats'])],
                               [new_rows[e,j,m] for j in range(new['design']['repeats'])]) for m in MODES}
            cases.append(dict(phase=phase,environment=e,clearance=a['clearance'],stratum=f'{phase}/'+a['stratum'],rows=rows))
            verified.append(dict(phase=phase,environment=e,source_sha256=a['sha256'],replay_sha256=b['sha256'],
                identical_geometry_endpoints_models_references=True))
    if len(cutoffs)!=1:raise ValueError('Cohorts have different cutoff transitions')
    before_cutoff,after_cutoff=next(iter(cutoffs));rng=np.random.default_rng(20260930)
    results=dict(before_cutoff=before_cutoff,after_cutoff=after_cutoff,environments=len(cases),primary=summarize(cases,rng),secondary=[],purity=purity,
        by_phase={str(p):descriptive([c for c in cases if c['phase']==p]) for p in range(len(args.before))},
        by_clearance={str(q):descriptive([c for c in cases if c['clearance']==q]) for q in sorted({c['clearance'] for c in cases})},failure_cases=[],
        note='Cutoff-only sensitivity; same original-height scenes, endpoints, clearances and GMM payloads. Reused environments, not independent confirmation.')
    for mode in MODES:
        for section,metric in [(None,'par2_s'),(None,'action_wall_s'),(None,'end_to_end_wall_s'),
                               ('audit','joint_path_length'),('audit','ee_path_length_m'),('audit','min_world_clearance_m'),('curve','min_clearance_m')]:
            values=[];strata=[];pairs_count=0
            for c in cases:
                pairs=list(zip(*c['rows'][mode]))
                if section:pairs=[(a,b) for a,b in pairs if a['success'] and b['success']]
                extract=lambda r:r[section].get(metric) if section else r.get(metric)
                d=[extract(b)-extract(a) for a,b in pairs if extract(a) is not None and extract(b) is not None]
                if d:values.append(np.mean(d));strata.append(c['stratum']);pairs_count+=len(d)
            if values:
                d=np.array(values);boot=bootstrap_indices(strata,rng)
                results['secondary'].append(dict(mode=mode,metric=(section+'.' if section else '')+metric,
                    difference=float(d.mean()),ci95=interval(d[boot].mean(axis=1)),environments=len(d),matched_requests=pairs_count,success_only=bool(section)))
    for c in cases:
        for mode in MODES:
            old,new=c['rows'][mode]
            if any(not r['success'] for r in old+new):results['failure_cases'].append({k:c[k] for k in ('phase','environment','clearance')} | dict(mode=mode,
                before=dict(Counter(r['failure_stage'] for r in old if not r['success'])),after=dict(Counter(r['failure_stage'] for r in new if not r['success']))))
    args.output.mkdir(parents=True,exist_ok=True)
    (args.output/'statistics.json').write_text(json.dumps(results,indent=2)+'\n')
    (args.output/'input_verification.json').write_text(json.dumps(verified,indent=2)+'\n')
    figures=args.output/'figures';figures.mkdir(exist_ok=True)
    plt.rcParams.update({'font.size':10,'axes.spines.top':False,'axes.spines.right':False,'pdf.fonttype':42})
    labels=['Cartesian\n+ IK','Joint\nprojection','OMPL\nunrestricted']
    fig,axes=plt.subplots(1,2,figsize=(11,4.5),layout='constrained')
    for offset,arm,cutoff,color in ((-.18,'before',before_cutoff,'#777777'),(.18,'after',after_cutoff,'#0072B2')):
        rates=np.array([r[arm+'_rate'] for r in results['primary']])*100;ci=np.array([r[arm+'_ci95'] for r in results['primary']])*100
        axes[0].bar(np.arange(3)+offset,rates,width=.36,label=f'Cutoff {cutoff:g}',color=color)
        axes[0].errorbar(np.arange(3)+offset,rates,yerr=np.maximum(0,[rates-ci[:,0],ci[:,1]-rates]),fmt='none',color='black',capsize=3)
    axes[0].set_xticks(range(3),labels);axes[0].set_ylim(0,105);axes[0].set_ylabel('Environment-weighted success (%)');axes[0].legend(loc='lower left')
    d=np.array([r['difference'] for r in results['primary']])*100;ci=np.array([r['ci95'] for r in results['primary']])*100
    axes[1].errorbar(d,range(3),xerr=np.maximum(0,[d-ci[:,0],ci[:,1]-d]),fmt='o',capsize=4)
    axes[1].set_yticks(range(3),LABELS);axes[1].invert_yaxis();axes[1].axvline(0,ls=':',color='gray');axes[1].set_xlabel('Success change (percentage points)')
    fig.suptitle(f'Covariance cutoff {before_cutoff:g} → {after_cutoff:g}; 95% paired environment-cluster intervals')
    for ext in ('png','pdf'):fig.savefig(figures/f'success_comparison.{ext}',dpi=200)
    plt.close(fig)
    fig,axes=plt.subplots(1,2,figsize=(11,4.5),sharey=True,layout='constrained')
    stages=sorted({k for r in results['primary'] for arm in ('before','after') for k in r[arm+'_failures']})
    for ax,arm,cutoff in zip(axes,('before','after'),(before_cutoff,after_cutoff)):
        bottom=np.zeros(3)
        for i,stage in enumerate(stages):
            vals=np.array([r[arm+'_failures'].get(stage,0) for r in results['primary']])
            if np.any(vals):ax.bar(range(3),vals,bottom=bottom,label=stage,color=plt.get_cmap('tab10')(i));bottom+=vals
        for i,total in enumerate(bottom):ax.annotate(str(int(total)),(i,total),xytext=(0,4),textcoords='offset points',ha='center')
        ax.set_xticks(range(3),labels);ax.set_title(f'Cutoff {cutoff:g}');ax.set_ylabel('Failed request outcomes')
        if ax.get_legend_handles_labels()[0]:ax.legend(fontsize=8)
    for ext in ('png','pdf'):fig.savefig(figures/f'failure_stages.{ext}',dpi=200)
    plt.close(fig)
    fig,axes=plt.subplots(1,2,figsize=(11,4),layout='constrained')
    for ax,metric,title in zip(axes,('audit.min_world_clearance_m','curve.min_clearance_m'),('Minimum robot–world clearance','Minimum EE–cylinder clearance')):
        rows=[r for mode in MODES for r in results['secondary'] if r['mode']==mode and r['metric']==metric]
        if rows:
            d=np.array([r['difference'] for r in rows])*1000;ci=np.array([r['ci95'] for r in rows])*1000
            ax.errorbar(d,[MODES.index(r['mode']) for r in rows],xerr=np.maximum(0,[d-ci[:,0],ci[:,1]-d]),fmt='o',capsize=4)
        ax.axvline(0,ls=':',color='gray');ax.set_yticks(range(3),LABELS);ax.invert_yaxis();ax.set_title(title);ax.set_xlabel('New minus previous (mm)')
    fig.suptitle('Matched successful requests only; 95% environment-cluster intervals')
    for ext in ('png','pdf'):fig.savefig(figures/f'clearance_change.{ext}',dpi=200)
    plt.close(fig)
    lines=['# Covariance cutoff sensitivity','',results['note'],'',
        f'**Cutoff {before_cutoff:g} → {after_cutoff:g}**, keeping requested RL clearance 0.2/0.8 and original table/goal heights.',
        '', '| Method | Previous successes | Replay successes | Success change (pp), 95% CI | Holm p |','|---|---:|---:|---:|---:|']
    for r in results['primary']:lines.append(f'| {r["mode"]} | {r["before_successes"]}/{r["requests"]} | {r["after_successes"]}/{r["requests"]} | {100*r["difference"]:.2f} [{100*r["ci95"][0]:.2f}, {100*r["ci95"][1]:.2f}] | {r["holm_p"]:.6g} |')
    lines+=['', 'Raw counts pool different repeat counts; inference gives each source environment equal weight. Repeats stay within environment clusters. Three primary two-sided success tests have one Holm correction. Pointwise intervals can differ from corrected-test conclusions. All-success intervals do not establish perfect population reliability.',
        '', '![Success comparison](figures/success_comparison.png)', '', '| Method | Previous failure stages | Replay failure stages |', '|---|---|---|']
    for r in results['primary']:lines.append(f'| {r["mode"]} | {json.dumps(r["before_failures"])} | {json.dumps(r["after_failures"])} |')
    lines+=['', '![Failure stages](figures/failure_stages.png)', '', '![Matched-success clearance](figures/clearance_change.png)',
        '', 'Changing cutoff widens both the truncated proposal support and the enclosing hard box corridor. Covariance matrices and mixture weights do not change. The unrestricted baseline ignores cutoff, so its changes reflect stochastic rerun variability. No fallback, additional IK rescue, policy change or geometry change was introduced.',
        '', 'Secondary runtime/PAR2, matched-success path length and clearance effects, case-level failures, per-clearance/per-cohort counts and purity checks are in `statistics.json`. Timing drift and independent OMPL/IK RNG streams limit before/after runtime attribution. Conditional path-quality effects have survivor selection. See per-cohort reports for sampler stage/preparation times and distribution diagnostics.']
    (args.output/'summary.md').write_text('\n'.join(lines)+'\n')
    print(json.dumps(results['primary'],indent=2))

if __name__=='__main__':main()
