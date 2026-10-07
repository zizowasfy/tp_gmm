#!/usr/bin/env python3
"""Environment-cluster paired inference and publication figures for the pure sampler study."""
import argparse
import csv
import itertools
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

MODES = ('cartesian_ik','joint_projected','ompl_uniform')
LABELS = ('Cartesian + IK','Joint projection','OMPL unrestricted')
COLORS = ('#0072B2','#D55E00','#009E73')


def holm(values):
    order = np.argsort(values)
    adjusted = np.empty(len(values))
    previous = 0.
    for rank,index in enumerate(order):
        previous = max(previous,min(1.,(len(values)-rank)*values[index]))
        adjusted[index] = previous
    return adjusted


def bootstrap_indices(strata, rng, count=20000):
    groups = [np.flatnonzero(np.asarray(strata) == s) for s in sorted(set(strata))]
    return np.concatenate([rng.choice(g,size=(count,len(g)),replace=True) for g in groups],axis=1)


def sign_pvalue(difference, rng, count=100000, alternative="two-sided"):
    difference = np.asarray(difference)
    # Zero-difference clusters contribute no sign information. Enumerating the
    # informative clusters gives the exact same randomization distribution.
    if not np.any(difference):
        return 1.
    difference = difference[difference != 0]
    n = len(difference)
    if alternative not in ("two-sided", "greater"):
        raise ValueError("Unknown alternative")
    observed = abs(difference.mean()) if alternative == "two-sided" else difference.mean()
    if not np.any(difference):
        return 1.
    if n <= 16:
        signs = np.array(list(itertools.product((-1,1),repeat=n)))
        values = (signs*difference).mean(axis=1)
        return float(np.mean((abs(values) if alternative == "two-sided" else values) >= observed-1e-14))
    exceed = 0
    for offset in range(0,count,2000):
        signs = rng.choice([-1,1],size=(min(2000,count-offset),n))
        values = (signs*difference).mean(axis=1)
        exceed += np.sum((abs(values) if alternative == "two-sided" else values) >= observed-1e-14)
    return float((exceed+1)/(count+1))


def interval(values):
    return np.quantile(values,[.025,.975]).tolist()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory',type=Path)
    args=parser.parse_args()
    root=args.directory
    manifest=json.loads((root/'manifest.json').read_text())
    rows=[json.loads(s) for s in (root/'results.jsonl').read_text().splitlines()]
    if manifest['status'] != 'complete' or len(rows) != len(manifest['schedule']):
        raise ValueError('Analyze only a completed frozen study')
    n=len(manifest['environments']); repeats=manifest['design']['repeats']
    lookup={(r['environment'],r['repeat'],r['mode']):r for r in rows}
    expected={(e,j,m) for e in range(n) for j in range(repeats) for m in MODES}
    if set(lookup) != expected or len(lookup) != len(rows):
        raise ValueError('Incomplete, unexpected or duplicated trial keys')
    # Sanity-check that successful outputs solve the frozen endpoint problem.
    endpoint_checks=[]
    for e,environment in enumerate(manifest['environments']):
        content=json.loads((root/environment['file']).read_text())
        expected_states=[dict(zip(content[k]['joint_state']['name'],content[k]['joint_state']['position'])) for k in ('start','goal')]
        for j in range(repeats):
            for m in MODES:
                row=lookup[e,j,m]
                if not row['success']:
                    continue
                trajectory=row['trajectory']['joint_trajectory']
                errors=[]
                for point,expected_state in zip((trajectory['points'][0],trajectory['points'][-1]),expected_states):
                    errors.append(max(abs(q-expected_state[name]) for name,q in zip(trajectory['joint_names'],point['positions'])))
                if errors[0] > .0011 or errors[1] > .0011:
                    raise ValueError(f'Endpoint sanity audit failed: {e}/{j}/{m}: {errors}')
                endpoint_checks.append(dict(environment=e,repeat=j,mode=m,start_max_joint_error=errors[0],goal_max_joint_error=errors[1]))
    (root/'endpoint_audit.json').write_text(json.dumps(endpoint_checks,indent=2)+'\n')
    # Replays retain source IDs such as central_c0.2 even after clearance changes.
    # Keep the same grouping, but display the actual requested clearance.
    strata=[e['stratum'].split('_c')[0]+f'_c{e["clearance"]:g}' for e in manifest['environments']]
    rng=np.random.default_rng(79123)
    boot=bootstrap_indices(strata,rng)
    cube={k:np.array([[[lookup[e,j,m].get(k,False if k=='success' else 0) for m in MODES]
                      for j in range(repeats)] for e in range(n)],dtype=float)
          for k in ('success','par2_s','action_wall_s','moveit_planning_s','end_to_end_wall_s')}
    means={k:v.mean(axis=1) for k,v in cube.items()}
    results=dict(environments=n,repeats=repeats,plans=len(rows),design=manifest['design'],modes={},primary=[],quality=[])
    if 'replay' in manifest: results['replay']=manifest['replay']
    for i,mode in enumerate(MODES):
        subset=[r for r in rows if r['mode']==mode]
        successes=[r for r in subset if r['success']]
        item=dict(n=len(subset),successes=len(successes),success_rate=float(means['success'][:,i].mean()),
            endpoint_design_failures=sum(r.get('planning_not_attempted',False) for r in subset),
            action_requests=sum('error_code' in r for r in subset),
            success_ci95=interval(means['success'][boot,i].mean(axis=1)),
            failures_by_stage={s:sum(r.get('failure_stage')==s for r in subset) for s in sorted({r['failure_stage'] for r in subset if r.get('failure_stage')})},
            total_attempts=sum(r['sampling'].get('attempts',0) for r in subset),
            total_valid_samples=sum(r['sampling'].get('valid_samples',0) for r in subset),
            uniform_attempts=sum(r['sampling'].get('uniform_attempts',0) for r in subset),
            online_ik_calls=sum(r['sampling'].get('online_ik_calls',0) for r in subset),
            missing_anchor_fallbacks=sum(r['sampling'].get('missing_anchor_fallbacks',0) for r in subset))
        for key in ('par2_s','action_wall_s','moveit_planning_s','end_to_end_wall_s'):
            item[key+'_mean']=float(means[key][:,i].mean())
            item[key+'_ci95']=interval(means[key][boot,i].mean(axis=1))
            values=[r[key] for r in successes]
            item[key+'_successful_median']=float(np.median(values)) if values else None
            item[key+'_successful_p95']=float(np.quantile(values,.95)) if values else None
        item['valid_per_attempt']=item['total_valid_samples']/item['total_attempts'] if item['total_attempts'] else None
        for stage in ('setup_s','sampling_s','draw_s','online_ik_s','anchor_ik_s','jacobian_setup_s','fk_mapping_s','validity_s'):
            item[stage+'_median']=float(np.median([r['sampling'].get(stage,0) for r in subset])) if mode!='ompl_uniform' else None
        for metric in ('joint_path_length','ee_path_length_m','min_world_clearance_m','trajectory_duration_s'):
            values=[r['audit'][metric] for r in successes if r['audit'].get(metric) is not None]
            item[metric+'_median']=float(np.median(values)) if values else None
        results['modes'][mode]=item
    for a,b in itertools.combinations(range(3),2):
        for metric in ('success','par2_s'):
            diff=means[metric][:,a]-means[metric][:,b]
            results['primary'].append(dict(a=MODES[a],b=MODES[b],metric=metric,difference=float(diff.mean()),
                ci95=interval(diff[boot].mean(axis=1)),p=sign_pvalue(diff,rng),independent_environments=n))
    for contrast,p in zip(results['primary'],holm([c['p'] for c in results['primary']])):
        contrast['holm_p']=float(p)
    for a,b in itertools.combinations(range(3),2):
        for metric in ('joint_path_length','ee_path_length_m','min_world_clearance_m'):
            environment_diffs=[]; ids=[]; matched=0
            for e in range(n):
                diffs=[]
                for j in range(repeats):
                    ra,rb=lookup[e,j,MODES[a]],lookup[e,j,MODES[b]]
                    if ra['success'] and rb['success']:
                        va,vb=ra['audit'].get(metric),rb['audit'].get(metric)
                        if va is not None and vb is not None:
                            diffs.append(va-vb); matched+=1
                if diffs:
                    environment_diffs.append(np.mean(diffs)); ids.append(e)
            if environment_diffs:
                d=np.array(environment_diffs)
                matched_boot=bootstrap_indices([strata[e] for e in ids],rng)
                results['quality'].append(dict(a=MODES[a],b=MODES[b],metric=metric,difference=float(d.mean()),
                    ci95=interval(d[matched_boot].mean(axis=1)),environments=len(ids),matched_success_pairs=matched))
    if manifest.get('study_phase') == 'path_quality_confirmation':
        results['confirmation_primary']=[]
        for metric in ('joint_path_length','ee_path_length_m'):
            comparison=next((c.copy() for c in results['quality'] if c['a']=='cartesian_ik' and c['b']=='joint_projected' and c['metric']==metric),None)
            if comparison is None:
                results.setdefault('confirmation_unavailable',[]).append(metric+': no matched successful pairs')
                continue
            d=[]
            for e in range(n):
                matched=[lookup[e,j,'cartesian_ik']['audit'][metric]-lookup[e,j,'joint_projected']['audit'][metric]
                         for j in range(repeats) if lookup[e,j,'cartesian_ik']['success'] and lookup[e,j,'joint_projected']['success']]
                if matched: d.append(np.mean(matched))
            comparison['p_one_sided']=sign_pvalue(d,np.random.default_rng(68272),alternative='greater')
            results['confirmation_primary'].append(comparison)
        for c,p in zip(results['confirmation_primary'],holm([c['p_one_sided'] for c in results['confirmation_primary']])):
            c['holm_p']=float(p)
    frozen_inputs=[json.loads((root/e['file']).read_text()) for e in manifest['environments']]
    results['preparation']={key:dict(median=float(np.median([v['pipeline'][key] for v in frozen_inputs])),
        mean=float(np.mean([v['pipeline'][key] for v in frozen_inputs])))
        for key in ('model_load_s','reproduction_s','rl_deformation_s','deformed_regression_s','total_s')}
    results['strata']={s:{m:dict(successes=sum(lookup[e,j,m]['success'] for e in range(n) if strata[e]==s for j in range(repeats)),
         plans=sum(x==s for x in strata)*repeats) for m in MODES} for s in sorted(set(strata))}
    ratio=means['par2_s'][:,0].mean()/means['par2_s'][:,1].mean()
    ratio_boot=means['par2_s'][boot,0].mean(axis=1)/means['par2_s'][boot,1].mean(axis=1)
    results['cartesian_over_projected_par2_ratio']=dict(estimate=float(ratio),ci95=interval(ratio_boot))
    (root/'statistics.json').write_text(json.dumps(results,indent=2)+'\n')
    fields=['environment','stratum','repeat','mode','seed','success','planner_success','failure_stage','error_code',
            'action_wall_s','moveit_planning_s','par2_s','end_to_end_wall_s']
    with (root/'trials.csv').open('w') as output:
        writer=csv.DictWriter(output,fieldnames=fields); writer.writeheader()
        for r in rows: writer.writerow({k:r.get(k) for k in fields})
    from sampling_fidelity import distribution_report
    models={}
    for environment in manifest['environments']:
        content=json.loads((root/environment['file']).read_text())
        models[environment['sha256']]=content['response']['deformed_gmm']
    diagnostics=distribution_report([r | {'model_sha256':r['environment_sha256']} for r in rows],
        dict(models=models,modes=MODES[:2],settings=manifest['settings']))
    (root/'distribution_diagnostics.json').write_text(json.dumps(diagnostics,indent=2)+'\n')
    figures=root/'figures'; figures.mkdir(exist_ok=True)
    plt.rcParams.update({'font.size':10,'axes.spines.top':False,'axes.spines.right':False,'pdf.fonttype':42})
    fig,axes=plt.subplots(1,3,figsize=(12,3.8),layout='constrained')
    for ax,metric,title in zip(axes,('success','par2_s','end_to_end_wall_s'),('Audited success (%)','Failure-penalized time (s)','Post-design pipeline wall time (s)')):
        scale=100 if metric=='success' else 1
        values=means[metric].mean(axis=0)*scale
        ci=np.array([interval(means[metric][boot,i].mean(axis=1)*scale) for i in range(3)])
        ax.bar(range(3),values,color=COLORS,alpha=.85)
        ax.errorbar(range(3),values,yerr=np.maximum(0,np.array([values-ci[:,0],ci[:,1]-values])),fmt='none',color='black',capsize=4)
        ax.set_xticks(range(3),['Cartesian\n+ IK','Joint\nprojection','OMPL\nunrestricted'])
        ax.set_title(title); ax.grid(axis='y',alpha=.2); ax.set_axisbelow(True)
        if metric=='success': ax.set_ylim(0,105)
    fig.suptitle(f'{n} environments × {repeats} repeats; 95% environment-cluster intervals')
    for ext in ('png','pdf'): fig.savefig(figures/f'primary_outcomes.{ext}',dpi=200)
    plt.close(fig)
    fig,axes=plt.subplots(1,2,figsize=(9,4),layout='constrained')
    for ax,metric,label in zip(axes,('success','par2_s'),('Success fraction','PAR2 (s)')):
        for s in sorted(set(strata)):
            ix=np.array(strata)==s
            ax.scatter(means[metric][ix,0],means[metric][ix,1],label=s,s=25,alpha=.7)
        lo,hi=0,max(means[metric][:,:2].max()*1.05,.01)
        ax.plot([lo,hi],[lo,hi],':',color='gray'); ax.set_xlim(lo,hi); ax.set_ylim(lo,hi)
        ax.set_xlabel('Cartesian + IK: '+label); ax.set_ylabel('Joint projection: '+label)
    axes[1].legend(fontsize=7,loc='best')
    for ext in ('png','pdf'): fig.savefig(figures/f'paired_environments.{ext}',dpi=200)
    plt.close(fig)
    fig,axes=plt.subplots(1,3,figsize=(12,3.6),layout='constrained')
    for ax,metric,label in zip(axes,('joint_path_length','ee_path_length_m','min_world_clearance_m'),('Joint travel: sum |Δq| (rad)','EE path length (m)','Minimum robot–world clearance (m)')):
        data=[[r['audit'][metric] for r in rows if r['mode']==m and r['success'] and r['audit'].get(metric) is not None] for m in MODES]
        parts=ax.boxplot(data,patch_artist=True,showfliers=False)
        for patch,color in zip(parts['boxes'],COLORS): patch.set_facecolor(color); patch.set_alpha(.7)
        ax.set_xticks([1,2,3],['Cartesian\n+ IK','Joint\nprojection','OMPL\nunrestricted']); ax.set_title(label)
    fig.suptitle('Successful plans only; survivor selection applies')
    for ext in ('png','pdf'): fig.savefig(figures/f'path_quality.{ext}',dpi=200)
    plt.close(fig)
    # Fixed representative example: environment 0, repeat 0, never selected by outcome.
    frozen=json.loads((root/manifest['environments'][0]['file']).read_text())
    case=frozen['case']
    fig=plt.figure(figsize=(13,4.5),layout='constrained')
    u,v=np.mgrid[0:2*np.pi:24j,0:np.pi:14j]
    sphere=np.array([np.cos(u)*np.sin(v),np.sin(u)*np.sin(v),np.cos(v)])
    for i,mode in enumerate(MODES):
        ax=fig.add_subplot(1,3,i+1,projection='3d')
        for gaussian in frozen['response']['deformed_gmm']['gaussians']:
            mu=np.array(gaussian['means'])[-3:]
            dim=len(gaussian['means'])
            covariance=np.array(gaussian['covariances']).reshape(dim,dim)[-3:,-3:]
            values,vectors=np.linalg.eigh(covariance)
            points=(vectors@np.diag(2*np.sqrt(np.maximum(values,1e-8)))@sphere.reshape(3,-1)).reshape(sphere.shape)+mu[:,None,None]
            ax.plot_surface(*points,color='#56B4E9',alpha=.07,linewidth=0)
        theta=np.linspace(0,2*np.pi,32)
        theta,z=np.meshgrid(theta,[case['obstacle'][2]-case['height']/2,case['obstacle'][2]+case['height']/2])
        ax.plot_surface(case['obstacle'][0]+case['radius']*np.cos(theta),case['obstacle'][1]+case['radius']*np.sin(theta),z,color='gray',alpha=.5)
        row=lookup[0,0,mode]
        if row['success']:
            path=np.array(row['audit']['ee_path'])
            ax.plot(*path.T,color=COLORS[i],lw=2)
        ax.scatter(*case['start'],marker='o',color='black',s=25)
        ax.scatter(*case['goal'],marker='*',color='black',s=45)
        ax.set(xlabel='x (m)',ylabel='y (m)',zlabel='z (m)',title=LABELS[i]+('' if row['success'] else ' (failed)'))
        ax.view_init(elev=25,azim=-60)
    # Shared extents prevent visually shrinking a long baseline path.
    allpoints=np.array([x for mode in MODES for x in lookup[0,0,mode].get('audit',{}).get('ee_path',[])]+[case['start'],case['goal']])
    low=allpoints.min(axis=0)-.04; high=allpoints.max(axis=0)+.04
    for ax in fig.axes:
        ax.set_xlim(low[0],high[0]); ax.set_ylim(low[1],high[1]); ax.set_zlim(min(.25,low[2]),high[2])
    fig.suptitle('Fixed example: environment 0, repeat 0; deformed GMM at cutoff 2.0')
    for ext in ('png','pdf'): fig.savefig(figures/f'representative_paths.{ext}',dpi=200)
    plt.close(fig)
    stage_keys=('setup_s','sampling_s')
    stage_values=np.array([[np.mean([r['sampling'].get(k,0) for r in rows if r['mode']==m]) for m in MODES] for k in stage_keys])
    action=np.array([results['modes'][m]['action_wall_s_mean'] for m in MODES])
    fig,ax=plt.subplots(figsize=(7,4),layout='constrained')
    bottom=np.zeros(3)
    for values,label,color in zip([*stage_values,np.maximum(0,action-stage_values.sum(axis=0))],
            ['Sampler setup (includes anchor IK)','Sampling loop (includes online IK/FK)','Remaining action wall time'],['#CC79A7','#56B4E9','#999999']):
        ax.bar(range(3),values,bottom=bottom,label=label,color=color); bottom+=values
    ax.set_xticks(range(3),LABELS); ax.set_ylabel('Mean action wall time (s)'); ax.legend(fontsize=8)
    for ext in ('png','pdf'): fig.savefig(figures/f'timing_breakdown.{ext}',dpi=200)
    plt.close(fig)
    def f(value): return '—' if value is None else f'{value:.4f}'
    replay_label='goal/table-height sensitivity' if manifest.get('replay',{}).get('kind')=='goal_height' else 'clearance sensitivity'
    lines=['# Pure sampling case study results','',f'{n} source environments, {repeats} repeats per method, {len(rows)} request outcomes; fixed {manifest["design"]["planning_time"]:g} s planning budget.',
        '', '| Method | Audited success | Cluster 95% CI | Mean PAR2 (s) | Success median action (s) | Median EE length (m) | Median world clearance (m) |',
        '|---|---:|---:|---:|---:|---:|---:|']
    for m in MODES:
        v=results['modes'][m]
        lines.append(f'| {m} | {v["successes"]}/{v["n"]} ({100*v["success_rate"]:.1f}%) | {100*v["success_ci95"][0]:.1f}–{100*v["success_ci95"][1]:.1f}% | {v["par2_s_mean"]:.4f} | {f(v["action_wall_s_successful_median"])} | {f(v["ee_path_length_m_median"])} | {f(v["min_world_clearance_m_median"])} |')
    lines+=['',('Paired secondary contrasts' if manifest.get('study_phase') == 'path_quality_confirmation' else 'Paired primary contrasts')+' (first minus second; negative time favors first):','',
        '| First / second | Outcome | Difference | Cluster 95% CI | Holm p |','|---|---|---:|---:|---:|']
    for c in results['primary']:
        scale=100 if c['metric']=='success' else 1
        lines.append(f'| {c["a"]} / {c["b"]} | {"success (pp)" if scale==100 else "PAR2 (s)"} | {c["difference"]*scale:.4f} | [{c["ci95"][0]*scale:.4f}, {c["ci95"][1]*scale:.4f}] | {c["holm_p"]:.6g} |')
    lines+=['','All three pairwise success/PAR2 tests belong to one Holm family. Intervals are pointwise, not multiplicity-adjusted. Zero-width intervals in all-success samples do not prove perfect population reliability.',
        '', 'Pipeline times start after scene and endpoint design. They exclude the shared endpoint IK solve and offline dense path audit. Model-preparation wall time is included for custom methods, while the unrestricted baseline needs no deformation.',
        '', 'Sampler stage medians in seconds (setup and sampling contain the nested timers):','',
        '| Method | Setup | Draw loop | Online IK | Anchor IK | Jacobian | FK mapping | Valid / attempted |',
        '|---|---:|---:|---:|---:|---:|---:|---:|']
    for m in MODES:
        v=results['modes'][m]
        lines.append('| '+m+' | '+' | '.join(f(v.get(k)) for k in ('setup_s_median','sampling_s_median','online_ik_s_median','anchor_ik_s_median','jacobian_setup_s_median','fk_mapping_s_median','valid_per_attempt'))+' |')
    lines+=['', 'Shared model-preparation stage times (one measurement per environment; seconds):', '',
        '| Stage | Mean | Median |','|---|---:|---:|']
    for stage,values in results['preparation'].items():
        lines.append(f'| {stage} | {values["mean"]:.6f} | {values["median"]:.6f} |')
    lines+=['','Pure sampler counters: zero internal uniform attempts and missing-anchor Cartesian rescues are required. The separate strict runtime regression verifies the MoveIt wrapper failure path. The unrestricted baseline has no GMM session/counters.',
        '', (f'This is a {replay_label} replay of previously used environments, not new independent confirmation. All path-quality results below are conditional on success.' if 'replay' in manifest else 'Path quality is conditional on success; only the prespecified confirmation endpoints have confirmatory status.')+' Matched-pair effects and retained sample counts are in `statistics.json`. Minimum world clearance includes the table and all robot links; it is not the RL reward or only an EE-to-cylinder distance.',
        '', 'The custom hard corridor is a union of oriented boxes enclosing the cutoff ellipsoids; accepted targets obey the ellipsoidal cutoff, while tree edges may enter box corners. The baseline has no GMM corridor, so its contrasts are between different feasible-region configurations. Projected proposals only approximate the Cartesian Gaussian under a local linearization. The two custom methods also differ in proposal orientation behavior.',
        '', 'Raw inputs and outputs: `manifest.json`, `environments/*.json`, `results.jsonl`, `trials.csv`. Primary figures are supplied as PDF and PNG. No result from this study alone establishes transfer to EV battery disassembly.']
    if 'confirmation_primary' in results:
        lines+=['', ('## Replayed confirmation cohort (sensitivity analysis)' if 'replay' in manifest else '## Prespecified independent confirmation'), '',
            (f'This rerun reuses the original confirmation environments for {replay_label}, so it is not another independent confirmation. The original directional path-length tests are reproduced below as sensitivity analyses.' if 'replay' in manifest else 'This phase tests the exploratory path-quality observation on new environments. Its primary family comprises the two directional path-length tests below; the success/PAR2 tests above are secondary in this phase.'), '',
            '| Outcome (Cartesian minus projected) | Effect | 95% cluster CI | One-sided Holm p | Matched pairs / environments |',
            '|---|---:|---:|---:|---:|']
        for c in results['confirmation_primary']:
            lines.append(f'| {c["metric"]} | {c["difference"]:.5f} | [{c["ci95"][0]:.5f}, {c["ci95"][1]:.5f}] | {c["holm_p"]:.6g} | {c["matched_success_pairs"]} / {c["environments"]} |')
    if any(r.get('planning_not_attempted') for r in rows):
        lines+=['','Endpoint-design failures remain in the full denominator and receive the failure penalty. No planning action or sampler was called for these records. Their zero action/stage durations mean not attempted, not fast planning. Shared endpoint IK preparation is outside planning timers. Conditional results on environments that passed endpoint solving are supplied by the paired goal-height analyzer.']
    (root/'summary.md').write_text('\n'.join(lines)+'\n')
    print(json.dumps({m:{k:results['modes'][m][k] for k in ('successes','n','par2_s_mean')} for m in MODES},indent=2))


if __name__=='__main__': main()
