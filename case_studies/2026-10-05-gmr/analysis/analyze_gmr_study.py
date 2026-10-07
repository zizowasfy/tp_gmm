#!/usr/bin/env python3
"""Validate and summarize a completed strict GMR study, clustering by environment."""
import argparse
from collections import Counter
import csv
import hashlib
import itertools
import json
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from analyze_sampling_study import bootstrap_indices, sign_pvalue, interval, holm
from clearance_analysis import resample_polyline

MODES=('gmm','gmr_30mm','hybrid_30mm','reference_ik')
LABELS=dict(zip(MODES,('GMM','GMR 30 mm','Hybrid 30 mm','Reference IK')))
COLORS=('#0072B2','#D55E00','#009E73','#CC79A7')
QUALITY=('joint_path_length','ee_path_length_m','min_world_clearance_m','reference_deviation_mean_m')


def reference_distances(points,reference):
    """Exact Euclidean point-to-polyline-segment distance, bounded memory."""
    points=np.asarray(points,float);ref=np.asarray(reference,float)
    a,b=ref[:-1],ref[1:]
    v=b-a;den=np.einsum('ij,ij->i',v,v)
    keep=den>1e-20;a,v,den=a[keep],v[keep],den[keep]
    if not len(a):
        raise ValueError('Degenerate reference')
    distances=[]
    for offset in range(0,len(points),128):
        delta=points[offset:offset+128,None,:]-a[None,:,:]
        t=np.clip(np.einsum('ijk,jk->ij',delta,v)/den,0,1)
        distance=np.linalg.norm(delta-t[:,:,None]*v[None,:,:],axis=2)
        distances.extend(distance.min(axis=1))
    return np.asarray(distances)


def write_csv(path,rows):
    keys=list(dict.fromkeys(k for row in rows for k in row))
    with path.open('w') as file:
        writer=csv.DictWriter(file,fieldnames=keys);writer.writeheader();writer.writerows(rows)


def finite_stats(values):
    v=np.asarray([x for x in values if x is not None and np.isfinite(x)],float)
    return dict(n=len(v),mean=float(v.mean()) if len(v) else None,
                median=float(np.median(v)) if len(v) else None,p95=float(np.quantile(v,.95)) if len(v) else None)


def analyze(root):
    manifest=json.loads((root/'manifest.json').read_text())
    allrows=[json.loads(x) for x in (root/'results.jsonl').read_text().splitlines()]
    key=lambda r:(r['environment'],r['repeat'],r['mode'])
    if manifest['status']!='complete' or len(allrows)!=len(manifest['schedule']) or len({key(r) for r in allrows})!=len(allrows) or {key(r) for r in allrows}!={key(r) for r in manifest['schedule']}:
        raise ValueError('Study incomplete, duplicated or unexpected rows')
    if manifest['modes']!=list(MODES) or manifest['clearance']!=.7:
        raise ValueError('Unsupported study design')
    # Load purity checker only here so mathematical helpers/tests need no ROS.
    from run_gmr_study import assert_proposal_pure, digest
    inputs={}
    for env in manifest['environments']:
        content=json.loads((root/env['file']).read_text())
        if digest(content)!=env['sha256']:
            raise ValueError('Frozen environment changed')
        inputs[env['id']]=content
    enriched=[];endpoints=[]
    for row in allrows:
        if row['environment_sha256']!=manifest['environments'][row['environment']]['sha256']:
            raise ValueError('Mismatched paired input')
        verify=row['scene_verification']
        if not verify['verified'] or verify['actual_sha256']!=verify['expected_sha256']:
            raise ValueError('Unverified planning scene')
        if row['sampling']:
            assert_proposal_pure(row['mode'],row['sampling'])
        elif row['failure_stage']!='endpoint_ik':
            raise ValueError('Missing proposal report')
        expected_trial=next(t for t in manifest['schedule'] if key(t)==key(row))
        if row['warmup']!=expected_trial['warmup'] or row['seed']!=expected_trial['seed']:
            raise ValueError('Schedule mismatch')
        item={k:row[k] for k in ('environment','repeat','mode','cohort','layout','warmup','success','failure_stage','action_wall_s','planning_pipeline_wall_s','end_to_end_wall_s','par2_s')}
        item.update({k:row['sampling'].get(k) for k in ('attempts','valid_samples','gmm_attempts','gmr_attempts',
            'gmr_valid','uniform_attempts','online_ik_calls','online_ik_success','constraint_rejections','collision_rejections',
            'setup_s','sampling_s','draw_s','online_ik_s','validity_s')})
        item.update({k:row['pipeline'][k] for k in ('model_load_s','reproduction_s','rl_deformation_s','deformed_regression_s','total_s')})
        if row['success']:
            if row['audit']['audit_invalid_samples']!=0:
                raise ValueError('Invalid successful path')
            content=inputs[row['environment']]
            trajectory=row['trajectory']['joint_trajectory']
            errors=[]
            for state_key,point in zip(('start','goal'),(trajectory['points'][0],trajectory['points'][-1])):
                state=content[state_key]['joint_state'];q=dict(zip(state['name'],state['position']))
                errors.append(max(abs(x-q[n]) for n,x in zip(trajectory['joint_names'],point['positions'])))
            if max(errors)>.0011:
                raise ValueError(f'Wrong endpoint: {key(row)} / {errors}')
            endpoints.append(dict(environment=row['environment'],repeat=row['repeat'],mode=row['mode'],start_error_rad=errors[0],goal_error_rad=errors[1]))
            item.update({k:row['audit'][k] for k in QUALITY[:3]})
            item['ee_cylinder_clearance_m']=row['curve']['min_clearance_m']
            ref=np.array([[p['position'][a] for a in 'xyz'] for p in content['response']['deformed_trajectory']['poses']])
            # Near arc-uniform sampling at <=2 mm makes deviation comparable across planners.
            path=resample_polyline(row['audit']['ee_path'],.002)
            distances=reference_distances(path,ref)
            item.update(reference_deviation_mean_m=float(distances.mean()),reference_deviation_max_m=float(distances.max()))
        enriched.append(item)
    rows=[r for r in enriched if not r['warmup']]
    random_rows=[r for r in rows if r['cohort']=='randomized']
    envs=[e for e in manifest['environments'] if e['cohort']=='randomized']
    ids=[e['id'] for e in envs];strata=[e['layout'] for e in envs]
    lookup={key(r):r for r in random_rows};repeats=manifest['design']['repeats']
    rng=np.random.default_rng(20261006)
    boot=bootstrap_indices(strata,rng)
    cubes={metric:np.array([[[lookup[e,j,m][metric] for m in MODES] for j in range(repeats)] for e in ids],float)
           for metric in ('success','par2_s','action_wall_s','end_to_end_wall_s')}
    means={k:v.mean(axis=1) for k,v in cubes.items()}
    stats=dict(randomized_environments=len(ids),randomized_outcomes=len(random_rows),
        legacy_outcomes=sum(r['cohort']=='legacy' for r in rows),excluded_warmups=len(allrows)-len(rows),
        modes={},legacy={},primary=[],quality=[],purity=dict(uniform_attempts=sum(r['sampling'].get('uniform_attempts',0) for r in allrows),
            projected_attempts=sum(r['sampling'].get('projected_attempts',0) for r in allrows),
            fallback_enabled=manifest['runtime']['ompl.panda_arm.allow_constraint_sampler_fallback']['bool_value']))
    for i,m in enumerate(MODES):
        selected=[r for r in random_rows if r['mode']==m];success=[r for r in selected if r['success']]
        s=dict(n=len(selected),successes=len(success),success_rate=float(means['success'][:,i].mean()),
               success_ci95=interval(means['success'][boot,i].mean(axis=1)),failure_stages=dict(Counter(r['failure_stage'] for r in selected if not r['success'])))
        feasible=[r for r in selected if r['failure_stage']!='endpoint_ik']
        s['endpoint_feasible_n']=len(feasible)
        s['endpoint_feasible_success_rate']=sum(r['success'] for r in feasible)/len(feasible) if feasible else None
        for metric in ('par2_s','action_wall_s','end_to_end_wall_s'):
            s[metric+'_mean']=float(means[metric][:,i].mean());s[metric+'_ci95']=interval(means[metric][boot,i].mean(axis=1))
            s[metric+'_successful']=finite_stats([r[metric] for r in success])
        for metric in QUALITY+('ee_cylinder_clearance_m','reference_deviation_max_m'):
            s[metric]=finite_stats([r.get(metric) for r in success])
        s['attempts']=sum(r.get('attempts') or 0 for r in selected)
        s['valid_samples']=sum(r.get('valid_samples') or 0 for r in selected)
        s['acceptance']=s['valid_samples']/s['attempts'] if s['attempts'] else None
        s['gmr_attempts']=sum(r.get('gmr_attempts') or 0 for r in selected)
        s['gmm_attempts']=sum(r.get('gmm_attempts') or 0 for r in selected)
        for metric in ('setup_s','sampling_s','draw_s','online_ik_s','validity_s'):
            s[metric]=finite_stats([r.get(metric) for r in selected])
        stats['modes'][m]=s
        legacy=[r for r in rows if r['cohort']=='legacy' and r['mode']==m]
        stats['legacy'][m]=dict(n=len(legacy),successes=sum(r['success'] for r in legacy),
            by_scene={e['layout']:dict(n=sum(r['environment']==e['id'] for r in legacy),
                successes=sum(r['environment']==e['id'] and r['success'] for r in legacy)) for e in manifest['environments'] if e['cohort']=='legacy'})
    for a,b in itertools.combinations(range(4),2):
        for metric in ('success','par2_s'):
            diff=means[metric][:,a]-means[metric][:,b]
            stats['primary'].append(dict(a=MODES[a],b=MODES[b],metric=metric,difference=float(diff.mean()),
                ci95=interval(diff[boot].mean(axis=1)),p=sign_pvalue(diff,rng),environments=len(ids)))
    for row,p in zip(stats['primary'],holm([r['p'] for r in stats['primary']])):row['p_holm']=float(p)
    for a,b in itertools.combinations(MODES[:3],2):
        for metric in QUALITY:
            differences=[];groups=[];pairs=0
            for env in envs:
                values=[lookup[env['id'],j,a][metric]-lookup[env['id'],j,b][metric] for j in range(repeats)
                        if lookup[env['id'],j,a]['success'] and lookup[env['id'],j,b]['success']]
                if values:
                    differences.append(np.mean(values));groups.append(env['layout']);pairs+=len(values)
            if differences:
                diff=np.asarray(differences);indices=bootstrap_indices(groups,rng)
                stats['quality'].append(dict(a=a,b=b,metric=metric,difference=float(diff.mean()),
                    ci95=interval(diff[indices].mean(axis=1)),p=sign_pvalue(diff,rng),environments=len(diff),matched_repeats=pairs))
            else:
                stats['quality'].append(dict(a=a,b=b,metric=metric,difference=None,ci95=None,p=1.,environments=0,matched_repeats=0))
    for row,p in zip(stats['quality'],holm([r['p'] for r in stats['quality']])):row['p_holm']=float(p)
    output=root/'analysis';output.mkdir(exist_ok=True)
    (output/'statistics.json').write_text(json.dumps(stats,indent=2,allow_nan=False)+'\n')
    (output/'endpoint_audit.json').write_text(json.dumps(endpoints,indent=2)+'\n')
    write_csv(output/'trials.csv',enriched)
    method_rows=[]
    for mode,data in stats['modes'].items():
        item=dict(mode=mode)
        for k,v in data.items():
            if isinstance(v,dict):
                item.update({k+'_'+sub:value for sub,value in v.items() if value is None or isinstance(value,(int,float,str))})
            elif not isinstance(v,list):
                item[k]=v
        method_rows.append(item)
    write_csv(output/'method_summary.csv',method_rows)
    write_csv(output/'primary_contrasts.csv',[{k:v for k,v in r.items() if k!='ci95'}|dict(ci_low=r['ci95'][0],ci_high=r['ci95'][1]) for r in stats['primary']])
    write_csv(output/'quality_contrasts.csv',[{k:v for k,v in r.items() if k!='ci95'}|dict(ci_low=r['ci95'][0] if r['ci95'] else None,ci_high=r['ci95'][1] if r['ci95'] else None) for r in stats['quality']])
    write_csv(output/'reference_diagnostics.csv',[dict(environment=e['id'],cohort=e['cohort'],layout=e['layout'],endpoint_error=inputs[e['id']]['endpoint_error'])|inputs[e['id']]['reference_metrics'] for e in manifest['environments']])
    plot(output,stats,rows,random_rows,lookup,ids,repeats,inputs,manifest)
    report(output,stats,manifest,inputs)
    (output/'provenance.json').write_text(json.dumps(dict(manifest_sha256=hashlib.sha256((root/'manifest.json').read_bytes()).hexdigest(),
        results_sha256=hashlib.sha256((root/'results.jsonl').read_bytes()).hexdigest(),
        analysis_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()),indent=2)+'\n')
    return stats


def plot(output,stats,rows,random_rows,lookup,ids,repeats,inputs,manifest):
    plt.rcParams.update({'font.size':10,'axes.spines.top':False,'axes.spines.right':False,'savefig.bbox':'tight','pdf.fonttype':42})
    def save(name,fig):
        fig.savefig(output/(name+'.png'),dpi=200);fig.savefig(output/(name+'.pdf'));plt.close(fig)
    fig,axes=plt.subplots(1,3,figsize=(13.5,4.1))
    x=np.arange(4)
    for ax,metric,title,scale in [(axes[0],'success','Audited success (%)',100),(axes[1],'par2_s','Failure-penalized time, PAR2 (s)',1)]:
        values=[];cis=[]
        for m in MODES:
            s=stats['modes'][m];values.append(s['success_rate'] if metric=='success' else s[metric+'_mean'])
            cis.append(s['success_ci95'] if metric=='success' else s[metric+'_ci95'])
        y=np.array(values)*scale;ci=np.array(cis)*scale
        ax.bar(x,y,color=COLORS,width=.65);ax.errorbar(x,y,yerr=np.maximum(0,np.vstack([y-ci[:,0],ci[:,1]-y])),fmt='none',color='black',capsize=3)
        ax.set_xticks(x,[LABELS[m].replace(' ','\n',1) for m in MODES]);ax.set_title(title);ax.grid(axis='y',alpha=.2)
    axes[0].set_ylim(0,106)
    for m,c in zip(MODES,COLORS):
        values=sorted(r['action_wall_s'] for r in random_rows if r['mode']==m and r['success'])
        if values:axes[2].plot(values,np.arange(1,len(values)+1)/len(values),label=LABELS[m],color=c)
    axes[2].set(xlabel='Successful action wall time (s)',ylabel='Empirical cumulative fraction',xscale='log',title='Latency conditional on success')
    axes[2].legend(fontsize=8);fig.suptitle(f'{len(ids)} randomized environments × {repeats} repeats • clearance 0.7 • cutoffs 3.0',fontsize=12)
    fig.tight_layout();save('outcomes',fig)
    fig,axes=plt.subplots(2,2,figsize=(10,7))
    titles=['Joint path length (MoveIt distance, rad)','End-effector path length (m)','Minimum robot–world clearance (m)','Mean distance to DSGMR reference (m)']
    for ax,metric,title in zip(axes.flat,QUALITY,titles):
        data=[]
        for m in MODES[:3]:
            values=[]
            for e in ids:
                paired=[j for j in range(repeats) if all(lookup[e,j,k]['success'] for k in MODES[:3])]
                if paired:values.append(np.mean([lookup[e,j,m][metric] for j in paired]))
            data.append(values)
        boxes=ax.boxplot(data,tick_labels=[LABELS[m] for m in MODES[:3]],patch_artist=True,showfliers=False)
        for patch,c in zip(boxes['boxes'],COLORS):patch.set_facecolor(c);patch.set_alpha(.7)
        ax.set_title(title);ax.grid(axis='y',alpha=.2)
    fig.suptitle('Path quality on repeats successful for all three RRT proposals\nEach box uses environment means; reference IK shown in tables',fontsize=11)
    fig.tight_layout();save('path_quality',fig)
    fig,axes=plt.subplots(1,2,figsize=(11,4.5))
    stages=['setup_s','draw_s','online_ik_s','validity_s']
    width=.18
    for i,stage in enumerate(stages):
        axes[0].bar(np.arange(3)+(i-1.5)*width,[1000*stats['modes'][m][stage]['mean'] for m in MODES[:3]],width,label=stage.removesuffix('_s').replace('_',' '))
    axes[0].set_xticks(np.arange(3),[LABELS[m] for m in MODES[:3]]);axes[0].set(ylabel='Mean recorded time per request (ms)',title='Sampler components (not additive to total)');axes[0].legend(fontsize=8)
    axes[1].bar(np.arange(3),[100*stats['modes'][m]['acceptance'] if stats['modes'][m]['acceptance'] is not None else 0 for m in MODES[:3]],color=COLORS[:3])
    axes[1].set_xticks(np.arange(3),[LABELS[m] for m in MODES[:3]]);axes[1].set(ylabel='Valid targets / proposal attempts (%)',title='Sampling efficiency')
    fig.tight_layout();save('sampling_diagnostics',fig)
    fig,ax=plt.subplots(figsize=(10,4.5))
    failure_stages=sorted({r['failure_stage'] for r in random_rows if not r['success']})
    bottom=np.zeros(4)
    for stage in failure_stages:
        counts=np.array([stats['modes'][m]['failure_stages'].get(stage,0) for m in MODES])
        ax.bar(np.arange(4),counts,bottom=bottom,label=stage.replace('_',' '));bottom+=counts
    ax.set_xticks(np.arange(4),[LABELS[m] for m in MODES]);ax.set(ylabel='Failed requests',title='Failures retained in all denominators')
    if failure_stages:ax.legend(fontsize=8)
    fig.tight_layout();save('failure_stages',fig)
    # Fixed original scenes, first measured repeat: selection independent of outcome.
    source_rows=[json.loads(x) for x in (output.parent/'results.jsonl').read_text().splitlines()]
    fig,axes=plt.subplots(1,3,figsize=(13.5,4.5),subplot_kw={'projection':'3d'})
    for ax,e in zip(axes,[e for e in manifest['environments'] if e['cohort']=='legacy']):
        content=inputs[e['id']];case=content['case']
        ref=np.array([[p['position'][k] for k in 'xyz'] for p in content['response']['deformed_trajectory']['poses']])
        ax.plot(*ref.T,'k--',lw=1.3,label='DSGMR reference')
        angle=np.linspace(0,2*np.pi,40);z=np.linspace(case['obstacle'][2]-case['height']/2,case['obstacle'][2]+case['height']/2,2)
        angle,z=np.meshgrid(angle,z)
        ax.plot_surface(case['obstacle'][0]+case['radius']*np.cos(angle),case['obstacle'][1]+case['radius']*np.sin(angle),z,color='gray',alpha=.3)
        for m,c in zip(MODES,COLORS):
            row=next(r for r in source_rows if r['environment']==e['id'] and r['repeat']==0 and r['mode']==m)
            if row['success']:
                p=np.array(row['audit']['ee_path']);ax.plot(*p.T,color=c,lw=1.2,label=LABELS[m])
            else:ax.plot([],[],[],color=c,label=LABELS[m]+' (failed)')
        ax.scatter(*case['start'],color='black',marker='o');ax.scatter(*case['goal'],color='black',marker='*')
        ax.set(xlabel='x (m)',ylabel='y (m)',zlabel='z (m)',title=e['layout'].replace('_',' '));ax.view_init(26,-58)
    axes[0].legend(fontsize=7,loc='upper left');fig.suptitle('Original pilot scenes • first measured repeat • failed paths omitted',fontsize=11)
    fig.tight_layout();save('example_paths',fig)


def report(output,stats,manifest,inputs):
    lines=['# Strict GMM / GMR proposal comparison','',
        f"Completed {stats['randomized_outcomes']} measured outcomes in {stats['randomized_environments']} new randomized environments, plus {stats['legacy_outcomes']} measured outcomes on the three original pilot scenes. {stats['excluded_warmups']} warmups excluded.",'',
        '**Settings:** GMM cutoff 3.0; GMR radial cutoff 3.0; clearance input 0.7; GMR standard deviation 30 mm; hybrid 80% GMR / 20% GMM; Cartesian + IK mapping; RRTConnect, 3 s budget, no simplification. The reference is the frozen post-RL DSGMR rollout. Uniform mixture and MoveIt wrapper fallback are disabled.','',
        '| Method | Audited success | Environment-bootstrap 95% CI | Mean PAR2 (s) | Successful median action (ms) | Proposal acceptance |',
        '|---|---:|---:|---:|---:|---:|']
    for m in MODES:
        s=stats['modes'][m];ci=s['success_ci95'];median=s['action_wall_s_successful']['median']
        median_text=f'{median*1000:.1f}' if median is not None else 'n/a'
        acceptance_text=f"{s['acceptance']:.1%}" if s['acceptance'] is not None else 'n/a'
        lines.append(f"| {LABELS[m]} | {s['successes']}/{s['n']} ({s['success_rate']:.1%}) | {ci[0]:.1%}–{ci[1]:.1%} | {s['par2_s_mean']:.3f} | {median_text} | {acceptance_text} |")
    lines+=['','![Outcomes](outcomes.png)','','## Inference','',
        'Environment is the independent unit. Confidence intervals use 20,000 layout-stratified cluster bootstrap samples. Primary comparisons use paired environment sign permutations and Holm correction across all six method pairs × two metrics. PAR2 assigns 6 s to every failure; successful action time is capped at 3 s. Repeats are not treated as independent experimental units.','',
        '| Contrast (A − B) | Metric | Difference [95% CI] | Holm p |','|---|---|---:|---:|']
    for r in stats['primary']:
        scale=100 if r['metric']=='success' else 1;unit='pp' if scale==100 else 's'
        lines.append(f"| {LABELS[r['a']]} − {LABELS[r['b']]} | {r['metric']} | {r['difference']*scale:.3f} [{r['ci95'][0]*scale:.3f}, {r['ci95'][1]*scale:.3f}] {unit} | {r['p_holm']:.4g} |")
    lines+=['','## Path quality on paired successes','',
        'Each contrast uses repeats where both methods succeeded, averages the paired difference within each environment, then weights environments equally. These results are conditional on success and do not replace the reliability comparison. The separate 12-test secondary family covers three RRT proposal pairs × four quality metrics. Reference IK quality is descriptive.','',
        '| Contrast (A − B) | Metric | Mean difference [95% CI] | Matched repeats / environments | Holm p |','|---|---|---:|---:|---:|']
    for r in stats['quality']:
        if r['difference'] is not None:
            lines.append(f"| {LABELS[r['a']]} − {LABELS[r['b']]} | {r['metric']} | {r['difference']:.5f} [{r['ci95'][0]:.5f}, {r['ci95'][1]:.5f}] | {r['matched_repeats']} / {r['environments']} | {r['p_holm']:.4g} |")
    lines+=['','![Path quality](path_quality.png)','','## Descriptive path medians','',
        'These use each method’s successful requests; reference IK has a smaller conditioning set. Use the paired contrasts above for comparative inference.','',
        '| Method | Successful requests | Joint length (MoveIt distance, rad) | EE length (m) | World clearance (mm) | Mean reference deviation (mm) |',
        '|---|---:|---:|---:|---:|---:|']
    for m in MODES:
        s=stats['modes'][m]
        vals=[s[k]['median'] for k in QUALITY]
        texts=[f'{v*scale:.3f}' if v is not None else 'n/a' for v,scale in zip(vals,(1,1,1000,1000))]
        lines.append(f"| {LABELS[m]} | {s['successes']} | "+' | '.join(texts)+' |')
    lines+=['','## Failures and original-scene replication','']
    for m in MODES:
        s=stats['modes'][m];legacy=stats['legacy'][m]
        lines.append(f"- **{LABELS[m]}:** randomized failures `{json.dumps(s['failure_stages'],sort_keys=True)}`; original scenes {legacy['successes']}/{legacy['n']} successes.")
    lines+=['','![Failures](failure_stages.png)','','![Sampler diagnostics](sampling_diagnostics.png)','','![Example paths](example_paths.png)','',
        '## Interpretation and limits','',
        '- GMM explores the full truncated learned mixture. GMR concentrates its proposals around the reproduced reference, while the hybrid retains a broader GMM component. Accepted targets are also conditioned on IK feasibility, collisions and the hard GMM corridor; RRT vertices need not follow the proposal distribution.',
        '- Reference IK follows the rollout without search or detour repair and must connect the actual task endpoints. Its timing includes public IK/FK/validity-service calls and it returns an untimed geometric path. It is a feasibility baseline; runtime differences also reflect architecture. It is not an unrestricted OMPL baseline.',
        '- All four methods use identical frozen inputs and endpoints per environment. Endpoint IK failures count for every method; no candidate is replaced after observing difficulty. Inspect `reference_diagnostics.csv` for endpoint mismatch and cylinder intersections.',
        '- Mean reference deviation uses the whole audited EE polyline, resampled at at most 2 mm, and exact point-to-reference-segment distances. World clearance is robot-versus-world and excludes self-distance; EE-cylinder clearance is a separate point metric.',
        '- Clearance 0.7 is normalized RL input, not 0.7 m guaranteed separation. The original table and goal-height range are retained. These conclusions are limited to this distribution and checkpoint.',
        '- Deformation is run once per environment and its measured cost is attributed to each request. End-to-end figures are additive estimates, not repeated independent RL inference measurements. Offline audit and analysis time is excluded from planning timing.',
        '- Bootstrap intervals can collapse when every observed environment has the same outcome; this does not establish perfect population reliability. No equivalence claim follows from a nonsignificant difference.',
        '- Historical pilot results are not pooled: their outer MoveIt wrapper could silently fall back. This run checks the wrapper parameter, exhausted-sampler behavior, per-request source counters, scene readback and dense returned-path audits.','',
        f"Purity totals (including warmups): `{json.dumps(stats['purity'],sort_keys=True)}`.",'',
        'See `statistics.json`, `trials.csv`, contrast CSVs and `endpoint_audit.json` for complete values. Vector PDF versions accompany every figure. Raw JSONL, frozen inputs, source snapshots and protocol remain one directory above.']
    (output/'report.md').write_text('\n'.join(lines)+'\n')


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('directory',type=Path)
    args=parser.parse_args();stats=analyze(args.directory.resolve())
    print(json.dumps({m:{k:stats['modes'][m][k] for k in ('n','successes','success_rate','par2_s_mean','acceptance')} for m in MODES},indent=2))
