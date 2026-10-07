#!/usr/bin/env python3
"""Paired analysis of original scenes versus cloned lower-goal/lower-table scenes."""
import argparse
from collections import Counter
from copy import deepcopy
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from analyze_sampling_study import MODES,LABELS,COLORS,bootstrap_indices,interval,sign_pvalue,holm
from compare_clearance_replays import load


def verify_pair(control,training,intervention):
    if control['start']!=training['start']:raise ValueError('Start state changed')
    case=deepcopy(training['case'])
    low,high=intervention['source_goal_z_range'];new_low,new_high=intervention['training_goal_z_range']
    expected=new_low+(control['case']['goal'][2]-low)/(high-low)*(new_high-new_low)
    if not np.isclose(case['goal'][2],expected,atol=1e-12,rtol=0):raise ValueError('Goal-height mapping changed')
    case['goal'][2]=control['case']['goal'][2]
    if case!=control['case']:raise ValueError('Geometry other than goal height changed')
    scene=deepcopy(training['scene'])
    tables=[[o for o in s['world']['collision_objects'] if o['id']=='tpgmm_benchmark_table'] for s in (control['scene'],scene)]
    if any(len(t)!=1 for t in tables):raise ValueError('Table missing or duplicated')
    old,new=tables[0][0],tables[1][0]
    if not np.isclose(new['pose']['position']['z']+new['primitives'][0]['dimensions'][2]/2,intervention['table_top_z'],atol=1e-12,rtol=0):
        raise ValueError('Unexpected lowered-table surface')
    new['pose']['position']['z']=old['pose']['position']['z']
    if scene!=control['scene']:raise ValueError('Scene changed beyond table height')
    for key in ('policy_sha256','action_scale'):
        if control['response'][key]!=training['response'][key]:raise ValueError('Policy changed')


def summarize(cases,rng):
    if not cases:return []
    boot=bootstrap_indices([c['stratum'] for c in cases],rng)
    results=[]
    for mode in MODES:
        before=[r for c in cases for r in c['rows'][mode][0]];after=[r for c in cases for r in c['rows'][mode][1]]
        a=np.array([np.mean([r['success'] for r in c['rows'][mode][0]]) for c in cases])
        b=np.array([np.mean([r['success'] for r in c['rows'][mode][1]]) for c in cases]);d=b-a
        results.append(dict(mode=mode,environments=len(cases),requests=len(after),control_successes=sum(r['success'] for r in before),
            training_successes=sum(r['success'] for r in after),control_rate=float(a.mean()),training_rate=float(b.mean()),
            control_ci95=interval(a[boot].mean(axis=1)),training_ci95=interval(b[boot].mean(axis=1)),difference=float(d.mean()),
            ci95=interval(d[boot].mean(axis=1)),p=sign_pvalue(d,rng),
            control_failures=dict(Counter(r['failure_stage'] for r in before if not r['success'])),
            training_failures=dict(Counter(r['failure_stage'] for r in after if not r['success'])),
            control_action_requests=sum('error_code' in r for r in before),training_action_requests=sum('error_code' in r for r in after),
            recovered_pairs=sum(not a['success'] and b['success'] for a,b in zip(before,after)),
            lost_pairs=sum(a['success'] and not b['success'] for a,b in zip(before,after))))
    for c,p in zip(results,holm([c['p'] for c in results])):c['holm_p']=float(p)
    return results


def descriptive(cases):
    return {mode:{arm:dict(requests=sum(len(c['rows'][mode][i]) for c in cases),
        successes=sum(r['success'] for c in cases for r in c['rows'][mode][i])) for i,arm in enumerate(('control','training'))} for mode in MODES}


def sampling_diagnostics(cases):
    reports=[]
    counters=('attempts','valid_samples','uniform_attempts','missing_anchor_fallbacks','online_ik_calls',
              'anchor_ik_calls','anchor_ik_success','unanchored_components','collision_rejections',
              'projection_support_rejections','bounds_rejections','trust_region_rejections')
    timers=('setup_s','sampling_s','online_ik_s','anchor_ik_s','jacobian_setup_s','fk_mapping_s')
    for i,arm in enumerate(('control','training')):
        for mode in MODES[:2]:
            rows=[r for c in cases for r in c['rows'][mode][i]]
            for outcome in ('success','planning','path_audit'):
                selected=[r for r in rows if (r['success'] if outcome=='success' else r['failure_stage']==outcome)]
                if not selected:continue
                totals={k:sum(r['sampling'].get(k,0) for r in selected) for k in counters}
                measured=[r for r in selected if r['sampling'].get('allocated_instances',0)>0]
                reports.append(dict(arm=arm,mode=mode,outcome=outcome,requests=len(selected),totals=totals,
                    requests_with_valid_samples=sum(r['sampling'].get('valid_samples',0)>0 for r in selected),
                    requests_with_unanchored_components=sum(r['sampling'].get('unanchored_components',0)>0 for r in selected),
                    valid_per_attempt=totals['valid_samples']/totals['attempts'] if totals['attempts'] else None,
                    measured_sessions=len(measured),stage_medians_s={k:float(np.median([r['sampling'].get(k,0) for r in measured])) for k in timers} if measured else {}))
    return reports


def plot_arrangements(root,figures):
    """Fixed source environment zero, selected without looking at its outcomes."""
    fig=plt.figure(figsize=(12,6))
    fig.subplots_adjust(left=.01,right=.97,bottom=.15,top=.85,wspace=.02)
    for i,arm in enumerate(('control','training')):
        manifest=json.loads((root/arm/'manifest.json').read_text())
        frozen=json.loads((root/arm/manifest['environments'][0]['file']).read_text())
        case=frozen['case'];ax=fig.add_subplot(1,2,i+1,projection='3d')
        table=next(o for o in frozen['scene']['world']['collision_objects'] if o['id']=='tpgmm_benchmark_table')
        center=np.array([table['pose']['position'][k] for k in ('x','y','z')])
        half=np.array(table['primitives'][0]['dimensions'])/2
        vertices=np.array([[x,y,z] for z in (-1,1) for y in (-1,1) for x in (-1,1)])*half+center
        faces=[[vertices[j] for j in ix] for ix in ((0,1,3,2),(4,5,7,6),(0,1,5,4),(2,3,7,6),(0,2,6,4),(1,3,7,5))]
        ax.add_collection3d(Poly3DCollection(faces,facecolor='#AAAAAA',edgecolor='#777777',alpha=.12,linewidth=.5))
        theta,z=np.meshgrid(np.linspace(0,2*np.pi,40),[case['obstacle'][2]-case['height']/2,case['obstacle'][2]+case['height']/2])
        ax.plot_surface(case['obstacle'][0]+case['radius']*np.cos(theta),case['obstacle'][1]+case['radius']*np.sin(theta),z,color='#D55E00',alpha=.65)
        for key,label,color in (('original_trajectory','Prior GMR','#888888'),('deformed_trajectory','Deformed GMR','#0072B2')):
            points=np.array([[p['position'][k] for k in ('x','y','z')] for p in frozen['response'][key]['poses']])
            ax.plot(*points.T,label=label,color=color,lw=2)
        means=np.array([g['means'][-3:] for g in frozen['response']['deformed_gmm']['gaussians']])
        ax.scatter(*means.T,color='#0072B2',marker='x',label='Deformed means')
        ax.scatter(*case['start'],color='black',marker='o',s=30,label='Start')
        ax.scatter(*case['goal'],color='black',marker='*',s=80,label='Goal')
        ax.set(xlim=(.2,1.),ylim=(-.5,.5),zlim=(-.3,.8),xlabel='x (m)',ylabel='y (m)',zlabel='z (m)',
            title=f'{arm.capitalize()}: goal z={case["goal"][2]:.3f} m, table top={center[2]+half[2]:.2f} m')
        ax.set_box_aspect((.8,1.,1.1));ax.view_init(elev=20,azim=-55);ax.legend(fontsize=7,loc='upper left')
    fig.suptitle('Identical table dimensions and obstacle poses; separate scenes (fixed source environment 0)')
    for ext in ('png','pdf'):fig.savefig(figures/f'scene_arrangements.{ext}',dpi=200,bbox_inches='tight',pad_inches=.15)
    plt.close(fig)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directories',type=Path,nargs='+',help='Parent study directories, one per source cohort')
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();cases=[];verified=[]
    for phase,root in enumerate(args.directories):
        parent=json.loads((root/'manifest.json').read_text())
        if parent['status']!='complete':raise ValueError('Incomplete parent run')
        control,cr=load(root/'control');training,tr=load(root/'training')
        for key in ('settings','runtime','design','schedule'):
            if control[key]!=training[key]:raise ValueError('Changed experimental controls')
        if len(control['environments'])!=len(training['environments']):raise ValueError('Changed environment count')
        for a,b in zip(control['environments'],training['environments']):
            if a['id']!=b['id'] or a['clearance']!=b['clearance']:raise ValueError('Pair identity changed')
            e=a['id'];ca=json.loads((root/'control'/a['file']).read_text());cb=json.loads((root/'training'/b['file']).read_text())
            verify_pair(ca,cb,parent['intervention'])
            verified.append(dict(phase=phase,environment=e,only_goal_and_table_height_changed=True,
                control_sha256=a['sha256'],training_sha256=b['sha256']))
            rows={m:([cr[e,j,m] for j in range(control['design']['repeats'])],
                     [tr[e,j,m] for j in range(training['design']['repeats'])]) for m in MODES}
            cases.append(dict(phase=phase,environment=e,clearance=a['clearance'],stratum=f'{phase}/'+a['stratum'],rows=rows,
                endpoint_feasible=ca['endpoint_status']['feasible'] and cb['endpoint_status']['feasible'],
                control_goal_z=ca['case']['goal'][2],training_goal_z=cb['case']['goal'][2]))
    rng=np.random.default_rng(20260929)
    results=dict(environments=len(cases),primary=summarize(cases,rng),
        endpoint_feasible=summarize([c for c in cases if c['endpoint_feasible']],rng),secondary=[],
        sampling_diagnostics=sampling_diagnostics(cases),
        by_phase={str(p):descriptive([c for c in cases if c['phase']==p]) for p in range(len(args.directories))},
        by_clearance={str(q):descriptive([c for c in cases if c['clearance']==q]) for q in sorted({c['clearance'] for c in cases})},
        endpoint_failure_cases=[{k:c[k] for k in ('phase','environment','clearance','training_goal_z')} for c in cases if not c['endpoint_feasible']],
        note='Training-height arrangement minus original-height control; goal and table both change. Reused environments. Endpoint-solve failures are not proofs of unreachability.')
    for mode in MODES:
        for section,metric in [('audit','joint_path_length'),('audit','ee_path_length_m'),('audit','min_world_clearance_m'),('curve','min_clearance_m')]:
            values=[];strata=[];count=0
            for c in cases:
                pairs=[(a,b) for a,b in zip(*c['rows'][mode]) if a['success'] and b['success']]
                diffs=[b[section][metric]-a[section][metric] for a,b in pairs if a[section].get(metric) is not None and b[section].get(metric) is not None]
                if diffs:values.append(np.mean(diffs));strata.append(c['stratum']);count+=len(diffs)
            if values:
                d=np.array(values);boot=bootstrap_indices(strata,rng)
                results['secondary'].append(dict(mode=mode,metric=section+'.'+metric,difference=float(d.mean()),ci95=interval(d[boot].mean(axis=1)),environments=len(d),matched_pairs=count))
    args.output.mkdir(parents=True,exist_ok=True)
    (args.output/'statistics.json').write_text(json.dumps(results,indent=2)+'\n')
    (args.output/'input_verification.json').write_text(json.dumps(verified,indent=2)+'\n')
    figures=args.output/'figures';figures.mkdir(exist_ok=True)
    plt.rcParams.update({'font.size':10,'axes.spines.top':False,'axes.spines.right':False,'pdf.fonttype':42})
    plot_arrangements(args.directories[0],figures)
    fig,axes=plt.subplots(1,2,figsize=(12,4.5),layout='constrained')
    for ax,rows,title in zip(axes,[results['primary'],results['endpoint_feasible']],['All fixed environments','Both endpoint procedures passed']):
        for offset,arm,label,color in [(-.18,'control','Original goal/table','#777777'),(.18,'training','Lower goal/table','#0072B2')]:
            rates=np.array([r[arm+'_rate'] for r in rows])*100;ci=np.array([r[arm+'_ci95'] for r in rows])*100
            if len(rows):
                x=np.arange(len(rows))+offset;ax.bar(x,rates,width=.36,label=label,color=color)
                ax.errorbar(x,rates,yerr=np.maximum(0,np.array([rates-ci[:,0],ci[:,1]-rates])),fmt='none',color='black',capsize=3)
        ax.set_xticks(range(3),['Cartesian\n+ IK','Joint\nprojection','OMPL\nunrestricted']);ax.set_ylim(0,105)
        ax.set_ylabel('Environment-weighted success (%)');ax.set_title(title+f' (n={rows[0]["environments"] if rows else 0})');ax.legend(fontsize=8)
    fig.suptitle('Paired goal-height / table-height comparison; pointwise 95% cluster intervals')
    for ext in ('png','pdf'):fig.savefig(figures/f'success_comparison.{ext}',dpi=200)
    plt.close(fig)
    fig,ax=plt.subplots(figsize=(8,4),layout='constrained')
    v=np.array([r['difference'] for r in results['primary']])*100;ci=np.array([r['ci95'] for r in results['primary']])*100
    ax.errorbar(v,range(3),xerr=np.maximum(0,np.array([v-ci[:,0],ci[:,1]-v])),fmt='o',capsize=4)
    ax.axvline(0,color='gray',ls=':');ax.set_yticks(range(3),LABELS);ax.invert_yaxis()
    ax.set_xlabel('Lower-goal/lower-table minus original: success (pp)');ax.set_title('All fixed environments: paired differences and 95% intervals')
    for ext in ('png','pdf'):fig.savefig(figures/f'paired_success_change.{ext}',dpi=200)
    plt.close(fig)
    fig,axes=plt.subplots(1,2,figsize=(12,4.5),sharey=True,layout='constrained')
    categories=['goal_ik_design','start_invalid_design','goal_invalid_design','start_invalid_in_corridor','goal_invalid_in_corridor','planning','path_audit']
    colors=['#999999','#BBBBBB','#DDDDDD','#E69F00','#F0E442','#CC79A7','#D55E00']
    for ax,arm,label in zip(axes,('control','training'),('Original-height scene','Lower-goal/lower-table scene')):
        bottom=np.zeros(3)
        for stage,color in zip(categories,colors):
            vals=np.array([r[arm+'_failures'].get(stage,0) for r in results['primary']])
            if np.any(vals):ax.bar(range(3),vals,bottom=bottom,label=stage,color=color);bottom+=vals
        for i,total in enumerate(bottom):ax.annotate(str(int(total)),(i,total),xytext=(0,4),textcoords='offset points',ha='center',fontsize=9)
        ax.set_xticks(range(3),['Cartesian\n+ IK','Joint\nprojection','OMPL\nunrestricted']);ax.set_ylabel('Failed request outcomes');ax.set_title(label)
        if ax.get_legend_handles_labels()[0]:ax.legend(fontsize=7)
    for ext in ('png','pdf'):fig.savefig(figures/f'failure_stages.{ext}',dpi=200)
    plt.close(fig)
    lines=['# Goal-height / table-height sensitivity','',results['note'],'',
        '| Method | Original successes | Lower-goal/table successes | Success change (pp), 95% CI | Holm p |',
        '|---|---:|---:|---:|---:|']
    for r in results['primary']:
        lines.append(f'| {r["mode"]} | {r["control_successes"]}/{r["requests"]} | {r["training_successes"]}/{r["requests"]} | {100*r["difference"]:.2f} [{100*r["ci95"][0]:.2f}, {100*r["ci95"][1]:.2f}] | {r["holm_p"]:.6g} |')
    lines+=['','Inference gives each source environment equal weight; raw counts weight cohorts with different repeat counts differently. Primary Holm correction covers the three success contrasts. The conditional analysis below is secondary and uses only environments where both fixed endpoint procedures passed. It does not erase endpoint failures from the main analysis.',
        '',f'Endpoint procedures passed in both arrangements for {sum(c["endpoint_feasible"] for c in cases)}/{len(cases)} source environments.',
        '', '| Method | Original successes on endpoint-feasible subset | Lower arrangement successes |','|---|---:|---:|']
    for r in results['endpoint_feasible']:lines.append(f'| {r["mode"]} | {r["control_successes"]}/{r["requests"]} | {r["training_successes"]}/{r["requests"]} |')
    lines+=['','![Scene arrangements](figures/scene_arrangements.png)','', 'The fixed example shows GMR reference curves and Gaussian means, not planned paths. Cylinder poses are unchanged in these plan-only scenes; no gravity simulation is applied.',
        '', '![Success comparison](figures/success_comparison.png)','', '![Paired effect](figures/paired_success_change.png)','',
        '![Failure stages](figures/failure_stages.png)','',
        'The intervention changes table position and the geometric planning task as well as the goal-height distribution. It cannot attribute an effect uniquely to RL generalization or rule out every other cause. The checkpoint, clearance 0.5/0.7, cutoff 2.0 and strict sampling settings are unchanged. Timings/path lengths must be interpreted in light of changed geometry and survivor selection. Detailed matched-success effects, per-clearance/per-cohort counts and failure cases are in `statistics.json`.']
    (args.output/'summary.md').write_text('\n'.join(lines)+'\n')
    print(json.dumps(results['primary'],indent=2))

if __name__=='__main__':main()
