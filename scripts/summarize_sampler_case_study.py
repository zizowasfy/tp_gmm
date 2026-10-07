#!/usr/bin/env python3
"""Compare samplers at one fixed configuration, with equal environment weights.

This retrospective analysis uses no before/after observations. All 24 pairwise
tests (three method pairs, eight outcomes) share one Holm correction.
"""
import argparse
from collections import Counter
import csv
import hashlib
import itertools
import json
from pathlib import Path
import shutil

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from analyze_sampling_study import MODES, LABELS, COLORS, bootstrap_indices, interval, sign_pvalue, holm
from compare_clearance_replays import load

PERFORMANCE = ('success', 'par2_s', 'action_wall_s', 'end_to_end_wall_s')
QUALITY = ('audit.joint_path_length', 'audit.ee_path_length_m',
           'audit.min_world_clearance_m', 'curve.min_clearance_m')
STAGES = ('setup_s', 'sampling_s', 'draw_s', 'online_ik_s', 'anchor_ik_s',
          'jacobian_setup_s', 'fk_mapping_s', 'validity_s')
COUNTERS = ('attempts', 'valid_samples', 'uniform_attempts', 'missing_anchor_fallbacks',
            'online_ik_calls', 'anchor_ik_calls', 'trust_region_rejections',
            'projection_support_rejections', 'missing_anchor_rejections')


def metric(row, key):
    value = row
    for part in key.split('.'):
        value = value.get(part) if isinstance(value, dict) else None
    if value is None or not np.isfinite(value):
        raise ValueError('Missing/non-finite metric: '+key)
    return float(value)


def environment_values(cases, mode, key):
    values, strata, requests = [], [], 0
    for case in cases:
        rows = [r for r in case['rows'][mode] if key not in QUALITY or r['success']]
        if rows:
            values.append(np.mean([metric(r, key) for r in rows]))
            strata.append(case['stratum']); requests += len(rows)
    return np.asarray(values), strata, requests


def paired_values(cases, first, second, key):
    values, strata, requests = [], [], 0
    for case in cases:
        left, right = case['rows'][first], case['rows'][second]
        if len(left) != len(right):
            raise ValueError('Unpaired repeats')
        if any(a['repeat'] != b['repeat'] for a, b in zip(left, right)):
            raise ValueError('Repeat order mismatch')
        pairs = [(a, b) for a, b in zip(left, right)
                 if key not in QUALITY or (a['success'] and b['success'])]
        if pairs:
            values.append(np.mean([metric(a, key)-metric(b, key) for a, b in pairs]))
            strata.append(case['stratum']); requests += len(pairs)
    return np.asarray(values), strata, requests


def estimate(values, strata, rng):
    if not len(values):
        raise ValueError('No eligible environments')
    boot = bootstrap_indices(strata, rng)
    return dict(mean=float(np.mean(values)), ci95=interval(values[boot].mean(axis=1)),
                environments=len(values))


def verify_configuration(manifest, reference=None):
    settings = manifest['settings']
    if tuple(manifest['modes']) != MODES or settings['proposal'] != 'gmm' or settings['corridor_mode'] != 'covariance':
        raise ValueError('Expected two pure GMM samplers and unrestricted OMPL')
    if settings['uniform_fraction'] or settings['cartesian_fraction']:
        raise ValueError('Mixed sampling configuration')
    if manifest['runtime']['ompl.panda_arm.allow_constraint_sampler_fallback']['bool_value']:
        raise ValueError('Default sampler fallback enabled')
    if reference:
        for key in ('settings', 'runtime', 'policy_sha256', 'environment_ranges'):
            if manifest[key] != reference[key]:
                raise ValueError('Different cohort configuration: '+key)
        if manifest['design']['planning_time'] != reference['design']['planning_time']:
            raise ValueError('Different planning budget')


def verify_row(row, cutoff):
    stats = row['sampling']; mode = row['mode']
    if mode == 'ompl_uniform':
        if stats:
            raise ValueError('Unrestricted baseline used GMM plugin')
    else:
        forbidden = ('uniform_attempts', 'missing_anchor_fallbacks', 'gmr_attempts',
                     'online_ik_calls' if mode == 'joint_projected' else 'projected_attempts')
        if any(stats.get(key, 0) for key in forbidden):
            raise ValueError('Sampler purity violation')
        if (stats.get('proposal') != 'gmm' or stats.get('corridor_mode') != 'covariance'
                or not stats.get('component_cutoffs')
                or any(c != cutoff for c in stats['component_cutoffs'])):
            raise ValueError('Unexpected proposal/cutoff')
    audited = row.get('audit')
    if bool(row['success']) != bool(row['planner_success'] and audited and audited['audit_invalid_samples'] == 0):
        raise ValueError('Success differs from dense path audit')
    for key in PERFORMANCE + (QUALITY if row['success'] else ()):
        metric(row, key)


def analyze(cases, rng):
    result = dict(environments=len(cases), requests=sum(len(c['rows'][m]) for c in cases for m in MODES),
                  modes={}, contrasts=[], cohorts={}, strata={})
    for mode in MODES:
        rows = [r for c in cases for r in c['rows'][mode]]
        successes = [r for r in rows if r['success']]
        entry = dict(requests=len(rows), successes=len(successes), pooled_success_rate=len(successes)/len(rows),
                     failure_stages=dict(Counter(r['failure_stage'] for r in rows if not r['success'])),
                     metrics={}, stages={}, counters={})
        for key in PERFORMANCE + QUALITY:
            values, strata, count = environment_values(cases, mode, key)
            entry['metrics'][key] = estimate(values, strata, rng) | dict(requests=count, success_only=key in QUALITY)
        entry['successful_request_medians'] = {key:float(np.median([metric(r,key) for r in successes]))
                                               for key in PERFORMANCE[1:] + QUALITY}
        for key in STAGES:
            entry['stages'][key] = (dict(median_s=float(np.median([r['sampling'].get(key,0.) for r in rows])),
                mean_s=float(np.mean([np.mean([r['sampling'].get(key,0.) for r in c['rows'][mode]]) for c in cases])))
                if mode != 'ompl_uniform' else None)
        if mode != 'ompl_uniform':
            entry['counters'] = {key:sum(r['sampling'].get(key,0) for r in rows) for key in COUNTERS}
            ratios, strata = [], []
            for c in cases:
                attempts = sum(r['sampling'].get('attempts',0) for r in c['rows'][mode])
                if attempts:
                    ratios.append(sum(r['sampling'].get('valid_samples',0) for r in c['rows'][mode])/attempts)
                    strata.append(c['stratum'])
            entry['acceptance'] = estimate(np.asarray(ratios),strata,rng)
            entry['acceptance']['aggregate_ratio'] = entry['counters']['valid_samples']/entry['counters']['attempts']
        result['modes'][mode] = entry
    for first, second in itertools.combinations(MODES,2):
        for key in PERFORMANCE + QUALITY:
            values,strata,count = paired_values(cases,first,second,key)
            result['contrasts'].append(dict(first=first,second=second,metric=key,
                **estimate(values,strata,rng),p=sign_pvalue(values,rng),matched_requests=count,success_only=key in QUALITY))
    for contrast,p in zip(result['contrasts'],holm([r['p'] for r in result['contrasts']])):
        contrast['holm_p'] = float(p)
    for field,target in (('cohort','cohorts'),('design_stratum','strata')):
        for value in sorted({c[field] for c in cases}):
            selection = [c for c in cases if c[field] == value]
            result[target][value] = {}
            for mode in MODES:
                rows = [r for c in selection for r in c['rows'][mode]]
                result[target][value][mode] = dict(requests=len(rows),successes=sum(r['success'] for r in rows),
                    means={k:float(environment_values(selection,mode,k)[0].mean()) for k in PERFORMANCE+QUALITY})
    result['preparation'] = {key:dict(mean_s=float(np.mean([c['pipeline'][key] for c in cases])),
        median_s=float(np.median([c['pipeline'][key] for c in cases]))) for key in
        ('model_load_s','reproduction_s','rl_deformation_s','deformed_regression_s','total_s')}
    return result


def export_tables(root, results, cases):
    tables=root/'tables';tables.mkdir(exist_ok=True)
    def write(name, rows):
        with (tables/name).open('w') as out:
            writer=csv.DictWriter(out,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
    write('pairwise_comparisons.csv',[{k:v for k,v in r.items() if k!='ci95'} | dict(ci_low=r['ci95'][0],ci_high=r['ci95'][1])
                                     for r in results['contrasts']])
    write('sampler_summary.csv',[dict(mode=m,requests=v['requests'],successes=v['successes'],
        pooled_success_rate=v['pooled_success_rate'],**{key.replace('.','_')+'_environment_mean':s['mean'] for key,s in v['metrics'].items()},
        **{key.replace('.','_')+'_successful_request_median':s for key,s in v['successful_request_medians'].items()})
        for m,v in results['modes'].items()])
    rows=[];environments=[]
    for c in cases:
        for m in MODES:
            values={key:environment_values([c],m,key)[0] for key in PERFORMANCE+QUALITY}
            environments.append(dict(cohort=c['cohort'],environment=c['environment'],stratum=c['design_stratum'],mode=m,
                **{key.replace('.','_'):float(v[0]) if len(v) else None for key,v in values.items()}))
            for r in c['rows'][m]:
                rows.append(dict(cohort=c['cohort'],environment=c['environment'],stratum=c['design_stratum'],repeat=r['repeat'],mode=m,
                    failure_stage=r['failure_stage'],**{key.replace('.','_'):metric(r,key) if key in PERFORMANCE or r['success'] else None
                                                      for key in PERFORMANCE+QUALITY},
                    **{key:(r['sampling'].get(key,0) if m!='ompl_uniform' else None) for key in STAGES+COUNTERS}))
    write('trial_metrics.csv',rows);write('environment_metrics.csv',environments)


def figures(root,results,cases):
    folder=root/'figures';folder.mkdir(exist_ok=True)
    plt.rcParams.update({'font.size':10,'axes.spines.top':False,'axes.spines.right':False,'pdf.fonttype':42})
    short=['Cartesian\n+ IK','Joint\nprojection','OMPL\nunrestricted']
    def save(fig,name):
        for ext in ('png','pdf'):fig.savefig(folder/(name+'.'+ext),dpi=250)
        plt.close(fig)
    fig,axes=plt.subplots(2,2,figsize=(9.5,6.5),layout='constrained')
    for ax,key,title,scale in zip(axes.flat,PERFORMANCE,
        ['Audited success','Failure-penalized time (PAR2)','Planning action wall time','Attributed pipeline time, with preparation'],[100,1000,1000,1000]):
        for i,m in enumerate(MODES):
            v=results['modes'][m]['metrics'][key];y=v['mean']*scale;lo,hi=np.array(v['ci95'])*scale
            ax.errorbar(i,y,yerr=[[max(0,y-lo)],[max(0,hi-y)]],fmt='o',color=COLORS[i],capsize=4)
            if key=='success':ax.annotate(f"{results['modes'][m]['successes']}/{results['modes'][m]['requests']}",
                (i,y),xytext=(0,9),textcoords='offset points',ha='center',fontsize=9)
        ax.set_xticks(range(3),short);ax.set_xlim(-.4,2.4);ax.set_title(title,fontsize=11)
        ax.set_ylabel('%' if key=='success' else 'ms');ax.grid(axis='y',alpha=.2)
        if key=='success':
            ax.set_ylim(98.8,100.45);ax.set_yticks([99,99.5,100])
    fig.suptitle('Sampler comparison | environment means and 95% cluster intervals')
    save(fig,'sampler_outcomes')
    fig,axes=plt.subplots(1,3,figsize=(12,4.1),layout='constrained')
    for ax,key,title,scale in zip(axes,QUALITY[:3],['Joint travel (rad)','EE path length (m)','Minimum robot–world clearance (mm)'],[1,1,1000]):
        values=[environment_values(cases,m,key)[0]*scale for m in MODES]
        boxes=ax.boxplot(values,patch_artist=True,showfliers=True,flierprops=dict(markersize=3,alpha=.4))
        for patch,color in zip(boxes['boxes'],COLORS):patch.set_facecolor(color);patch.set_alpha(.6)
        ax.set_xticks([1,2,3],short);ax.set_title(title,fontsize=11);ax.grid(axis='y',alpha=.2)
    fig.suptitle('Path quality | one successful-request mean per environment; survivor selection applies')
    save(fig,'path_quality')
    fig,axes=plt.subplots(1,3,figsize=(12,4.3),layout='constrained')
    pairs=list(itertools.combinations(MODES,2));pair_names=['Cartesian − projected','Cartesian − OMPL','Projected − OMPL']
    for ax,key,label,scale in zip(axes,QUALITY[:3],['Joint travel difference (rad)','EE length difference (m)','World clearance difference (mm)'],[1,1,1000]):
        for i,(a,b) in enumerate(pairs):
            r=next(r for r in results['contrasts'] if (r['first'],r['second'],r['metric'])==(a,b,key))
            x=r['mean']*scale;lo,hi=np.array(r['ci95'])*scale
            ax.errorbar(x,i,xerr=[[max(0,x-lo)],[max(0,hi-x)]],fmt='o',color='#0072B2',capsize=4)
        ax.axvline(0,color='gray',ls=':');ax.set_yticks(range(3),pair_names);ax.invert_yaxis();ax.set_xlabel(label)
    fig.suptitle('Paired sampler effects | matched successes; 95% environment-cluster intervals')
    save(fig,'paired_sampler_effects')
    fig,axes=plt.subplots(1,2,figsize=(9,4.2),layout='constrained')
    for offset,key,label,color in [(-.18,'setup_s','Setup (includes anchor IK)','#CC79A7'),(.18,'sampling_s','Online sampling (includes IK/FK)','#56B4E9')]:
        values=[results['modes'][m]['stages'][key]['median_s']*1000 for m in MODES[:2]]
        axes[0].bar(np.arange(2)+offset,values,width=.36,label=label,color=color)
        for i,value in enumerate(values):
            axes[0].annotate(f'{value:.3f}',(i+offset,value),xytext=(0,4),textcoords='offset points',ha='center',fontsize=9)
    axes[0].set_xticks([0,1],short[:2]);axes[0].set_ylabel('Median per-request time (ms)');axes[0].legend(fontsize=8)
    axes[0].set_ylim(0,1.45*max(results['modes'][m]['stages'][k]['median_s']*1000 for m in MODES[:2] for k in ('setup_s','sampling_s')))
    for i,m in enumerate(MODES[:2]):
        r=results['modes'][m]['acceptance'];x=r['mean']*100;lo,hi=np.array(r['ci95'])*100
        axes[1].errorbar(i,x,yerr=[[max(0,x-lo)],[max(0,hi-x)]],fmt='o',color=COLORS[i],capsize=4)
    axes[1].set_xticks([0,1],short[:2]);axes[1].set_xlim(-.4,1.4);axes[1].set_ylim(0,100)
    axes[1].set_ylabel('Mean environment acceptance (%)');axes[1].set_title('Valid sampler returns / attempted proposals',fontsize=10)
    fig.suptitle('GMM sampler costs | setup and sampling are distinct stages')
    save(fig,'sampler_costs')
    fig,ax=plt.subplots(figsize=(7,3.8),layout='constrained')
    keys=['model_load_s','reproduction_s','rl_deformation_s','deformed_regression_s']
    ax.bar(range(4),[results['preparation'][k]['mean_s']*1000 for k in keys],color=['#999999','#0072B2','#D55E00','#009E73'])
    ax.set_xticks(range(4),['Model\nloading','TP-GMM\nreproduction','RL\ndeformation','Deformed\nGMR']);ax.set_ylabel('Mean core preparation time (ms)')
    ax.set_title('Shared GMM preparation | one measurement per environment')
    save(fig,'preparation_stages')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('cohorts',type=Path,nargs='+')
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    if args.output.exists():parser.error('Use a fresh output directory; existing results are preserved')
    cases=[];reference=None;provenance=[];fingerprints=set();manifests=[]
    for i,path in enumerate(args.cohorts):
        manifest,rows=load(path);verify_configuration(manifest,reference)
        reference=reference or manifest;manifests.append(manifest)
        for row in rows.values():verify_row(row,manifest['settings']['cutoff'])
        provenance.append(dict(cohort=f'cohort_{i+1}',directory=str(path.resolve()),
            manifest_sha256=hashlib.sha256((path/'manifest.json').read_bytes()).hexdigest(),
            results_sha256=hashlib.sha256((path/'results.jsonl').read_bytes()).hexdigest(),
            environments=len(manifest['environments']),repeats=manifest['design']['repeats']))
        for environment in manifest['environments']:
            content=json.loads((path/environment['file']).read_text());e=environment['id']
            identity={k:v for k,v in content['case'].items() if k not in ('name','id')}
            fingerprint=hashlib.sha256(json.dumps(identity,sort_keys=True).encode()).hexdigest()
            if fingerprint in fingerprints:raise ValueError('Repeated environment across inputs')
            fingerprints.add(fingerprint)
            group=environment['stratum'].split('_c')[0]+f'_c{environment["clearance"]:g}'
            cases.append(dict(cohort=f'cohort_{i+1}',environment=e,stratum=f'{i}/'+group,design_stratum=group,
                pipeline=content['pipeline'],rows={m:[rows[e,j,m] for j in range(manifest['design']['repeats'])] for m in MODES}))
    rng=np.random.default_rng(202609291)
    results=analyze(cases,rng)
    results['configuration']=dict(settings=reference['settings'],runtime=reference['runtime'],
        planning_time=reference['design']['planning_time'],policy_sha256=reference['policy_sha256'],environment_ranges=reference['environment_ranges'])
    results['analysis']=dict(scope='Fixed-configuration sampler comparison; no before/after data',
        status='Retrospective exploratory synthesis; configuration selected during preliminary diagnostics',
        unit='Environment, equal weight across cohorts; preserve paired repeat IDs',bootstrap=20000,permutations=100000,
        seed=202609291,stratification='Cohort × obstacle layout × requested clearance',
        correction='One Holm family: all 24 two-sided pairwise tests across eight metrics',
        intervals='Pointwise 95% cluster intervals; not simultaneous',
        quality='Paired contrasts use only requests successful under both methods',
        baseline='OMPL bounded joint-space sampling with empty path constraints; no GMM preparation')
    args.output.mkdir(parents=True)
    (args.output/'statistics.json').write_text(json.dumps(results,indent=2)+'\n')
    (args.output/'analysis_inputs.json').write_text(json.dumps(provenance,indent=2)+'\n')
    export_tables(args.output,results,cases);figures(args.output,results,cases)
    source=args.output/'provenance';source.mkdir()
    for script in (Path(__file__),Path(__file__).with_name('analyze_sampling_study.py'),Path(__file__).with_name('compare_clearance_replays.py')):
        shutil.copy2(script,source/script.name)
    for i,(path,manifest) in enumerate(zip(args.cohorts,manifests)):
        shutil.copy2(path/'manifest.json',source/f'cohort_{i+1}_manifest.json')
    example=args.cohorts[0]/'figures'
    for ext in ('png','pdf'):
        if (example/f'representative_paths.{ext}').exists():
            shutil.copy2(example/f'representative_paths.{ext}',args.output/'figures'/f'representative_paths.{ext}')
    print(json.dumps({m:dict(successes=v['successes'],requests=v['requests'],metrics=v['metrics'],acceptance=v.get('acceptance'))
                      for m,v in results['modes'].items()},indent=2))


if __name__=='__main__':main()
