#!/usr/bin/env python3
"""Publication/export figures from clearance sweep JSON; no ROS runtime required."""
import argparse
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import Normalize
from matplotlib.cm import ScalarMappable

LABELS = {'cartesian_ik':'Cartesian + IK', 'joint_projected':'Joint projection', 'ompl_uniform':'OMPL uniform joint space'}
COLORS = {'cartesian_ik':'#0891b2', 'joint_projected':'#7c3aed', 'ompl_uniform':'#d97706'}


def save_figure(fig, directory, name):
    for extension in ('png', 'pdf'):
        fig.savefig(directory/f'{name}.{extension}', dpi=180, bbox_inches='tight')
    plt.close(fig)


def band(ax, rows, key, label, color, linestyle='-'):
    levels = sorted({r['clearance'] for r in rows})
    median, low, high = [], [], []
    for c in levels:
        values = [r[key] for r in rows if r['clearance']==c and r.get(key) is not None]
        quantiles = np.percentile(values,[25,50,75]) if values else [np.nan]*3
        low.append(quantiles[0]); median.append(quantiles[1]); high.append(quantiles[2])
    ax.plot(levels, median, linestyle=linestyle, marker='o', ms=3, color=color, label=label)
    ax.fill_between(levels, low, high, color=color, alpha=.10)


def plot(source, max_examples=3):
    source = Path(source)
    data = json.loads(source.read_text())
    rows, metadata = data['rows'], data['metadata']
    directory = source.parent/'figures'; directory.mkdir(exist_ok=True)
    modes = list(dict.fromkeys(r['sampler'] for r in rows))
    # RL inference is shared within a trial/clearance: never count it repeatedly across modes.
    unique = list({(r['trial'],r['clearance']):r for r in rows}.values())
    lines = ['# RL clearance sweep', '', f"Run status: **{metadata['status']}**. Completed planning rows: {len(rows)}.", '',
             'Clearance input is normalized. Metrics use metres. The training reward reference distance and finite-cylinder surface distance are distinct.',
             'Shaded bands show the interquartile range across environments, not confidence intervals. Failed plans are included in success rates and excluded from successful-path metrics.',
             'OMPL uniform has no GMM constraints, so clearance does not affect its planning request; any apparent trend is planner randomness.', '',
             '| Sampler | Success | Deformed adjacent nondecreasing pairs | Planned adjacent nondecreasing pairs |',
             '|---|---:|---:|---:|']
    for mode, summary in data['summary'].items():
        selected = [r for r in rows if r['sampler']==mode]
        fmt = lambda x: 'unavailable' if x['nondecreasing_fraction'] is None else f"{100*x['nondecreasing_fraction']:.1f}% of {x['adjacent_pairs']} pairs"
        lines.append(f"| {LABELS[mode]} | {sum(bool(r.get('success')) for r in selected)}/{len(selected)} | {fmt(summary['deformed_monotonicity'])} | {fmt(summary['planned_monotonicity'])} |")
    lines += ['', 'Monotonicity allows 2 mm numerical/planning variation and uses adjacent observed levels only; missing levels are not bridged.',
              'Original/deformed curves are DSGMR trajectories, not collision-checked robot motions. Planned distances measure the named end-effector origin, not the full robot surface.',
              'The additional interior metric excludes the first/last configured fractions of arc length; full-path minima are always retained.',
              'FK validity is sampled at the recorded joint step. Polyline clearance is sampled at the recorded Cartesian step; neither is a continuous collision certificate.',
              'Environment generation is conditioned on valid endpoint IK; every rejected candidate and the full generation ranges are retained. This is a generalization evaluation, not proof of universal generalization.',
              'Sampler stage timers overlap: do not add IK/Jacobian/validity sub-timers to their setup/sampling totals. Analysis and FK-query time is recorded separately.']
    (source.parent/'summary.md').write_text('\n'.join(lines)+'\n')
    if not rows:
        return
    plt.rcParams.update({'font.size':10, 'axes.spines.top':False, 'axes.spines.right':False})
    fig, axes = plt.subplots(2,3,figsize=(16,9),constrained_layout=True)
    fig.suptitle('RL clearance generalization · fixed environment within each sweep', fontsize=17)
    band(axes[0,0], unique, 'original_min_clearance_m', 'Prior DSGMR', '#64748b', '--')
    band(axes[0,0], unique, 'deformed_min_clearance_m', 'RL-deformed DSGMR', '#0f766e')
    axes[0,0].set(title='Minimum trajectory-to-cylinder clearance', ylabel='Signed surface distance (m)')
    for mode in modes:
        selected = [r for r in rows if r['sampler']==mode]
        for ax, key in [(axes[0,1],'planned_min_clearance_m'),(axes[1,1],'action_wall_s'),(axes[1,2],'planned_path_length_m')]:
            band(ax, selected, key, LABELS[mode], COLORS[mode])
        values = data['summary'][mode]['levels']
        axes[0,2].plot([v['clearance'] for v in values],[v['success_rate'] for v in values],'-o',color=COLORS[mode],label=LABELS[mode],ms=4)
    axes[0,1].set(title='Planned end-effector minimum · successful only', ylabel='Signed surface distance (m)')
    axes[0,2].set(title='Planning success · failures retained',ylabel='Fraction',ylim=(-.03,1.03))
    policy_rows = [dict(r, measured=r['policy']['intermediate_mean_min_margin_m']) for r in unique]
    band(axes[1,0], policy_rows, 'measured', 'Intermediate Gaussian means', '#0f766e')
    levels = sorted({r['clearance'] for r in rows})
    args = metadata['args']
    axes[1,0].plot(levels,[args['base_buffer']+c*args['max_clearance_margin'] for c in levels],'k--',label='Training target incl. base buffer')
    axes[1,0].set(title='Training reward geometry · 3-D reference distance',ylabel='Distance minus obstacle radius (m)')
    axes[1,1].set(title='Planning action latency · all completed requests',ylabel='Seconds')
    axes[1,2].set(title='End-effector path length · successful only',ylabel='Metres')
    for ax in axes.flat:
        ax.set_xlabel('Requested clearance (normalized)'); ax.set_xticks(levels); ax.grid(alpha=.2)
        ax.legend(fontsize=7,loc='best')
    for ax in (axes[0,0],axes[0,1]):
        ax.axhline(0,color='#b91c1c',lw=.8,linestyle=':')
    save_figure(fig,directory,'clearance_overview')
    # Per-environment curves make heterogeneity visible rather than relying on aggregate trends.
    fig, axes = plt.subplots(1,3,figsize=(17,4.5),constrained_layout=True)
    for trial in sorted({r['trial'] for r in unique}):
        selected = sorted([r for r in unique if r['trial']==trial],key=lambda r:r['clearance'])
        axes[0].plot([r['clearance'] for r in selected],[r['deformed_min_clearance_m'] for r in selected],'-o',ms=3,alpha=.6)
        axes[1].plot([r['clearance'] for r in selected],[r['deformation_gain_m'] for r in selected],'-o',ms=3,alpha=.6)
    for ax, title in zip(axes,['RL-deformed curve · one line per environment','Clearance gain relative to the prior curve']):
        ax.set(title=title,xlabel='Requested clearance (normalized)',ylabel='Metres'); ax.axhline(0,color='grey',lw=.7); ax.grid(alpha=.2)
    endpoint_rows = [dict(r, start_error=r['curves']['deformed']['endpoint_start_error_m'],
                          goal_error=r['curves']['deformed']['endpoint_goal_error_m']) for r in unique]
    band(axes[2], endpoint_rows, 'start_error', 'Start', '#0891b2')
    band(axes[2], endpoint_rows, 'goal_error', 'Goal', '#7c3aed')
    axes[2].set(title='Deformed curve endpoint error', xlabel='Requested clearance (normalized)', ylabel='Metres')
    axes[2].grid(alpha=.2); axes[2].legend()
    save_figure(fig,directory,'paired_environment_response')
    for trial in sorted({r['trial'] for r in rows})[:max_examples]:
        for mode in modes:
            selected = sorted([r for r in rows if r['trial']==trial and r['sampler']==mode],key=lambda r:r['clearance'])
            if not selected: continue
            fig = plt.figure(figsize=(20,6),layout='constrained')
            fig.get_layout_engine().set(wspace=.15)
            a = fig.add_subplot(131,projection='3d'); b = fig.add_subplot(132,projection='3d'); cax = fig.add_subplot(133)
            fig.suptitle(f"Environment {trial} · {LABELS[mode]} · color = requested clearance",fontsize=15)
            case = selected[0]['environment']; center=np.array(case['obstacle'])
            theta=np.linspace(0,2*np.pi,32); z=np.array([-case['height']/2,case['height']/2])+center[2]
            xx=center[0]+case['radius']*np.cos(theta); yy=center[1]+case['radius']*np.sin(theta)
            for ax, title in [(a,'Prior and RL-deformed DSGMR'),(b,'Successful planned end-effector paths')]:
                ax.plot_surface(np.tile(xx,(2,1)),np.tile(yy,(2,1)),np.tile(z[:,None],(1,len(theta))),color='#94a3b8',alpha=.25)
                prior=np.array(selected[0]['curves']['original']['xyz'])
                ax.plot(*prior.T,'k--',lw=1.5,label='Prior')
                ax.scatter(*case['start'],c='black',marker='o',s=35); ax.scatter(*case['goal'],c='black',marker='*',s=70)
                ax.set(title=title,xlabel='x (m)',ylabel='y (m)',zlabel='z (m)'); ax.view_init(elev=25,azim=-60)
            norm=Normalize(0,1); cmap=plt.get_cmap('viridis')
            for row in selected:
                color=cmap(norm(row['clearance']))
                curve=row['curves']['deformed']; a.plot(*np.array(curve['xyz']).T,color=color,lw=1.5)
                cax.plot(curve['progress'],curve['surface_distance_m'],color=color,ls='--',alpha=.7)
                if row.get('success') and 'planned' in row['curves']:
                    curve=row['curves']['planned']; b.plot(*np.array(curve['xyz']).T,color=color,lw=1.5)
                    cax.plot(curve['progress'],curve['surface_distance_m'],color=color,lw=1.5)
            all_xyz = np.vstack([np.array(r['curves'][key]['xyz']) for r in selected for key in r['curves']])
            lower = np.minimum(all_xyz.min(axis=0), center-[case['radius'],case['radius'],case['height']/2])
            upper = np.maximum(all_xyz.max(axis=0), center+[case['radius'],case['radius'],case['height']/2])
            span = np.maximum(upper-lower, .05)
            for ax in (a,b):
                ax.set_xlim(lower[0]-.05*span[0],upper[0]+.05*span[0])
                ax.set_ylim(lower[1]-.05*span[1],upper[1]+.05*span[1])
                ax.set_zlim(lower[2]-.05*span[2],upper[2]+.05*span[2])
                ax.set_box_aspect(span)
                ax.tick_params(labelsize=8)
            cax.set(title='Cylinder clearance along arc length\nDashed: DSGMR · solid: successful plans',xlabel='Normalized arc length',ylabel='Signed surface distance (m)')
            cax.axhline(0,color='#b91c1c',lw=.8); cax.grid(alpha=.2)
            fig.colorbar(ScalarMappable(norm=norm,cmap=cmap),ax=[a,b,cax],shrink=.65,label='Requested clearance')
            save_figure(fig,directory,f'environment_{trial:03d}_{mode}')


if __name__ == '__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('results',type=Path)
    parser.add_argument('--max-examples',type=int,default=3)
    args=parser.parse_args()
    plot(args.results,max_examples=args.max_examples)
