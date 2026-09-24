#!/usr/bin/env python3
"""Offline GMR pilot figures; results.json is the complete plotting input."""
import argparse
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def plot(source):
    data=json.loads(source.read_text())
    labels=list(data['summary'])
    rows=[r for r in data['trials'] if not r.get('warmup')]
    colors={label:plt.get_cmap('tab10')(i%10) for i,label in enumerate(labels)}
    fig,axes=plt.subplots(1,3,figsize=(16,5),constrained_layout=True)
    for ax,key,title,unit in zip(axes,
        ['reference_deviation_mean_m','ee_min_obstacle_clearance_m','end_to_end_wall_s'],
        ['Distance from GMR reference','End-effector obstacle clearance','Total pipeline latency'],
        ['Metres (successful paths)','Metres (successful paths)','Seconds (all attempts)']):
        for i,label in enumerate(labels):
            subset=[r for r in rows if r['mode']==label and (key=='end_to_end_wall_s' or r.get('success'))]
            values=[r.get(key,r.get('sampling',{}).get(key)) for r in subset]
            values=[v for v in values if v is not None]
            if values:
                ax.scatter(np.linspace(i-.12,i+.12,len(values)),values,color=colors[label],alpha=.5,s=20)
                ax.plot([i-.22,i+.22],[np.median(values)]*2,color=colors[label],lw=3)
        ax.set(title=title,ylabel=unit)
        ax.set_xticks(range(len(labels)),[x.replace('/','\n') for x in labels],rotation=25,fontsize=8)
        ax.grid(axis='y',alpha=.2); ax.spines[['top','right']].set_visible(False)
    fig.suptitle('GMR proposal pilot — individual runs and medians',fontsize=16)
    for ext in ['png','pdf']: fig.savefig(source.with_name('gmr_diagnostics.'+ext),dpi=180)
    plt.close(fig)
    cases=data['metadata']['cases']
    fig=plt.figure(figsize=(6*len(cases),5),constrained_layout=True)
    # One common width for readable overlays, explicitly label the selected runs.
    gmr_labels=[x for x in labels if x.startswith('gmr_path/')]
    width=min(gmr_labels,key=lambda x:abs(float(x.split('/')[1][:-1])-.015)).split('/')[1] if gmr_labels else ''
    selected=['gmm',f'gmr_path/{width}',f'hybrid/{width}']
    for i,case in enumerate(cases):
        ax=fig.add_subplot(1,len(cases),i+1,projection='3d')
        eligible=[r for r in rows if r['case']==case['name'] and r.get('success') and r.get('reference_sha256')]
        if eligible:
            ref=data['metadata']['references'][eligible[0]['reference_sha256']]
            points=np.array([[p['position'][k] for k in ['x','y','z']] for p in ref['poses']])
            ax.plot(*points.T,color='black',ls='--',lw=2,label='DSGMR reference')
            for label in selected:
                candidates=[r for r in eligible if r['mode']==label and r.get('sampling',{}).get('ee_path')]
                if candidates:
                    row=sorted(candidates,key=lambda r:r['repeat'])[0]
                    xyz=np.array(row['sampling']['ee_path'])
                    ax.plot(*xyz.T,color=colors[label],lw=1.8,label=f'{label} (run {row["repeat"]})')
        theta=np.linspace(0,2*np.pi,32)
        z=np.array([case['obstacle'][2]-case['height']/2,case['obstacle'][2]+case['height']/2])
        ax.plot_surface(case['obstacle'][0]+case['radius']*np.cos(theta)[None,:]+np.zeros((2,1)),
                        case['obstacle'][1]+case['radius']*np.sin(theta)[None,:]+np.zeros((2,1)),
                        z[:,None]+np.zeros((1,len(theta))),color='gray',alpha=.3)
        ax.scatter(*case['start'],color='green',s=35);ax.scatter(*case['goal'],color='red',s=35)
        ax.set(title=case['name'],xlabel='x (m)',ylabel='y (m)',zlabel='z (m)')
        ax.legend(fontsize=7,loc='upper left')
    fig.suptitle('Successful planned end-effector paths — not measured execution',fontsize=15)
    for ext in ['png','pdf']: fig.savefig(source.with_name('gmr_paths.'+ext),dpi=180)
    plt.close(fig)

if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('results',type=Path)
    plot(parser.parse_args().results)
