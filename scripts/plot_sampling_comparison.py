#!/usr/bin/env python3
"""Render a standalone figure from comparison JSON; no running ROS required."""
import argparse
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def plot(source):
    data = json.loads(source.read_text())
    modes = list(data['summary'])
    labels = {'cartesian_ik': 'Cartesian + IK', 'joint_projected': 'Joint projection', 'uniform': 'MoveIt uniform'}
    colors = ['#0891b2', '#7c3aed', '#d97706']
    fig, axes = plt.subplots(2, 3, figsize=(15, 8), constrained_layout=True)
    fig.suptitle('TP-GMM sampling comparison', fontsize=20, fontweight='bold')
    rows = [r for r in data['trials'] if not r.get('warmup')]
    summary = data['summary']
    x = np.arange(len(modes))
    rates = [summary[m]['success_rate'] or 0 for m in modes]
    axes[0, 0].bar(x, rates, color=colors[:len(modes)])
    for i, mode in enumerate(modes):
        s = summary[mode]
        axes[0, 0].text(i, rates[i]+0.025, f"{s['successes']}/{s['trials']}", ha='center')
        if 'success_rate_wilson95' in s:
            lo, hi = s['success_rate_wilson95']
            axes[0, 0].plot([i, i], [lo, hi], color='#334155', linewidth=1)
    axes[0, 0].set(title='Planning success · 95% Wilson interval', ylim=(0, 1.15), ylabel='Fraction')
    panels = [(axes[0, 1], 'action_wall_s', 'Action latency · all completed requests', 'Seconds', False),
              (axes[0, 2], 'min_world_clearance_m', 'Robot-to-world clearance · successful paths', 'Metres', True),
              (axes[1, 0], 'joint_path_length', 'Joint path length · successful paths', 'MoveIt joint distance', True)]
    rng = np.random.default_rng(42)
    for ax, key, title, unit, successful in panels:
        for i, mode in enumerate(modes):
            values = [r.get(key, r.get('sampling', {}).get(key)) for r in rows
                      if r['mode'] == mode and (not successful or r.get('success'))]
            values = [v for v in values if v is not None]
            if values:
                ax.scatter(i+rng.uniform(-0.1, 0.1, len(values)), values, color=colors[i], alpha=0.6)
                ax.plot([i-.2, i+.2], [np.median(values)]*2, color=colors[i], linewidth=3)
        ax.set(title=title, ylabel=unit)
    values = [summary[m]['valid_per_attempt'] for m in modes]
    axes[1, 1].bar(x, [v or 0 for v in values], color=colors[:len(modes)])
    axes[1, 1].set(title='Sampling efficiency', ylabel='Valid targets / proposal attempts', ylim=(0, 1.05))
    for i, v in enumerate(values):
        if v is None: axes[1, 1].text(i, .1, 'No samples', ha='center', rotation=90)
    bottom = np.zeros(len(modes))
    for key, label, color in [('setup_s', 'Setup (includes anchor IK + Jacobian)', '#7c3aed'),
                              ('sampling_s', 'Sampling (includes IK/FK + validity)', '#0891b2')]:
        vals = [summary[m].get(key+'_median') or 0 for m in modes]
        axes[1, 2].bar(x, vals, bottom=bottom, label=label, color=color)
        bottom += vals
    axes[1, 2].set(title='Instrumented sampler time · medians', ylabel='Seconds')
    axes[1, 2].legend(fontsize=7)
    for ax in axes.flat:
        ax.set_xticks(x, [labels[m] for m in modes], rotation=15, fontsize=9)
        ax.spines[['top', 'right']].set_visible(False)
        ax.grid(axis='y', alpha=.15)
        ax.set_axisbelow(True)
    for extension in ('png', 'pdf'):
        fig.savefig(source.with_name('comparison.'+extension), dpi=180)
    plt.close(fig)

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('results', type=Path)
    plot(parser.parse_args().results)
