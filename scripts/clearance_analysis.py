#!/usr/bin/env python3
"""ROS-independent geometry, experiment design and summaries for clearance sweeps."""
import csv
import json
from pathlib import Path
import numpy as np

SAMPLERS = ('cartesian_ik', 'joint_projected', 'ompl_uniform')
DEFAULT_ENVIRONMENT = {
    'start_low': [0.28, -0.20, 0.57], 'start_high': [0.38, 0.20, 0.66],
    'goal_low': [0.53, -0.20, 0.45], 'goal_high': [0.65, 0.20, 0.54],
    'obstacle_fraction': [0.4, 0.6], 'obstacle_offset_low': [-0.025, -0.035, -0.025],
    'obstacle_offset_high': [0.025, 0.035, 0.025],
    'radius': [0.03, 0.05], 'height': [0.25, 0.38], 'minimum_endpoint_separation': 0.20,
}


def clearances(include_zero=False):
    return [i / 10 for i in range(0 if include_zero else 1, 11)]


def validate_environment(config):
    if set(config) != set(DEFAULT_ENVIRONMENT):
        raise ValueError('Environment JSON must contain exactly the documented range keys')
    for prefix in ('start', 'goal', 'obstacle_offset'):
        lo, hi = np.asarray(config[prefix+'_low']), np.asarray(config[prefix+'_high'])
        if lo.shape != (3,) or hi.shape != (3,) or not np.all(np.isfinite([lo, hi])) or np.any(lo > hi):
            raise ValueError('Invalid environment bounds: '+prefix)
    for name in ('radius', 'height', 'obstacle_fraction'):
        values = np.asarray(config[name])
        if values.shape != (2,) or not np.all(np.isfinite(values)) or values[0] > values[1] or values[0] <= 0:
            raise ValueError('Invalid range: '+name)
    if config['obstacle_fraction'][1] >= 1 or not np.isfinite(config['minimum_endpoint_separation']) or config['minimum_endpoint_separation'] <= 0:
        raise ValueError('Invalid obstacle fraction or endpoint separation')


def random_environment(rng, config, index):
    start = rng.uniform(config['start_low'], config['start_high'])
    goal = rng.uniform(config['goal_low'], config['goal_high'])
    fraction = rng.uniform(*config['obstacle_fraction'])
    top = start + fraction * (goal-start) + rng.uniform(config['obstacle_offset_low'], config['obstacle_offset_high'])
    height = float(rng.uniform(*config['height']))
    center = top - [0, 0, height/2]
    return dict(name=f'trial_{index:03d}', start=start.tolist(), goal=goal.tolist(), obstacle=center.tolist(),
                radius=float(rng.uniform(*config['radius'])), height=height, orientation_xyzw=[1., 0., 0., 0.])


def validate_case(case):
    for name in ('start', 'goal', 'obstacle'):
        values = np.asarray(case[name], dtype=float)
        if values.shape != (3,) or not np.all(np.isfinite(values)):
            raise ValueError('Invalid environment xyz: '+name)
    for name in ('radius', 'height'):
        if not np.isfinite(case[name]) or case[name] <= 0:
            raise ValueError('Invalid obstacle dimension: '+name)
    if not np.allclose(case.get('orientation_xyzw', [1.,0.,0.,0.]), [1.,0.,0.,0.]):
        raise ValueError('This experiment uses fixed downward endpoint orientations')


def cylinder_distance(points, center, radius, height):
    """Exact signed distance of points to the surface of a finite vertical cylinder."""
    local = np.asarray(points, dtype=float) - np.asarray(center, dtype=float)
    radial = np.linalg.norm(local[..., :2], axis=-1) - radius
    vertical = np.abs(local[..., 2]) - height/2
    d = np.stack((radial, vertical), axis=-1)
    return np.linalg.norm(np.maximum(d, 0), axis=-1) + np.minimum(np.max(d, axis=-1), 0)


def resample_polyline(points, step):
    points = np.asarray(points, dtype=float)
    if points.ndim != 2 or points.shape[1] != 3 or len(points) < 2 or not np.all(np.isfinite(points)):
        raise ValueError('Trajectory must contain at least two finite xyz points')
    if not np.isfinite(step) or step <= 0:
        raise ValueError('Sampling step must be positive')
    lengths = np.linalg.norm(np.diff(points, axis=0), axis=1)
    counts = np.maximum(1, np.ceil(lengths/step).astype(int))
    if counts.sum() > 100000:
        raise ValueError('Trajectory exceeds analysis sample limit')
    pieces = [a + (b-a)*np.arange(n)[:, None]/n for a, b, n in zip(points[:-1], points[1:], counts)]
    return np.vstack(pieces + [points[-1:]])


def curve_metrics(points, case, step=0.002, trim=0.1):
    samples = resample_polyline(points, step)
    distance = cylinder_distance(samples, case['obstacle'], case['radius'], case['height'])
    length = np.r_[0., np.cumsum(np.linalg.norm(np.diff(samples, axis=0), axis=1))]
    progress = length / length[-1] if length[-1] > 0 else np.zeros(len(length))
    interior = distance[(progress >= trim) & (progress <= 1-trim)]
    return dict(min_clearance_m=float(distance.min()), p05_clearance_m=float(np.percentile(distance, 5)),
                median_clearance_m=float(np.median(distance)),
                interior_min_clearance_m=float(interior.min()) if len(interior) else None,
                path_length_m=float(length[-1]), intersecting=bool(np.any(distance < 0)),
                closest_point_xyz=samples[int(distance.argmin())].tolist(),
                endpoint_start_error_m=float(np.linalg.norm(samples[0]-case['start'])),
                endpoint_goal_error_m=float(np.linalg.norm(samples[-1]-case['goal'])),
                xyz=samples.tolist(), progress=progress.tolist(), surface_distance_m=distance.tolist())


def policy_metrics(original, deformed, reference, radius, target_margin):
    old = np.array([g['means'][-3:] for g in original['gaussians']])
    new = np.array([g['means'][-3:] for g in deformed['gaussians']])
    if old.shape != new.shape or len(new) < 3 or not np.all(np.isfinite([old, new])):
        raise ValueError('Expected at least three corresponding finite GMM means')
    # The training reward excludes first/last components and uses 3-D reference distance.
    margins = np.linalg.norm(new[1:-1]-reference, axis=1)-radius
    shift = np.linalg.norm(new-old, axis=1)
    return dict(intermediate_mean_min_margin_m=float(margins.min()),
                intermediate_mean_margins_m=margins.tolist(), target_margin_m=target_margin,
                training_clearance_violation_sum_m=float(np.maximum(target_margin-margins, 0).sum()),
                training_clearance_satisfied=bool(np.all(margins >= target_margin)),
                mean_shift_rms_m=float(np.sqrt(np.mean(shift**2))), max_mean_shift_m=float(shift.max()))


def monotonic_summary(rows, key, tolerance=0.002):
    pairs, increasing = 0, 0
    slopes = []
    for trial in sorted({r['trial'] for r in rows}):
        selected = sorted([r for r in rows if r['trial'] == trial and r.get(key) is not None], key=lambda r:r['clearance'])
        for a, b in zip(selected, selected[1:]):
            if not np.isclose(b['clearance']-a['clearance'], 0.1):
                continue  # Never bridge missing/failed levels.
            pairs += 1
            increasing += b[key] >= a[key]-tolerance
            slopes.append((b[key]-a[key])/(b['clearance']-a['clearance']))
    return dict(adjacent_pairs=pairs, nondecreasing_fraction=increasing/pairs if pairs else None,
                median_slope_m_per_unit=float(np.median(slopes)) if slopes else None, tolerance_m=tolerance)


def summarize(rows):
    result = {}
    for mode in sorted({r['sampler'] for r in rows}):
        subset = [r for r in rows if r['sampler'] == mode]
        levels = []
        for c in sorted({r['clearance'] for r in subset}):
            group = [r for r in subset if r['clearance'] == c]
            success = [r for r in group if r.get('success')]
            item = dict(clearance=c, trials=len(group), successes=len(success), success_rate=len(success)/len(group))
            for key in ('deformed_min_clearance_m', 'planned_min_clearance_m', 'action_wall_s',
                        'planned_path_length_m', 'policy_violation_m', 'deformation_gain_m'):
                values = [r[key] for r in group if r.get(key) is not None]
                item[key+'_median'] = float(np.median(values)) if values else None
                item[key+'_n'] = len(values)
            levels.append(item)
        result[mode] = dict(levels=levels, deformed_monotonicity=monotonic_summary(subset, 'deformed_min_clearance_m'),
                            planned_monotonicity=monotonic_summary(subset, 'planned_min_clearance_m'))
    return result


def save(output, metadata, rows):
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    payload = dict(metadata=metadata, summary=summarize(rows), rows=rows)
    temp = output/'results.tmp'
    temp.write_text(json.dumps(payload, indent=2, allow_nan=False))
    temp.replace(output/'results.json')
    keys = sorted({k for r in rows for k, v in r.items() if not isinstance(v, (dict, list))})
    with (output/'trials.csv').open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=keys)
        writer.writeheader()
        for row in rows:
            writer.writerow({k:row.get(k) for k in keys})
    with (output/'trajectory_points.csv').open('w', newline='') as stream:
        writer = csv.writer(stream)
        writer.writerow(['trial', 'clearance', 'sampler', 'curve', 'progress', 'x_m', 'y_m', 'z_m', 'surface_distance_m'])
        for row in rows:
            for name, curve in row.get('curves', {}).items():
                for xyz, progress, distance in zip(curve['xyz'], curve['progress'], curve['surface_distance_m']):
                    writer.writerow([row['trial'], row['clearance'], row['sampler'], name, progress, *xyz, distance])
    return payload
