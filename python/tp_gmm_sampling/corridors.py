"""Canonical hard-support checks and endpoint-connected references."""
from copy import deepcopy
import json
import numpy as np


def contains(descriptor, points):
    """Independent vectorized ellipsoid membership; no geometric tolerance."""
    support = json.loads(descriptor) if isinstance(descriptor, str) else descriptor
    points = np.asarray(points, dtype=float).reshape(-1, 3)
    if support['geometry'] != 'gmm_ellipsoids':
        raise ValueError('Ellipsoid descriptor required')
    cutoff = support['cutoff']
    result = np.zeros(len(points), dtype=bool)
    for component in support['components']:
        if component['weight'] <= 0:
            continue
        axes = np.asarray(component['axes']).reshape(3, 3)
        local = (points - component['mean']) @ axes
        squared = np.sum(local**2 / component['eigenvalues'], axis=1)
        result |= squared <= cutoff**2 + 64*np.finfo(float).eps*max(1., cutoff**2)
    return result


def connect_reference(reference, start_pose, goal_pose):
    """Include actual FK endpoints, matching the benchmark's GMR tube."""
    result = deepcopy(reference)
    result.poses = []
    for pose in [start_pose, *reference.poses, goal_pose]:
        x = np.array([pose.position.x, pose.position.y, pose.position.z])
        if result.poses:
            previous = result.poses[-1].position
            if np.linalg.norm(x - [previous.x, previous.y, previous.z]) <= 1e-10:
                continue
        result.poses.append(deepcopy(pose))
    if len(result.poses) < 2:
        raise ValueError('Degenerate endpoint-connected reference')
    return result
