#!/usr/bin/env python3
"""Descriptive distribution checks from raw proposal and FK moments, without mixing acceptance stages."""
import numpy as np
from scipy.stats import chi2


def distribution_report(rows, metadata):
    output = []
    settings = metadata['settings']
    cutoff, floor = settings['cutoff'], settings['covariance_floor']
    for digest, model in metadata['models'].items():
        for mode in metadata['modes']:
            selected_rows = [r for r in rows if r.get('model_sha256') == digest and r['mode'] == mode and not r.get('warmup')]
            for k, component in enumerate(model['gaussians']):
                radius = (0.5 * settings.get('corridor_scale', 10.0) * model['weights'][k]
                          if settings.get('corridor_mode', 'covariance') == 'legacy_weighted' else cutoff)
                if radius <= 0:
                    continue
                # Each component can have a different historical truncation radius.
                covariance_ratio = chi2.cdf(radius**2, 5) / chi2.cdf(radius**2, 3)
                dimension = len(component['means'])
                mean = np.array(component['means'][-3:])
                covariance = np.array(component['covariances']).reshape(dimension, dimension)[-3:, -3:]
                values, axes = np.linalg.eigh((covariance+covariance.T)/2)
                expected = covariance_ratio * ((axes*np.maximum(values, floor))@axes.T)
                for stage in ('proposal', 'projected_linear', 'projected_fk', 'valid'):
                    prefix = f'component_{k}_{stage}_'
                    metrics = [r.get('sampling', {}) for r in selected_rows]
                    count = sum(m.get(prefix+'count', 0) for m in metrics)
                    if count < 2:
                        continue
                    total = np.array([sum(m.get(prefix+f'sum_{i}', 0) for m in metrics) for i in range(3)])
                    outer = np.array([[sum(m.get(prefix+f'outer_{i}{j}', 0) for m in metrics)
                                       for j in range(3)] for i in range(3)])
                    empirical_mean = total/count
                    empirical_cov = outer/count - np.outer(empirical_mean, empirical_mean)
                    output.append(dict(model_sha256=digest, mode=mode, component=k, stage=stage,
                        cutoff=float(radius), count=int(count), empirical_mean=empirical_mean.tolist(),
                        empirical_covariance=empirical_cov.tolist(), expected_proposal_mean=mean.tolist(),
                        expected_truncated_covariance=expected.tolist(),
                        mean_error_m=float(np.linalg.norm(empirical_mean-mean)),
                        covariance_relative_error=float(np.linalg.norm(empirical_cov-expected)/np.linalg.norm(expected)),
                        interpretation=('Exact task-space proposal; low counts are noisy' if stage in ('proposal','projected_linear')
                                        else 'FK/validity-conditioned distribution; equality with the GMM is not expected')))
    return output
