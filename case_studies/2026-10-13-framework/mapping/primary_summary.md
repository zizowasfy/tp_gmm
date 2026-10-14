# Pure sampling case study results

60 source environments, 10 repeats per method, 1800 request outcomes; fixed 10 s planning budget.

| Method | Audited success | Cluster 95% CI | Mean PAR2 (s) | Success median action (s) | Median EE length (m) | Median world clearance (m) |
|---|---:|---:|---:|---:|---:|---:|
| cartesian_ik | 600/600 (100.0%) | 100.0–100.0% | 0.1887 | 0.1824 | 0.7557 | 0.0144 |
| joint_projected | 600/600 (100.0%) | 100.0–100.0% | 0.1861 | 0.1765 | 0.6793 | 0.0126 |
| ompl_uniform | 600/600 (100.0%) | 100.0–100.0% | 0.2094 | 0.2041 | 2.1753 | 0.0172 |

Paired primary contrasts (first minus second; negative time favors first):

| First / second | Outcome | Difference | Cluster 95% CI | Holm p |
|---|---|---:|---:|---:|
| cartesian_ik / joint_projected | success (pp) | 0.0000 | [0.0000, 0.0000] | 1 |
| cartesian_ik / joint_projected | PAR2 (s) | 0.0025 | [-0.0080, 0.0126] | 1 |
| cartesian_ik / ompl_uniform | success (pp) | 0.0000 | [0.0000, 0.0000] | 1 |
| cartesian_ik / ompl_uniform | PAR2 (s) | -0.0207 | [-0.0277, -0.0136] | 5.99994e-05 |
| joint_projected / ompl_uniform | success (pp) | 0.0000 | [0.0000, 0.0000] | 1 |
| joint_projected / ompl_uniform | PAR2 (s) | -0.0232 | [-0.0331, -0.0125] | 0.000549995 |

All three pairwise success/PAR2 tests belong to one Holm family. Intervals are pointwise, not multiplicity-adjusted. Zero-width intervals in all-success samples do not prove perfect population reliability.

Pipeline times start after scene and endpoint design. They exclude the shared endpoint IK solve and offline dense path audit. Model-preparation wall time is included for custom methods, while the unrestricted baseline needs no deformation.

Sampler stage medians in seconds (setup and sampling contain the nested timers):

| Method | Setup | Draw loop | Online IK | Anchor IK | Jacobian | FK mapping | Valid / attempted |
|---|---:|---:|---:|---:|---:|---:|---:|
| cartesian_ik | 0.0000 | 0.0095 | 0.0094 | 0.0000 | 0.0000 | 0.0000 | 0.5423 |
| joint_projected | 0.0191 | 0.0002 | 0.0000 | 0.0189 | 0.0001 | 0.0000 | 0.1820 |
| ompl_uniform | — | — | — | — | — | — | — |

Shared model-preparation stage times (one measurement per environment; seconds):

| Stage | Mean | Median |
|---|---:|---:|
| model_load_s | 0.004970 | 0.004309 |
| reproduction_s | 0.008145 | 0.008120 |
| rl_deformation_s | 0.000866 | 0.000802 |
| deformed_regression_s | 0.001755 | 0.001741 |
| total_s | 0.017359 | 0.016682 |

Pure sampler counters: zero internal uniform attempts and missing-anchor Cartesian rescues are required. The separate strict runtime regression verifies the MoveIt wrapper failure path. The unrestricted baseline has no GMM session/counters.

This is a benchmark-framework alignment replay of previously used environments, not new independent confirmation. All path-quality results below are conditional on success. Matched-pair effects and retained sample counts are in `statistics.json`. Minimum world clearance includes the table and all robot links; it is not the RL reward or only an EE-to-cylinder distance.

The custom hard corridor is an exact union of covariance ellipsoids; sampler targets, planner states and interpolated edges must satisfy it. The baseline has no GMM corridor, so its contrasts are between different feasible-region configurations. Projected proposals only approximate the Cartesian Gaussian under a local linearization. The two custom methods also differ in proposal orientation behavior.

Raw inputs and outputs: `manifest.json`, `environments/*.json`, `results.jsonl`, `trials.csv`. Primary figures are supplied as PDF and PNG. No result from this study alone establishes transfer to EV battery disassembly.
