# Pure sampling case study results

60 source environments, 10 repeats per method, 1800 request outcomes; fixed 3 s planning budget.

| Method | Audited success | Cluster 95% CI | Mean PAR2 (s) | Success median action (s) | Median EE length (m) | Median world clearance (m) |
|---|---:|---:|---:|---:|---:|---:|
| cartesian_ik | 599/600 (99.8%) | 99.5–100.0% | 0.1675 | 0.1505 | 0.7979 | 0.0148 |
| joint_projected | 599/600 (99.8%) | 99.5–100.0% | 0.1648 | 0.1465 | 0.7199 | 0.0133 |
| ompl_uniform | 600/600 (100.0%) | 100.0–100.0% | 0.1766 | 0.1657 | 2.1349 | 0.0158 |

Paired primary contrasts (first minus second; negative time favors first):

| First / second | Outcome | Difference | Cluster 95% CI | Holm p |
|---|---|---:|---:|---:|
| cartesian_ik / joint_projected | success (pp) | 0.0000 | [-0.5000, 0.5000] | 1 |
| cartesian_ik / joint_projected | PAR2 (s) | 0.0027 | [-0.0265, 0.0316] | 1 |
| cartesian_ik / ompl_uniform | success (pp) | -0.1667 | [-0.5000, 0.0000] | 1 |
| cartesian_ik / ompl_uniform | PAR2 (s) | -0.0091 | [-0.0249, 0.0144] | 1 |
| joint_projected / ompl_uniform | success (pp) | -0.1667 | [-0.5000, 0.0000] | 1 |
| joint_projected / ompl_uniform | PAR2 (s) | -0.0119 | [-0.0294, 0.0123] | 1 |

All three pairwise success/PAR2 tests belong to one Holm family. Intervals are pointwise, not multiplicity-adjusted. Zero-width intervals in all-success samples do not prove perfect population reliability.

Pipeline times start after scene and endpoint design. They exclude the shared endpoint IK solve and offline dense path audit. Model-preparation wall time is included for custom methods, while the unrestricted baseline needs no deformation.

Sampler stage medians in seconds (setup and sampling contain the nested timers):

| Method | Setup | Draw loop | Online IK | Anchor IK | Jacobian | FK mapping | Valid / attempted |
|---|---:|---:|---:|---:|---:|---:|---:|
| cartesian_ik | 0.0000 | 0.0093 | 0.0091 | 0.0000 | 0.0000 | 0.0000 | 0.5376 |
| joint_projected | 0.0212 | 0.0002 | 0.0000 | 0.0210 | 0.0001 | 0.0000 | 0.1899 |
| ompl_uniform | — | — | — | — | — | — | — |

Shared model-preparation stage times (one measurement per environment; seconds):

| Stage | Mean | Median |
|---|---:|---:|
| model_load_s | 0.005860 | 0.005043 |
| reproduction_s | 0.011551 | 0.009079 |
| rl_deformation_s | 0.002869 | 0.000990 |
| deformed_regression_s | 0.002396 | 0.001903 |
| total_s | 0.024681 | 0.019733 |

Pure sampler counters: zero internal uniform attempts and missing-anchor Cartesian rescues are required. The separate strict runtime regression verifies the MoveIt wrapper failure path. The unrestricted baseline has no GMM session/counters.

This is a covariance-cutoff sensitivity replay of previously used environments, not new independent confirmation. All path-quality results below are conditional on success. Matched-pair effects and retained sample counts are in `statistics.json`. Minimum world clearance includes the table and all robot links; it is not the RL reward or only an EE-to-cylinder distance.

The custom hard corridor is a union of oriented boxes enclosing the cutoff ellipsoids; accepted targets obey the ellipsoidal cutoff, while tree edges may enter box corners. The baseline has no GMM corridor, so its contrasts are between different feasible-region configurations. Projected proposals only approximate the Cartesian Gaussian under a local linearization. The two custom methods also differ in proposal orientation behavior.

Raw inputs and outputs: `manifest.json`, `environments/*.json`, `results.jsonl`, `trials.csv`. Primary figures are supplied as PDF and PNG. No result from this study alone establishes transfer to EV battery disassembly.
