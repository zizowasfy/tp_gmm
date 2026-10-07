# Pure sampling case study results

60 independent environments, 5 repeats per method, 900 plans; fixed 3 s planning budget.

| Method | Audited success | Cluster 95% CI | Mean PAR2 (s) | Success median action (s) | Median EE length (m) | Median world clearance (m) |
|---|---:|---:|---:|---:|---:|---:|
| cartesian_ik | 280/300 (93.3%) | 86.7–98.3% | 0.5296 | 0.1265 | 0.7268 | 0.0154 |
| joint_projected | 279/300 (93.0%) | 86.3–98.3% | 0.5456 | 0.1229 | 0.6679 | 0.0132 |
| ompl_uniform | 300/300 (100.0%) | 100.0–100.0% | 0.1515 | 0.1436 | 2.0636 | 0.0177 |

Paired secondary contrasts (first minus second; negative time favors first):

| First / second | Outcome | Difference | Cluster 95% CI | Holm p |
|---|---|---:|---:|---:|
| cartesian_ik / joint_projected | success (pp) | 0.3333 | [0.0000, 1.0000] | 1 |
| cartesian_ik / joint_projected | PAR2 (s) | -0.0161 | [-0.0563, 0.0098] | 1 |
| cartesian_ik / ompl_uniform | success (pp) | -6.6667 | [-13.3333, -1.6667] | 0.494635 |
| cartesian_ik / ompl_uniform | PAR2 (s) | 0.3781 | [0.0798, 0.7708] | 0.494635 |
| joint_projected / ompl_uniform | success (pp) | -7.0000 | [-13.6667, -1.6667] | 0.375 |
| joint_projected / ompl_uniform | PAR2 (s) | 0.3942 | [0.0816, 0.7852] | 0.375 |

All three pairwise success/PAR2 tests belong to one Holm family. Intervals are pointwise, not multiplicity-adjusted. Zero-width intervals in all-success samples do not prove perfect population reliability.

Sampler stage medians in seconds (setup and sampling contain the nested timers):

| Method | Setup | Draw loop | Online IK | Anchor IK | Jacobian | FK mapping | Valid / attempted |
|---|---:|---:|---:|---:|---:|---:|---:|
| cartesian_ik | 0.0000 | 0.0097 | 0.0094 | 0.0000 | 0.0000 | 0.0000 | 0.5442 |
| joint_projected | 0.0163 | 0.0001 | 0.0000 | 0.0162 | 0.0001 | 0.0000 | 0.2482 |
| ompl_uniform | — | — | — | — | — | — | — |

Shared model-preparation stage times (one measurement per environment; seconds):

| Stage | Mean | Median |
|---|---:|---:|
| model_load_s | 0.004183 | 0.004146 |
| reproduction_s | 0.008056 | 0.008016 |
| rl_deformation_s | 0.000927 | 0.000757 |
| deformed_regression_s | 0.001711 | 0.001699 |
| total_s | 0.016505 | 0.016292 |

Pure sampler counters: zero internal uniform attempts and missing-anchor Cartesian rescues are required. The separate strict runtime regression verifies the MoveIt wrapper failure path. The unrestricted baseline has no GMM session/counters.

This is a clearance sensitivity replay of previously used environments, not new independent confirmation. All path-quality results below are conditional on success. Matched-pair effects and retained sample counts are in `statistics.json`. Minimum world clearance includes the table and all robot links; it is not the RL reward or only an EE-to-cylinder distance.

The custom hard corridor is a union of oriented boxes enclosing the cutoff ellipsoids; accepted targets obey the ellipsoidal cutoff, while tree edges may enter box corners. The baseline has no GMM corridor, so its contrasts are between different feasible-region configurations. Projected proposals only approximate the Cartesian Gaussian under a local linearization. The two custom methods also differ in proposal orientation behavior.

Raw inputs and outputs: `manifest.json`, `environments/*.json`, `results.jsonl`, `trials.csv`. Primary figures are supplied as PDF and PNG. No result from this study alone establishes transfer to EV battery disassembly.

## Replayed confirmation cohort (sensitivity analysis)

This rerun reuses the original confirmation environments with changed clearance, so it is not another independent confirmation. The original directional path-length tests are reproduced below as sensitivity analyses.

| Outcome (Cartesian minus projected) | Effect | 95% cluster CI | One-sided Holm p | Matched pairs / environments |
|---|---:|---:|---:|---:|
| joint_path_length | 1.09684 | [0.43185, 1.75936] | 0.00701993 | 279 / 56 |
| ee_path_length_m | 0.04476 | [0.00179, 0.08739] | 0.0340197 | 279 / 56 |
