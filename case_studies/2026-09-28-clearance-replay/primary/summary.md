# Pure sampling case study results

60 independent environments, 10 repeats per method, 1800 plans; fixed 3 s planning budget.

| Method | Audited success | Cluster 95% CI | Mean PAR2 (s) | Success median action (s) | Median EE length (m) | Median world clearance (m) |
|---|---:|---:|---:|---:|---:|---:|
| cartesian_ik | 580/600 (96.7%) | 91.7–100.0% | 0.3296 | 0.1294 | 0.7016 | 0.0136 |
| joint_projected | 579/600 (96.5%) | 91.5–100.0% | 0.3306 | 0.1168 | 0.6281 | 0.0118 |
| ompl_uniform | 600/600 (100.0%) | 100.0–100.0% | 0.1598 | 0.1523 | 2.1999 | 0.0167 |

Paired primary contrasts (first minus second; negative time favors first):

| First / second | Outcome | Difference | Cluster 95% CI | Holm p |
|---|---|---:|---:|---:|
| cartesian_ik / joint_projected | success (pp) | 0.1667 | [0.0000, 0.5000] | 1 |
| cartesian_ik / joint_projected | PAR2 (s) | -0.0010 | [-0.0230, 0.0135] | 1 |
| cartesian_ik / ompl_uniform | success (pp) | -3.3333 | [-8.3333, 0.0000] | 1 |
| cartesian_ik / ompl_uniform | PAR2 (s) | 0.1698 | [-0.0290, 0.4660] | 1 |
| joint_projected / ompl_uniform | success (pp) | -3.5000 | [-8.5000, 0.0000] | 1 |
| joint_projected / ompl_uniform | PAR2 (s) | 0.1708 | [-0.0345, 0.4666] | 1 |

All three pairwise success/PAR2 tests belong to one Holm family. Intervals are pointwise, not multiplicity-adjusted. Zero-width intervals in all-success samples do not prove perfect population reliability.

Sampler stage medians in seconds (setup and sampling contain the nested timers):

| Method | Setup | Draw loop | Online IK | Anchor IK | Jacobian | FK mapping | Valid / attempted |
|---|---:|---:|---:|---:|---:|---:|---:|
| cartesian_ik | 0.0000 | 0.0120 | 0.0117 | 0.0000 | 0.0000 | 0.0000 | 0.5848 |
| joint_projected | 0.0185 | 0.0001 | 0.0000 | 0.0182 | 0.0001 | 0.0000 | 0.2722 |
| ompl_uniform | — | — | — | — | — | — | — |

Shared model-preparation stage times (one measurement per environment; seconds):

| Stage | Mean | Median |
|---|---:|---:|
| model_load_s | 0.004253 | 0.004192 |
| reproduction_s | 0.008038 | 0.008016 |
| rl_deformation_s | 0.002516 | 0.000771 |
| deformed_regression_s | 0.001700 | 0.001695 |
| total_s | 0.018129 | 0.016309 |

Pure sampler counters: zero internal uniform attempts and missing-anchor Cartesian rescues are required. The separate strict runtime regression verifies the MoveIt wrapper failure path. The unrestricted baseline has no GMM session/counters.

This is a clearance sensitivity replay of previously used environments, not new independent confirmation. All path-quality results below are conditional on success. Matched-pair effects and retained sample counts are in `statistics.json`. Minimum world clearance includes the table and all robot links; it is not the RL reward or only an EE-to-cylinder distance.

The custom hard corridor is a union of oriented boxes enclosing the cutoff ellipsoids; accepted targets obey the ellipsoidal cutoff, while tree edges may enter box corners. The baseline has no GMM corridor, so its contrasts are between different feasible-region configurations. Projected proposals only approximate the Cartesian Gaussian under a local linearization. The two custom methods also differ in proposal orientation behavior.

Raw inputs and outputs: `manifest.json`, `environments/*.json`, `results.jsonl`, `trials.csv`. Primary figures are supplied as PDF and PNG. No result from this study alone establishes transfer to EV battery disassembly.
