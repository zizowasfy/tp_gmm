# Pure sampling case study results

60 source environments, 5 repeats per method, 900 request outcomes; fixed 3 s planning budget.

| Method | Audited success | Cluster 95% CI | Mean PAR2 (s) | Success median action (s) | Median EE length (m) | Median world clearance (m) |
|---|---:|---:|---:|---:|---:|---:|
| cartesian_ik | 280/300 (93.3%) | 86.7–98.3% | 0.5358 | 0.1327 | 0.7255 | 0.0148 |
| joint_projected | 278/300 (92.7%) | 86.0–98.0% | 0.5717 | 0.1273 | 0.6493 | 0.0134 |
| ompl_uniform | 299/300 (99.7%) | 99.0–100.0% | 0.1763 | 0.1554 | 2.0585 | 0.0181 |

Paired secondary contrasts (first minus second; negative time favors first):

| First / second | Outcome | Difference | Cluster 95% CI | Holm p |
|---|---|---:|---:|---:|
| cartesian_ik / joint_projected | success (pp) | 0.6667 | [0.0000, 1.6667] | 0.748753 |
| cartesian_ik / joint_projected | PAR2 (s) | -0.0359 | [-0.0976, 0.0103] | 0.748753 |
| cartesian_ik / ompl_uniform | success (pp) | -6.3333 | [-13.0000, -1.0000] | 0.487315 |
| cartesian_ik / ompl_uniform | PAR2 (s) | 0.3595 | [0.0450, 0.7551] | 0.487315 |
| joint_projected / ompl_uniform | success (pp) | -7.0000 | [-13.6667, -1.3333] | 0.375 |
| joint_projected / ompl_uniform | PAR2 (s) | 0.3954 | [0.0654, 0.7918] | 0.375 |

All three pairwise success/PAR2 tests belong to one Holm family. Intervals are pointwise, not multiplicity-adjusted. Zero-width intervals in all-success samples do not prove perfect population reliability.

Pipeline times start after scene and endpoint design. They exclude the shared endpoint IK solve and offline dense path audit. Model-preparation wall time is included for custom methods, while the unrestricted baseline needs no deformation.

Sampler stage medians in seconds (setup and sampling contain the nested timers):

| Method | Setup | Draw loop | Online IK | Anchor IK | Jacobian | FK mapping | Valid / attempted |
|---|---:|---:|---:|---:|---:|---:|---:|
| cartesian_ik | 0.0000 | 0.0135 | 0.0130 | 0.0000 | 0.0000 | 0.0000 | 0.5444 |
| joint_projected | 0.0213 | 0.0002 | 0.0000 | 0.0212 | 0.0001 | 0.0000 | 0.1598 |
| ompl_uniform | — | — | — | — | — | — | — |

Shared model-preparation stage times (one measurement per environment; seconds):

| Stage | Mean | Median |
|---|---:|---:|
| model_load_s | 0.004313 | 0.004159 |
| reproduction_s | 0.008111 | 0.008074 |
| rl_deformation_s | 0.000968 | 0.000759 |
| deformed_regression_s | 0.001711 | 0.001708 |
| total_s | 0.016703 | 0.016331 |

Pure sampler counters: zero internal uniform attempts and missing-anchor Cartesian rescues are required. The separate strict runtime regression verifies the MoveIt wrapper failure path. The unrestricted baseline has no GMM session/counters.

This is a goal/table-height sensitivity replay of previously used environments, not new independent confirmation. All path-quality results below are conditional on success. Matched-pair effects and retained sample counts are in `statistics.json`. Minimum world clearance includes the table and all robot links; it is not the RL reward or only an EE-to-cylinder distance.

The custom hard corridor is a union of oriented boxes enclosing the cutoff ellipsoids; accepted targets obey the ellipsoidal cutoff, while tree edges may enter box corners. The baseline has no GMM corridor, so its contrasts are between different feasible-region configurations. Projected proposals only approximate the Cartesian Gaussian under a local linearization. The two custom methods also differ in proposal orientation behavior.

Raw inputs and outputs: `manifest.json`, `environments/*.json`, `results.jsonl`, `trials.csv`. Primary figures are supplied as PDF and PNG. No result from this study alone establishes transfer to EV battery disassembly.

## Replayed confirmation cohort (sensitivity analysis)

This rerun reuses the original confirmation environments for goal/table-height sensitivity, so it is not another independent confirmation. The original directional path-length tests are reproduced below as sensitivity analyses.

| Outcome (Cartesian minus projected) | Effect | 95% cluster CI | One-sided Holm p | Matched pairs / environments |
|---|---:|---:|---:|---:|
| joint_path_length | 1.46650 | [0.80419, 2.14874] | 0.000259997 | 278 / 56 |
| ee_path_length_m | 0.03641 | [-0.00669, 0.07941] | 0.0620494 | 278 / 56 |
