# Pure sampling case study results

60 source environments, 5 repeats per method, 900 request outcomes; fixed 3 s planning budget.

| Method | Audited success | Cluster 95% CI | Mean PAR2 (s) | Success median action (s) | Median EE length (m) | Median world clearance (m) |
|---|---:|---:|---:|---:|---:|---:|
| cartesian_ik | 300/300 (100.0%) | 100.0–100.0% | 0.1441 | 0.1335 | 0.8318 | 0.0160 |
| joint_projected | 298/300 (99.3%) | 98.3–100.0% | 0.1920 | 0.1358 | 0.7149 | 0.0153 |
| ompl_uniform | 300/300 (100.0%) | 100.0–100.0% | 0.1579 | 0.1544 | 2.1462 | 0.0198 |

Paired secondary contrasts (first minus second; negative time favors first):

| First / second | Outcome | Difference | Cluster 95% CI | Holm p |
|---|---|---:|---:|---:|
| cartesian_ik / joint_projected | success (pp) | 0.6667 | [0.0000, 1.6667] | 1 |
| cartesian_ik / joint_projected | PAR2 (s) | -0.0479 | [-0.1102, -0.0017] | 0.389596 |
| cartesian_ik / ompl_uniform | success (pp) | 0.0000 | [0.0000, 0.0000] | 1 |
| cartesian_ik / ompl_uniform | PAR2 (s) | -0.0138 | [-0.0204, -0.0070] | 0.00221998 |
| joint_projected / ompl_uniform | success (pp) | -0.6667 | [-1.6667, 0.0000] | 1 |
| joint_projected / ompl_uniform | PAR2 (s) | 0.0341 | [-0.0129, 0.0976] | 1 |

All three pairwise success/PAR2 tests belong to one Holm family. Intervals are pointwise, not multiplicity-adjusted. Zero-width intervals in all-success samples do not prove perfect population reliability.

Pipeline times start after scene and endpoint design. They exclude the shared endpoint IK solve and offline dense path audit. Model-preparation wall time is included for custom methods, while the unrestricted baseline needs no deformation.

Sampler stage medians in seconds (setup and sampling contain the nested timers):

| Method | Setup | Draw loop | Online IK | Anchor IK | Jacobian | FK mapping | Valid / attempted |
|---|---:|---:|---:|---:|---:|---:|---:|
| cartesian_ik | 0.0000 | 0.0092 | 0.0090 | 0.0000 | 0.0000 | 0.0000 | 0.5291 |
| joint_projected | 0.0233 | 0.0001 | 0.0000 | 0.0230 | 0.0001 | 0.0000 | 0.1262 |
| ompl_uniform | — | — | — | — | — | — | — |

Shared model-preparation stage times (one measurement per environment; seconds):

| Stage | Mean | Median |
|---|---:|---:|
| model_load_s | 0.006154 | 0.005014 |
| reproduction_s | 0.010947 | 0.009791 |
| rl_deformation_s | 0.001196 | 0.001007 |
| deformed_regression_s | 0.002463 | 0.001917 |
| total_s | 0.022972 | 0.021130 |

Pure sampler counters: zero internal uniform attempts and missing-anchor Cartesian rescues are required. The separate strict runtime regression verifies the MoveIt wrapper failure path. The unrestricted baseline has no GMM session/counters.

This is a covariance-cutoff sensitivity replay of previously used environments, not new independent confirmation. All path-quality results below are conditional on success. Matched-pair effects and retained sample counts are in `statistics.json`. Minimum world clearance includes the table and all robot links; it is not the RL reward or only an EE-to-cylinder distance.

The custom hard corridor is a union of oriented boxes enclosing the cutoff ellipsoids; accepted targets obey the ellipsoidal cutoff, while tree edges may enter box corners. The baseline has no GMM corridor, so its contrasts are between different feasible-region configurations. Projected proposals only approximate the Cartesian Gaussian under a local linearization. The two custom methods also differ in proposal orientation behavior.

Raw inputs and outputs: `manifest.json`, `environments/*.json`, `results.jsonl`, `trials.csv`. Primary figures are supplied as PDF and PNG. No result from this study alone establishes transfer to EV battery disassembly.

## Replayed confirmation cohort (sensitivity analysis)

This rerun reuses the original confirmation environments for covariance-cutoff sensitivity, so it is not another independent confirmation. The original directional path-length tests are reproduced below as sensitivity analyses.

| Outcome (Cartesian minus projected) | Effect | 95% cluster CI | One-sided Holm p | Matched pairs / environments |
|---|---:|---:|---:|---:|
| joint_path_length | 1.54810 | [1.00296, 2.09427] | 3.99996e-05 | 298 / 60 |
| ee_path_length_m | 0.07286 | [0.03484, 0.11058] | 0.00133999 | 298 / 60 |
