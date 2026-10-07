# Pure sampling case study results

60 source environments, 5 repeats per method, 900 request outcomes; fixed 3 s planning budget.

| Method | Audited success | Cluster 95% CI | Mean PAR2 (s) | Success median action (s) | Median EE length (m) | Median world clearance (m) |
|---|---:|---:|---:|---:|---:|---:|
| cartesian_ik | 198/300 (66.0%) | 55.3–76.0% | 2.2278 | 0.1732 | 0.9792 | 0.0075 |
| joint_projected | 182/300 (60.7%) | 50.7–70.3% | 2.5283 | 0.1405 | 0.9019 | 0.0057 |
| ompl_uniform | 255/300 (85.0%) | 76.7–93.3% | 1.0535 | 0.1740 | 2.2639 | 0.0154 |

Paired secondary contrasts (first minus second; negative time favors first):

| First / second | Outcome | Difference | Cluster 95% CI | Holm p |
|---|---|---:|---:|---:|
| cartesian_ik / joint_projected | success (pp) | 5.3333 | [1.0000, 10.0000] | 0.0696193 |
| cartesian_ik / joint_projected | PAR2 (s) | -0.3006 | [-0.5359, -0.0844] | 0.0696193 |
| cartesian_ik / ompl_uniform | success (pp) | -19.0000 | [-28.3333, -10.6667] | 0.000366211 |
| cartesian_ik / ompl_uniform | PAR2 (s) | 1.1743 | [0.6944, 1.7092] | 5.99994e-05 |
| joint_projected / ompl_uniform | success (pp) | -24.3333 | [-34.0000, -15.6667] | 5.99994e-05 |
| joint_projected / ompl_uniform | PAR2 (s) | 1.4749 | [0.9676, 2.0270] | 5.99994e-05 |

All three pairwise success/PAR2 tests belong to one Holm family. Intervals are pointwise, not multiplicity-adjusted. Zero-width intervals in all-success samples do not prove perfect population reliability.

Pipeline times start after scene and endpoint design. They exclude the shared endpoint IK solve and offline dense path audit. Model-preparation wall time is included for custom methods, while the unrestricted baseline needs no deformation.

Sampler stage medians in seconds (setup and sampling contain the nested timers):

| Method | Setup | Draw loop | Online IK | Anchor IK | Jacobian | FK mapping | Valid / attempted |
|---|---:|---:|---:|---:|---:|---:|---:|
| cartesian_ik | 0.0000 | 0.0347 | 0.0342 | 0.0000 | 0.0000 | 0.0000 | 0.5784 |
| joint_projected | 0.0195 | 0.0007 | 0.0000 | 0.0192 | 0.0001 | 0.0000 | 0.2224 |
| ompl_uniform | — | — | — | — | — | — | — |

Shared model-preparation stage times (one measurement per environment; seconds):

| Stage | Mean | Median |
|---|---:|---:|
| model_load_s | 0.004900 | 0.004234 |
| reproduction_s | 0.008095 | 0.008085 |
| rl_deformation_s | 0.000780 | 0.000780 |
| deformed_regression_s | 0.001715 | 0.001713 |
| total_s | 0.017095 | 0.016444 |

Pure sampler counters: zero internal uniform attempts and missing-anchor Cartesian rescues are required. The separate strict runtime regression verifies the MoveIt wrapper failure path. The unrestricted baseline has no GMM session/counters.

This is a goal/table-height sensitivity replay of previously used environments, not new independent confirmation. All path-quality results below are conditional on success. Matched-pair effects and retained sample counts are in `statistics.json`. Minimum world clearance includes the table and all robot links; it is not the RL reward or only an EE-to-cylinder distance.

The custom hard corridor is a union of oriented boxes enclosing the cutoff ellipsoids; accepted targets obey the ellipsoidal cutoff, while tree edges may enter box corners. The baseline has no GMM corridor, so its contrasts are between different feasible-region configurations. Projected proposals only approximate the Cartesian Gaussian under a local linearization. The two custom methods also differ in proposal orientation behavior.

Raw inputs and outputs: `manifest.json`, `environments/*.json`, `results.jsonl`, `trials.csv`. Primary figures are supplied as PDF and PNG. No result from this study alone establishes transfer to EV battery disassembly.

## Replayed confirmation cohort (sensitivity analysis)

This rerun reuses the original confirmation environments for goal/table-height sensitivity, so it is not another independent confirmation. The original directional path-length tests are reproduced below as sensitivity analyses.

| Outcome (Cartesian minus projected) | Effect | 95% cluster CI | One-sided Holm p | Matched pairs / environments |
|---|---:|---:|---:|---:|
| joint_path_length | 0.91621 | [-0.05145, 1.93745] | 0.102479 | 177 / 41 |
| ee_path_length_m | -0.02153 | [-0.09031, 0.04733] | 0.721643 | 177 / 41 |

Endpoint-design failures remain in the full denominator and receive the failure penalty. No planning action or sampler was called for these records. Their zero action/stage durations mean not attempted, not fast planning. Shared endpoint IK preparation is outside planning timers. Conditional results on environments that passed endpoint solving are supplied by the paired goal-height analyzer.
