# Pure sampling case study results

60 source environments, 10 repeats per method, 1800 request outcomes; fixed 3 s planning budget.

| Method | Audited success | Cluster 95% CI | Mean PAR2 (s) | Success median action (s) | Median EE length (m) | Median world clearance (m) |
|---|---:|---:|---:|---:|---:|---:|
| cartesian_ik | 368/600 (61.3%) | 50.7–71.7% | 2.5121 | 0.1766 | 0.9924 | 0.0081 |
| joint_projected | 315/600 (52.5%) | 42.8–62.2% | 2.9606 | 0.1277 | 0.8418 | 0.0075 |
| ompl_uniform | 480/600 (80.0%) | 70.0–88.3% | 1.3467 | 0.1753 | 2.3013 | 0.0145 |

Paired primary contrasts (first minus second; negative time favors first):

| First / second | Outcome | Difference | Cluster 95% CI | Holm p |
|---|---|---:|---:|---:|
| cartesian_ik / joint_projected | success (pp) | 8.8333 | [4.1667, 14.0000] | 0.00183105 |
| cartesian_ik / joint_projected | PAR2 (s) | -0.4485 | [-0.7151, -0.2063] | 0.00183105 |
| cartesian_ik / ompl_uniform | success (pp) | -18.6667 | [-28.0000, -10.3333] | 5.99994e-05 |
| cartesian_ik / ompl_uniform | PAR2 (s) | 1.1655 | [0.6863, 1.6949] | 5.99994e-05 |
| joint_projected / ompl_uniform | success (pp) | -27.5000 | [-36.8375, -18.5000] | 5.99994e-05 |
| joint_projected / ompl_uniform | PAR2 (s) | 1.6139 | [1.0924, 2.1596] | 5.99994e-05 |

All three pairwise success/PAR2 tests belong to one Holm family. Intervals are pointwise, not multiplicity-adjusted. Zero-width intervals in all-success samples do not prove perfect population reliability.

Pipeline times start after scene and endpoint design. They exclude the shared endpoint IK solve and offline dense path audit. Model-preparation wall time is included for custom methods, while the unrestricted baseline needs no deformation.

Sampler stage medians in seconds (setup and sampling contain the nested timers):

| Method | Setup | Draw loop | Online IK | Anchor IK | Jacobian | FK mapping | Valid / attempted |
|---|---:|---:|---:|---:|---:|---:|---:|
| cartesian_ik | 0.0000 | 0.0259 | 0.0254 | 0.0000 | 0.0000 | 0.0000 | 0.6115 |
| joint_projected | 0.0155 | 0.0003 | 0.0000 | 0.0153 | 0.0001 | 0.0000 | 0.2394 |
| ompl_uniform | — | — | — | — | — | — | — |

Shared model-preparation stage times (one measurement per environment; seconds):

| Stage | Mean | Median |
|---|---:|---:|
| model_load_s | 0.004404 | 0.004239 |
| reproduction_s | 0.008083 | 0.008034 |
| rl_deformation_s | 0.000792 | 0.000766 |
| deformed_regression_s | 0.001702 | 0.001704 |
| total_s | 0.016578 | 0.016443 |

Pure sampler counters: zero internal uniform attempts and missing-anchor Cartesian rescues are required. The separate strict runtime regression verifies the MoveIt wrapper failure path. The unrestricted baseline has no GMM session/counters.

This is a goal/table-height sensitivity replay of previously used environments, not new independent confirmation. All path-quality results below are conditional on success. Matched-pair effects and retained sample counts are in `statistics.json`. Minimum world clearance includes the table and all robot links; it is not the RL reward or only an EE-to-cylinder distance.

The custom hard corridor is a union of oriented boxes enclosing the cutoff ellipsoids; accepted targets obey the ellipsoidal cutoff, while tree edges may enter box corners. The baseline has no GMM corridor, so its contrasts are between different feasible-region configurations. Projected proposals only approximate the Cartesian Gaussian under a local linearization. The two custom methods also differ in proposal orientation behavior.

Raw inputs and outputs: `manifest.json`, `environments/*.json`, `results.jsonl`, `trials.csv`. Primary figures are supplied as PDF and PNG. No result from this study alone establishes transfer to EV battery disassembly.

Endpoint-design failures remain in the full denominator and receive the failure penalty. No planning action or sampler was called for these records. Their zero action/stage durations mean not attempted, not fast planning. Shared endpoint IK preparation is outside planning timers. Conditional results on environments that passed endpoint solving are supplied by the paired goal-height analyzer.
