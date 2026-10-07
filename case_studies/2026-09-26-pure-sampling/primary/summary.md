# Pure sampling case study results

60 independent environments, 10 repeats per method, 1800 plans; fixed 3 s planning budget.

| Method | Audited success | Cluster 95% CI | Mean PAR2 (s) | Success median action (s) | Median EE length (m) | Median world clearance (m) |
|---|---:|---:|---:|---:|---:|---:|
| cartesian_ik | 569/600 (94.8%) | 88.3–99.8% | 0.4370 | 0.1248 | 0.6875 | 0.0135 |
| joint_projected | 568/600 (94.7%) | 88.3–99.5% | 0.4432 | 0.1200 | 0.6818 | 0.0123 |
| ompl_uniform | 600/600 (100.0%) | 100.0–100.0% | 0.1528 | 0.1458 | 2.0439 | 0.0166 |

Paired primary contrasts (first minus second; negative time favors first):

| First / second | Outcome | Difference | Cluster 95% CI | Holm p |
|---|---|---:|---:|---:|
| cartesian_ik / joint_projected | success (pp) | 0.1667 | [-0.3333, 0.6667] | 1 |
| cartesian_ik / joint_projected | PAR2 (s) | -0.0063 | [-0.0379, 0.0243] | 1 |
| cartesian_ik / ompl_uniform | success (pp) | -5.1667 | [-11.6667, -0.1667] | 0.625 |
| cartesian_ik / ompl_uniform | PAR2 (s) | 0.2842 | [-0.0072, 0.6586] | 0.625 |
| joint_projected / ompl_uniform | success (pp) | -5.3333 | [-11.6667, -0.5000] | 0.375 |
| joint_projected / ompl_uniform | PAR2 (s) | 0.2905 | [0.0038, 0.6575] | 0.625 |

All three pairwise success/PAR2 tests belong to one Holm family. Intervals are pointwise, not multiplicity-adjusted. Zero-width intervals in all-success samples do not prove perfect population reliability.

Sampler stage medians in seconds (setup and sampling contain the nested timers):

| Method | Setup | Draw loop | Online IK | Anchor IK | Jacobian | FK mapping | Valid / attempted |
|---|---:|---:|---:|---:|---:|---:|---:|
| cartesian_ik | 0.0000 | 0.0110 | 0.0106 | 0.0000 | 0.0000 | 0.0000 | 0.5641 |
| joint_projected | 0.0181 | 0.0001 | 0.0000 | 0.0180 | 0.0001 | 0.0000 | 0.2758 |
| ompl_uniform | — | — | — | — | — | — | — |

Shared model-preparation stage times (one measurement per environment; seconds):

| Stage | Mean | Median |
|---|---:|---:|
| model_load_s | 0.004283 | 0.004105 |
| reproduction_s | 0.008055 | 0.008002 |
| rl_deformation_s | 0.000908 | 0.000713 |
| deformed_regression_s | 0.001686 | 0.001692 |
| total_s | 0.016513 | 0.016160 |

Pure sampler counters: zero internal uniform attempts and missing-anchor Cartesian rescues are required. The separate strict runtime regression verifies the MoveIt wrapper failure path. The unrestricted baseline has no GMM session/counters.

Path quality is conditional on success; only the prespecified confirmation endpoints have confirmatory status. Matched-pair quality effects and retained sample counts are in `statistics.json`. Minimum world clearance includes the table and all robot links; it is not the RL reward or only an EE-to-cylinder distance.

The custom hard corridor is a union of oriented boxes enclosing the cutoff ellipsoids; accepted targets obey the ellipsoidal cutoff, while tree edges may enter box corners. The baseline has no GMM corridor, so its contrasts are between different feasible-region configurations. Projected proposals only approximate the Cartesian Gaussian under a local linearization. The two custom methods also differ in proposal orientation behavior.

Raw inputs and outputs: `manifest.json`, `environments/*.json`, `results.jsonl`, `trials.csv`. Primary figures are supplied as PDF and PNG. No result from this study alone establishes transfer to EV battery disassembly.
