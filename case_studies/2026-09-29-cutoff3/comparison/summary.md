# Covariance cutoff sensitivity

Cutoff-only sensitivity; same original-height scenes, endpoints, clearances and GMM payloads. Reused environments, not independent confirmation.

**Cutoff 2 → 3**, keeping requested RL clearance 0.2/0.8 and original table/goal heights.

| Method | Previous successes | Replay successes | Success change (pp), 95% CI | Holm p |
|---|---:|---:|---:|---:|
| cartesian_ik | 853/900 | 899/900 | 5.17 [1.75, 9.25] | 0.0292969 |
| joint_projected | 850/900 | 897/900 | 5.25 [1.75, 9.25] | 0.0292969 |
| ompl_uniform | 900/900 | 900/900 | 0.00 [0.00, 0.00] | 1 |

Raw counts pool different repeat counts; inference gives each source environment equal weight. Repeats stay within environment clusters. Three primary two-sided success tests have one Holm correction. Pointwise intervals can differ from corrected-test conclusions. All-success intervals do not establish perfect population reliability.

![Success comparison](figures/success_comparison.png)

| Method | Previous failure stages | Replay failure stages |
|---|---|---|
| cartesian_ik | {"start_invalid_in_corridor": 45, "path_audit": 2} | {"path_audit": 1} |
| joint_projected | {"start_invalid_in_corridor": 45, "planning": 3, "path_audit": 2} | {"path_audit": 2, "planning": 1} |
| ompl_uniform | {} | {} |

![Failure stages](figures/failure_stages.png)

![Matched-success clearance](figures/clearance_change.png)

Changing cutoff widens both the truncated proposal support and the enclosing hard box corridor. Covariance matrices and mixture weights do not change. The unrestricted baseline ignores cutoff, so its changes reflect stochastic rerun variability. No fallback, additional IK rescue, policy change or geometry change was introduced.

Secondary runtime/PAR2, matched-success path length and clearance effects, case-level failures, per-clearance/per-cohort counts and purity checks are in `statistics.json`. Timing drift and independent OMPL/IK RNG streams limit before/after runtime attribution. Conditional path-quality effects have survivor selection. See per-cohort reports for sampler stage/preparation times and distribution diagnostics.
