# Saved training-height range with an identical lower table

**Matching the saved training-height range did not reduce GMM-planner failures in this two-scene test.** Among the 99 environments where both endpoint procedures passed, Cartesian + IK achieved 566/735 audited successes (77.0%), joint projection 497/735 (67.6%), and unrestricted OMPL 735/735 (100%). The lower arrangement was harder for both GMM methods even after separating endpoint-solve failures.

**The original-height scene and table are preserved.** The second arrangement is a separate clone with goals mapped to the saved training range and the identical table lowered. All other scene geometry and sampling controls are unchanged.

Completed on 28 September 2026: **5,400 request outcomes across 120 paired source environments**. Of these, 4,775 reached a MoveGroup planning action; 495 stopped at endpoint design and 130 at corridor endpoint validation. The first cohort uses ten repeats per method and arrangement; the second uses five. These are reused case-study environments, so this is sensitivity evidence rather than a new independent confirmation.

## The two arrangements

| Setting | Original control | Training-height clone |
|---|---|---|
| Goal z | 0.45–0.54 m; exact original joint endpoint | 0.10–0.30 m; corresponding quantile and new collision-aware IK |
| Table top z | 0.25 m | −0.02 m |
| Table dimensions | 0.75 × 1.00 × 0.25 m | Identical |
| Starts, goal x/y/orientation, cylinder geometry and poses | Original | Identical |
| Requested RL clearance | 0.5 / 0.7, assigned as in source | Identical assignment |
| GMM / sampling | Same checkpoint; covariance cutoff 2.0; pure methods | Identical controls |

All methods use RRTConnect with one planning attempt, a 3-second budget, no path simplification and the same dense trajectory audit.

The saved checkpoint is `2026-09-11_21-20-28_ppo_torch/checkpoints/best_agent.pt`, action scale 0.15. The table is lowered only in the clone. The robot base and cylinder remain fixed. These are controller-free MoveIt collision scenes, without Gazebo dynamics or robot execution; the unchanged cylinder is not repositioned to rest on the lower table.

![Separate arrangements](comparison/figures/scene_arrangements.png)

The fixed example is source environment 0, selected independently of outcome. Lines show prior/deformed GMR references, not planned trajectories.

## Success and endpoint feasibility

| Method | Original audited successes | Lower arrangement successes | Environment-weighted change (pp), 95% CI | Holm p |
|---|---:|---:|---:|---:|
| cartesian_ik | 859/900 | 566/900 | -31.25 [-39.42, -22.75] | 2.99997e-05 |
| joint_projected | 857/900 | 497/900 | -38.00 [-45.92, -29.83] | 2.99997e-05 |
| ompl_uniform | 898/900 | 735/900 | -17.25 [-24.00, -10.83] | 2.99997e-05 |

The fixed 2-second collision-aware goal IK procedure found valid lower-scene endpoints in **99/120 environments** (48/60 first cohort, 51/60 second cohort); original endpoints passed in all 120. The 21 endpoint-solve failures produce **165 failed outcomes per method**, with no planning action or sampler invocation. Failure within this solve budget does not prove geometric unreachability.

| Method | Original success on the 99 endpoint-feasible pairs | Lower arrangement success on the same pairs |
|---|---:|---:|
| cartesian_ik | 704/735 | 566/735 |
| joint_projected | 702/735 | 497/735 |
| ompl_uniform | 734/735 | 735/735 |

This conditional comparison is secondary; it retains the same 99 source environments in both arrangements and does not remove endpoint failures from the main analysis.

![Success comparison](comparison/figures/success_comparison.png)

## Failure mechanisms

| Method | Original failures | Lower arrangement failures |
|---|---|---|
| cartesian_ik | {"start_invalid_in_corridor": 40, "path_audit": 1} | {"planning": 132, "path_audit": 12, "goal_invalid_in_corridor": 25, "goal_ik_design": 165} |
| joint_projected | {"start_invalid_in_corridor": 40, "path_audit": 1, "planning": 2} | {"planning": 207, "goal_invalid_in_corridor": 25, "goal_ik_design": 165, "path_audit": 6} |
| ompl_uniform | {"path_audit": 2} | {"goal_ik_design": 165} |

![Failure stages](comparison/figures/failure_stages.png)

`goal_ik_design` means endpoint solving failed before planning; `*_invalid_in_corridor` means the fixed endpoint is incompatible with the deformed hard corridor; `planning` is a failed MoveGroup planning result; `path_audit` rejects a returned path after dense collision/constraint checks. Every category remains a failure. The full diagnostic counters, including valid samples in timed-out searches and missing anchors, are in `comparison/statistics.json`.

## What this means for sampler selection

Cartesian + IK had **69 more audited successes** than projection in the lower arrangement. The first cohort's within-arrangement secondary comparison gives a Cartesian success advantage of **8.83 percentage points**, 95% environment-cluster CI **[4.17, 14.00]**, Holm p **0.00183**. Its PAR2 advantage is **0.448 s** [0.206, 0.715], with the same adjusted p-value. In the reused second cohort, the observed success advantage is **5.33 points** [1.00, 10.00], but the adjusted p-value is **0.0696**, so that cohort alone does not meet the 0.05 threshold. Each of these within-arrangement corrections covers its six success/PAR2 contrasts; they are separate from the primary three height-intervention tests. These are sensitivity analyses on reused environments, not independent replications.

Projection still returns shorter joint paths among matched successes: in the original-height controls, Cartesian-minus-projected travel is **1.241 rad [0.773, 1.712]** in the first cohort and **1.466 rad [0.804, 2.149]** in the second. That path-quality advantage must be weighed against the lower arrangement's reliability loss. For a benchmark intended to cover both tested height regimes, **Cartesian + IK is now the more defensible provisional choice on observed reliability**. These data do not support a universal sampler winner or reliability superiority over unrestricted OMPL for either GMM method.

Every lower-scene planning failure generated valid custom samples: **132/132 Cartesian failures** and **207/207 projected failures**. The projected failures generated about **6.29 million valid samples** in total, and only **5/207** had an unanchored component. Thus, the main pattern is not simply an absence of accepted samples or anchors. Coverage of useful configurations and connections within the restricted corridor remains a plausible limitation requiring targeted diagnosis; these counters do not prove a unique mechanism. All these planning failures returned MoveIt code `99999` after approximately the 3-second budget. Twelve Cartesian and six projected returned paths additionally failed the dense audit and were rejected.

## Timings and path quality

| Cohort / scene | Method | Successes | Successful action median (s) | Joint travel median (rad) | EE path median (m) | Robot–world minimum clearance median (m) |
|---|---|---:|---:|---:|---:|---:|
| primary / control | cartesian_ik | 579/600 | 0.1267 | 9.9331 | 0.6922 | 0.0137 |
| primary / control | joint_projected | 579/600 | 0.1205 | 8.3332 | 0.6440 | 0.0125 |
| primary / control | ompl_uniform | 599/600 | 0.1471 | 13.3289 | 2.1793 | 0.0166 |
| primary / training | cartesian_ik | 368/600 | 0.1766 | 11.9338 | 0.9924 | 0.0081 |
| primary / training | joint_projected | 315/600 | 0.1277 | 9.0309 | 0.8418 | 0.0075 |
| primary / training | ompl_uniform | 480/600 | 0.1753 | 14.0422 | 2.3013 | 0.0145 |
| confirmation / control | cartesian_ik | 280/300 | 0.1327 | 10.7396 | 0.7255 | 0.0148 |
| confirmation / control | joint_projected | 278/300 | 0.1273 | 8.5425 | 0.6493 | 0.0134 |
| confirmation / control | ompl_uniform | 299/300 | 0.1554 | 13.4134 | 2.0585 | 0.0181 |
| confirmation / training | cartesian_ik | 198/300 | 0.1732 | 11.6900 | 0.9792 | 0.0075 |
| confirmation / training | joint_projected | 182/300 | 0.1405 | 10.2937 | 0.9019 | 0.0057 |
| confirmation / training | ompl_uniform | 255/300 | 0.1740 | 13.9857 | 2.2639 | 0.0154 |

These medians condition on audited success and are subject to survivor selection. The two height arrangements also change task geometry and path length. Matched-success differences with environment-cluster intervals, including EE–cylinder distances, are in the paired statistics. Per-arm reports contain PAR2, deformation, setup, sampling, IK and Jacobian/FK timing. Nested timers must not be added. Planning/pipeline times exclude scene/endpoint design and the offline dense audit; failed endpoint solves are not zero-cost successful plans.

## Inference and limitations

- Primary analysis compares lower arrangement minus original for each sampler. The independent unit is the source environment; all repeats stay in its cluster. Each environment has equal inferential weight despite different repeat counts, while raw pooled counts weight the first cohort more heavily.
- Pointwise 95% intervals use 20,000 bootstraps stratified by source cohort, obstacle layout and requested clearance. Two-sided sign permutations use exact enumeration for at most 16 informative clusters, otherwise 100,000 draws; Holm correction covers the three primary sampler success contrasts. Monte Carlo p-values have finite resolution. All-success intervals do not establish perfect population reliability.
- The lower arrangement changes **both goal height and table height**. It cannot isolate RL goal-height generalization, prove the checkpoint is defective, or rule out all other causes. IK branch selection also necessarily changes with the goal. No sampler, policy, cutoff, budget or sample size was tuned after seeing these planning outcomes.
- The two custom methods enforce a union-of-boxes GMM corridor; unrestricted OMPL samples the bounded joint space without that corridor. Their comparison therefore changes feasible-region configuration as well as proposal density. Projection remains a local kinematic approximation, and pure sampling excludes exploratory fallback.
- The two clearance values are assigned to different source environments. This is not a within-environment 0.5-versus-0.7 sweep. Source IDs retain their historical clearance labels; current numeric clearances are explicit in every manifest/outcome.

## Verification and reproduction

All **14 relevant unit tests passed**, the `tp_gmm` build passed, and both installed CLI help checks passed. [Verification records](verification/checks.json) include the build log, input-audit script, repository base commits and source snapshots. The four archives contain 612 files verified against their sources.

All 240 frozen arrangement scenes passed live read-back verification. The original case geometry, exact joint endpoints and prior/deformed model payloads match their source inputs. Every lower scene differs only in table z; every lower case differs only in goal z. Runtime/model hashes and recorded runner snapshots match their frozen checksums.

**Zero explicit uniform attempts, zero missing-anchor Cartesian rescues and zero projected online IK calls** were retained. The unchanged strict-wrapper binary also has the saved exhausted-sampler runtime regression. Complete counters and validation checks are in [validation.json](comparison/validation.json).

- [Paired analysis](comparison/summary.md): success contrasts, endpoint-feasible subset, failure cases, descriptive clearance/cohort breakdowns, diagnostics and PDF/PNG figures.
- Per-arm reports: [first control](primary/control/summary.md), [first lower scene](primary/training/summary.md), [second control](confirmation/control/summary.md), [second lower scene](confirmation/training/summary.md).
- [Protocol](protocol.md), [command guide](../../COMMANDS-cod.md), [endpoint/table preflight](preflight/README.md), and [previous clearance-only study](../2026-09-28-clearance-replay/README.md).
- Four `raw-COHORT-ARM.tar.xz` archives preserve every frozen scene/model/state, raw result/trajectory, source snapshot, runtime/model hashes, policy-training configuration and scene verification. Each extracts under `COHORT/ARM/`. Parent `manifest.json` files preserve the randomized order. Extract all four archives from this report directory to reconstruct the `primary/` and `confirmation/` inputs for `analyze_goal_height_study.py`. Every archived file was checked against its source by SHA-256.
- `SHA256SUMS` covers the packaged files. Custom proposal seeds and method/repeat schedules are preserved; OMPL and IK RNG streams are not bit-identical between runs. No failed outcomes were selectively rerun.

The full table-contact preflight found collisions at all 120 low goals with the original table. Candidate table tops at 0.00 and −0.02 m left five and zero table-contact cases respectively for those saved candidate IK states. The final designs then used collision-aware IK; the preflight is a geometry check, not a planner comparison.
