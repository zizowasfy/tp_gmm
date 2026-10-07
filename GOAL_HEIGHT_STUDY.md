# Goal-height and table-height case study

Protocol fixed before measured plans. This follows the verified 0.5/0.7 clearance replay using the same 120 source environments and exact start joint states. Both clearance values, model/checkpoint, action scale, sampler settings, strict no-fallback wrapper, cutoff 2.0, RRTConnect budget (3 s), one attempt, response adapters, lack of simplification and dense path audit remain unchanged.

## Two separate scene snapshots

- **Control:** original goal z in [0.45, 0.54] m, original complete scene and table top at z=0.25 m, and original exact joint goal.
- **Training-height arrangement:** goal z mapped into [0.10, 0.30] m by preserving its original within-range quantile. Clone the original scene and lower only the table's vertical position to put its surface at z=−0.02 m. Preserve table dimensions, horizontal position, cylinder geometry/poses, starts, goal x/y and downward orientation. Solve collision-aware IK for the necessarily changed goal. Recompute TP-GMM reproduction and RL deformation with the unchanged clearance value and checkpoint.

The original data and scene are not edited. The low scene contains its own table at the lower height, not an additional high table. This implements the user's instruction to retain the original-height table and use a lower table in the new scene. It is a **combined goal-height/table-height intervention**, not a test isolating height or all possible causes of failure.

A preflight found table contacts at all 120 mapped goals with the original table. The checked candidate surface at z=0 had five remaining table contacts; z=−0.02 had none. The height choice used endpoint geometry only, before planning outcomes. These coordinates are relative to the existing robot-base/world frame. Cylinder objects remain at their source poses in this plan-only collision scene; no Gazebo dynamics or execution is involved.

## Fixed schedule and feasibility handling

Run both arrangements for the primary source cohort (60 environments × 10 repeats × 3 samplers × 2 arrangements = 3,600 requests), then both for the second source cohort (60 × 5 × 3 × 2 = 1,800). Total **5,400 request outcomes**, including explicitly recorded endpoint-design failures for which planning is not attempted. Within each source environment, randomize arrangement block order using source seed + 2809; retain the original sampler/repeat schedule and custom seeds inside each arrangement. No extra runs based on significance or selection by planner outcome.

Collision-aware goal IK gets the same 2 s endpoint solve budget as the original study, seeded by the source goal. Any failed endpoint solve/validation stays in the full denominator for every method, is identified as a design-stage failure and has no fabricated sampler statistics. Report endpoint-feasible results separately. Preserve every original start exactly. Freeze each scene, model, goal state, IK/validity evidence and source hash before planning. Read back and verify every active collision scene; abort on mismatch. The control prior and deformed GMM must match their source payloads except header time.

An IK failure within this budget does not prove that a collision-free endpoint solution is impossible. Here “endpoint-feasible” means the fixed solve/validation procedure found one; report failures as budgeted endpoint-solve failures, not proven geometric unreachability.

## Analysis

Compare training-height arrangement minus original-height control using paired source-environment identities, retaining repeats within clusters. Give environments equal inferential weight despite the two different repeat counts. Stratify 20,000 cluster bootstraps by source cohort, obstacle layout and clearance. The main family has three two-sided environment sign-permutation success tests, Holm corrected at 0.05 (100,000 draws or exact enumeration when at most 16 nonzero clusters contribute). Report raw counts, cluster intervals, corrected p-values and phase-specific/clearance-specific descriptive results. These reused environments do not provide fresh independent confirmation.

Secondary results: failures by stage, conditional endpoint-feasible comparisons, successful path quality, robot–world and EE–cylinder clearance, timings, sampler efficiency and zero-fallback counters. Path quality is conditional on success and changes in geometric task length must be considered. Matched successful runs are subject to survivor selection. Timers for IK/mapping are nested within sampling/setup timers and must not be added together. The low-goal arrangement differs in table position as well as goal height, and therefore cannot attribute an effect uniquely to RL training-height mismatch.

## Run

Use the existing `sampling_demo.launch.py` controller-free launch with `ROS_DOMAIN_ID=81`, original checkpoint and RViz disabled. From the sourced workspace root:

```bash
python3 src/tp_gmm/scripts/run_goal_height_study.py \
  --source sampling_results/2026-09-28-clearance-primary \
  --output sampling_results/2026-09-28-height-primary \
  --goal-z-range 0.1 0.3 --table-top-z -0.02
python3 src/tp_gmm/scripts/run_goal_height_study.py \
  --source sampling_results/2026-09-28-clearance-confirmation \
  --output sampling_results/2026-09-28-height-confirmation \
  --goal-z-range 0.1 0.3 --table-top-z -0.02
```

Each output contains separate `control/` and `training/` directories compatible with `analyze_sampling_study.py`, and a parent manifest recording the interleaved arrangement block order. Use `--resume` only with the same frozen design and settings; `--design-only` freezes inputs without running measured plans.

After all timed runs finish, analyze each arm with `analyze_sampling_study.py`, then combine the paired cohorts:

```bash
python3 src/tp_gmm/scripts/analyze_goal_height_study.py \
  sampling_results/2026-09-28-height-primary \
  sampling_results/2026-09-28-height-confirmation \
  --output sampling_results/2026-09-28-height-comparison
```

The combined output contains `statistics.json`, independent input verification, a Markdown report, and PDF/PNG figures for success, paired changes and failure stages. See `COMMANDS-cod.md` for the complete launch and analysis sequence.
