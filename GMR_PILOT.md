# GMR proposal pilot — 24 September 2026

The first paired experiment supports keeping the GMR sampler as an **optional proposal**, with hybrid mixing as the initial practical choice. It does not justify replacing ordinary GMM sampling or claiming a general speedup.

## Setup

- Controller-free Panda / MoveIt / RRTConnect, isolated ROS domain 81; no Gazebo execution.
- Three existing scenes: open, central obstacle, offset obstacle.
- Five measured repeats plus one warmup per scene and variant: **120 measured attempts**, 24 excluded warmups.
- Eight variants: GMM; GMR-only and 80/20 GMR/GMM hybrid at standard deviations 5, 15 and 30 mm; direct-reference sequential IK.
- GMR cutoff 2; original covariance corridor cutoff 2; extra constrained-uniform fallback disabled.
- Identical endpoint joint states and request-matched model/reference within each paired repeat. Variant order randomized, seed 42. Internal OMPL/IK randomness is not fully controlled.
- Existing checkpoint `2026-09-11_21-20-28_ppo_torch/checkpoints/best_agent.pt`, SHA-256 `d9ec51ff464b93f8ad87e4d5dc2b3bb649f89dbb5fa4155b5df46ed788d2e5ba`, action scale 0.15. No retraining.
- Three-second planning budget; sample/path visualization disabled.

## Results

Median action latencies include unsuccessful attempts. Reference deviation and EE path lengths use successful paths only. Latency is not the entire experiment wall time; post-hoc path audits and result writing are excluded from RRT action latency. Direct-reference IK includes service-based validation overhead.

| Variant | Success | Action median (s) | Valid samples / attempts | Mean reference deviation, median (m) | EE path length, median (m) |
|---|---:|---:|---:|---:|---:|
| GMM | 15/15 | 0.1431 | 0.502 | 0.0658 | 0.7752 |
| GMR, 5 mm | 11/15 | 0.1992 | 0.595 | 0.0610 | 0.8267 |
| Hybrid, 5 mm | 15/15 | 0.1421 | 0.591 | 0.0662 | 0.7190 |
| GMR, 15 mm | 15/15 | 0.1900 | 0.599 | 0.0612 | 0.8195 |
| Hybrid, 15 mm | 15/15 | 0.1082 | 0.638 | 0.0579 | 0.6628 |
| GMR, 30 mm | 15/15 | 0.1289 | 0.689 | 0.0427 | 0.6927 |
| Hybrid, 30 mm | 15/15 | 0.1483 | 0.604 | 0.0581 | 0.6774 |
| Direct reference IK | 5/15 | 0.1107 | unavailable | unavailable | 0.6594 |

## Interpretation

1. **Excessively narrow proposals can hurt connectivity.** All four 5 mm GMR planning failures occurred in the central-obstacle scene. Acceptance was nevertheless higher than the GMM baseline. Accepted targets alone do not establish useful tree connections or a feasible route from the particular endpoint configurations.
2. **Hybrid sampling is a promising default for further evaluation.** At 15 mm it solved all cases and had a roughly 24% lower median RRT action latency than GMM in this small run. This is an observed median difference, not a statistically established improvement.
3. **A wider GMR neighborhood can follow the reference better than a narrower one.** The 30 mm GMR proposal produced the lowest median reference deviation here. Proposal width does not monotonically determine final path deviation because RRT and simplification connect in joint space.
4. **The reference is not directly executable in every scene.** Sequential IK failed in all five central-obstacle repeats; joint-edge validation rejected all five offset-obstacle repeats. The reference link-origin paths still had positive cylinder clearance in all scenes. Those distances do not establish whole-robot feasibility or compatible IK branches.
5. **Reference endpoints need attention.** The first reference points were 26–37 mm from the requested starts. Final reference points were 93 mm (open), 93 mm (central obstacle) and 50 mm (offset obstacle) from the goals. The first prototype intentionally uses the rollout unchanged; RRT must bridge those differences. This is a possible contributor to narrow-neighborhood failures, not a demonstrated causal explanation. A future endpoint-conditioning/connector ablation should be separate from sampling-width changes.
6. **This is not yet an EV battery benchmark.** Three scene settings and five repeats are a pilot. Further evaluation should include unseen layouts, difficult endpoint branches, deliberately misleading references and a larger seed set. Whole-robot clearance and success should take priority over acceptance alone.

## Artifacts and reproduction

Full raw data, model/reference snapshots, trajectories, summary, CSV and PNG/PDF figures are in the workspace directory `gmr_results/2026-09-24-pilot/` (outside the source Git repositories).

```bash
# With the isolated sampling_demo launch and the checkpoint above:
ros2 run tp_gmm compare_gmr_proposals.py --repeats 5 --warmup 1 \
  --gmr-stddevs 0.005 0.015 0.03 --planning-time 3 \
  --output gmr_results/new_pilot
```

Use `config/sampling_gmr_hybrid.json` as the initial experiment preset. Both original GMM mapping modes remain available. See [GMR_SAMPLING.md](GMR_SAMPLING.md) for the API, visualization and tests.
