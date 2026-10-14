# RL clearance sweep

Run status: **complete**. Completed planning rows: 150.

Clearance input is normalized. Metrics use metres. The training reward reference distance and finite-cylinder surface distance are distinct.
Shaded bands show the interquartile range across environments, not confidence intervals. Failed plans are included in success rates and excluded from successful-path metrics.
OMPL uniform has no GMM constraints, so clearance does not affect its planning request; any apparent trend is planner randomness.

| Sampler | Success | Deformed adjacent nondecreasing pairs | Planned adjacent nondecreasing pairs |
|---|---:|---:|---:|
| Cartesian + IK | 50/50 | 97.8% of 45 pairs | 64.4% of 45 pairs |
| Joint projection | 50/50 | 97.8% of 45 pairs | 64.4% of 45 pairs |
| OMPL uniform joint space | 50/50 | 97.8% of 45 pairs | 80.0% of 45 pairs |

Monotonicity allows 2 mm numerical/planning variation and uses adjacent observed levels only; missing levels are not bridged.
Original/deformed curves are DSGMR trajectories, not collision-checked robot motions. Planned distances measure the named end-effector origin, not the full robot surface.
The additional interior metric excludes the first/last configured fractions of arc length; full-path minima are always retained.
FK validity is sampled at the recorded joint step. Polyline clearance is sampled at the recorded Cartesian step; neither is a continuous collision certificate.
Environment generation is conditioned on valid endpoint IK; every rejected candidate and the full generation ranges are retained. This is a generalization evaluation, not proof of universal generalization.
Sampler stage timers overlap: do not add IK/Jacobian/validity sub-timers to their setup/sampling totals. Analysis and FK-query time is recorded separately.
