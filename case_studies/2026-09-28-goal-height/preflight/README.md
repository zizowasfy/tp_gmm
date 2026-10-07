> Historical preflight note, written before the scene decision and planning runs. The user subsequently chose to preserve the original table for the control and lower only the cloned training-height scene. The completed study therefore tests a combined goal/table-height intervention; see [the final report](../README.md). The original note follows.

# Goal-height feasibility preflight

The same 120 source environments from the verified 0.5/0.7 clearance replay were examined with only Cartesian goal z remapped from [0.45, 0.54] into the saved training range [0.10, 0.30], preserving its original quantile. Goal x/y, downward orientation, start, cylinder and table geometry were unchanged. Every live scene passed the existing read-back check.

All 120 goals admitted an unconstrained IK solution, but all 120 had table contacts involving the hand or fingers. The table top is at z=0.25 m. Because the gripper's geometry at a fixed pose is independent of the arm's redundant IK configuration, finding another arm IK solution cannot remove these hand/table intersections. These are physical endpoint collisions, not GMM corridor failures or sampling failures.

No new case-study planning outcomes have been generated. A useful height-range comparison requires an explicit scene decision. If table geometry changes, a new original-height control under that same geometry is needed to separate table and height effects. Clearance remains 0.5/0.7, cutoff 2.0, pure samplers and strict no-fallback behavior.

`results.json` contains the mapped poses, IK solutions, collision contacts and scene verification for each environment. `summary.json` contains counts and ranges. `check_goal_height.py` reproduces the preflight against the isolated laboratory. The saved checkpoint training configuration is included.
