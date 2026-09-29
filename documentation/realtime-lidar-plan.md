# Real-time lidar milestone

Started 2026-09-28. One SUMO-controlled car, static obstacle avoidance; no PPO, moving traffic or communication in this milestone.

## Predeclared gates

- Profile the existing sample-and-hold loop before optimization. Startup and steady-state runtime must be reported separately.
- Preserve the baseline launcher and sensor behavior.
- New runtime: a documented mapping between traffic time, sensor time, wall time, and sensor pose; reject future, stale, malformed, or unaligned observations.
- Target 60 traffic seconds in 57–63 wall seconds after initialization (real-time factor 0.95–1.05 approximately), with bounded data age, not just high viewport FPS.
- Validate obstacle pass/return, blocked-lane stopping, and sensor dropout response. Preserve lane safety modes and evaluation-only obstacle truth.
- Retain all exploratory failures. Repeated runs and a GUI run are required before claiming the milestone complete. Native RTX stability remains an open risk.

Use the current Isaac Kit runtime, not a simultaneous ovrtx migration. First measure; select sensor scheduling based on evidence. Any sensor profile change must be explicit, with its coverage and timing limitations recorded.

## Evidence-driven follow-up protocol

The first fixed suite retained one >100 ms late cycle per completed case. GC profiling on the blocked case measured a 176 ms generation-2 collection overlapping the late frame. The next suite tests deferred automatic cyclic GC during the bounded <=60-second window, restoring the original collector state afterward and recording process RSS at loop start/end. This is not an unbounded service memory policy.

The initial dropout audit incorrectly scored frozen, rejected scans against the moved car's current self-filter, so old ego-body returns inflated obstacle alignment error. Contract version 2 scores alignment only for fresh, newly admitted controller packets. All stale raw packets and their ages remain recorded, and dropout still must stop safely with no new lane requests. This corrects the evaluation domain; it does not loosen the 30 cm threshold.
