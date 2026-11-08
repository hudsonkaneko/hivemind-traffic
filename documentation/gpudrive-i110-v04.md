# GPUDrive I-110 v04: destination-preserving route repair

November 6, 2026. **Route repair verified; short merge test completed. No training.**

The old `moveToXY(keepRoute=3)` restricted lane association to the original ramp
route. At 1.1 seconds the continuously steered car entered `sb1`, which was absent
from that route, and lost its lane association. The opt-in v04 configuration uses
`keepRoute=2` so SUMO can match the actual pose to another road without snapping
it. If SUMO truncates the route, `findRoute` rebuilds a connected route from that
road to the **unchanged destination `sb5`**, using the actual vehicle type.
`setRoute` readback must match exactly. Missing/unreachable routes fail closed;
valid internal-link routes are retained. Premature arrival on a truncated route
cannot count as reaching the goal.

The frozen weights, observations, steering, safety limits, seed, demand and
40-second maximum horizon are unchanged from v03. Historical configs and failed
results remain intact. This is external bicycle ego motion with SUMO background
traffic; it does not establish physical vehicle behavior or native safety parity.

## One short CPU run

| Metric | Result |
|---|---:|
| Route repair | At 1.1 s: `sb1` → `sb1 sb2 sb3 sb4 sb5` |
| Stable merge criterion | Met at 8.3 s (3 seconds mainline occupancy) |
| Goal reached / simulated duration | Yes / 28.5 s |
| SUMO loop wall time | 3.859 s; excludes Python startup and tests |
| Distance | 577.866 m |
| Minimum / maximum speed | 6.212 / 26.820 m/s |
| Stopped exposure / overspeed steps | 0 s / 0 |
| Contacts / native collisions / teleports | 0 / 0 / 0 |
| Route failures / unsafe proposals rejected | 0 / 0 |
| Applied decisions / steering reinterpretations | 284 / 0 |
| Changed acceleration requests | 255/284 (89.79%) |
| Peak acceleration-change jerk | 60 m/s³ |

Nonexclusive override reasons: acceleration range 251, following 137, speed
ceiling 4. The high intervention count is dominated by clipping the frozen
actor's acceleration to the existing vehicle limits; it is not a count of
near-collisions. SUMO also logged three background emergency-braking warnings.
Consequently this is **not a passing 5% intervention result or a smooth-driving
qualification**, despite successful route completion. No policy is promoted.

Maximum position error between the requested pose and SUMO readback was
5.68e-14 m. All 284 applied steering values equal the frozen actor's requests.
Validation: **30 tests plus 14 subtests pass**, including seven new route-repair
tests and the existing frozen-logit parity checks. One read-only/API-review
subagent also added the focused regression tests.

## Retained evidence

- [Compact metrics and provenance](evidence/gpudrive-i110-v04/results.json)
- [Resolved test configuration](../configs/evaluation/gpudrive-i110-v04.json)
- Full motion recording, observations, actions, source snapshots, SUMO log and
  metrics: `outputs/gpudrive-i110/v04/policy-01`.
- Same actor SHA-256:
  `f3f26475def35f375c6c72d8f8f20b2b091f77175010345dc3fa968a860521b7`.

No optimizer updates, new render, or additional scenario campaign. Next policy
work should address acceleration-contract compatibility and smoothness before
claiming acceptance; the SUMO-native MAPPO training path remains unchanged.
Google engineering-guide incremental sync is pending; the generated Markdown
mirror was not hand-edited.
