# GPUDrive continuous steering repair — v03

November 6, 2026. **Adapter repair verified; transfer remains unqualified. No training.**

The v02 steering-to-lane adapter discarded the actor's continuous steering.
Repeated braking while its lane requests were unavailable then stopped the car.
v03 returns to the original continuous bicycle bridge with the same frozen
weights and observations. Every applied steering value now equals the actor's
request. SUMO still runs the background traffic; this is an externally positioned
ego, not a policy running inside GPUDrive or a native SUMO lane-action policy.

## Changes and limits

- Added opt-in `continuous_safety` to the original evaluator; historical unguarded
  configuration remains available. v02 code and failed evidence are preserved.
- Retained the 60 mph ceiling (26.8224 m/s), lower current legal limits, shared
  following/lane-end acceleration bounds, and live minor-junction link checks.
  Bounded acceleration and prevented braking from reversing/bouncing through zero.
- Preserved steering exactly. Sampled full-footprint road/contact proposals every
  20 ms using constant-velocity traffic prediction; rejected proposals end the
  diagnostic instead of substituting a scripted driver. These checks are **not
  equivalent to the complete SUMO/MAPPO native safety stack**, and are not a
  continuous collision-avoidance guarantee.
- Added explicit recovery of missing SUMO lane metadata only when a route lane
  actually contains the front bumper. Unknown route association ends the test.
  No vehicle is snapped onto a lane to hide the mismatch.
- Saved raw observations/actions, source snapshots, hashes, metrics and motion
  recordings. No 3D rendering or weight changes.

## Bounded result

One requested 40-second CPU diagnostic, with a repair retry, ended early:

| Measurement | Final attempt |
|---|---:|
| Simulated duration | 1.1 s |
| SUMO loop wall time | 0.687 s |
| Distance travelled | 8.269 m |
| Speed range | 7.679–8.612 m/s |
| Stopped exposure / overspeed steps | 0 s / 0 |
| Contacts / native collision events / teleports | 0 / 0 / 0 |
| Applied decisions / steering reinterpretations | 10 / 0 |
| Changed acceleration decisions | 1 / 10 |
| Stable merge / goal | Neither |
| Terminal reason | `lane_association_unavailable` |

The actor now changes action indices (4, 33, 63, 68), rather than repeating the
v02 braking action under discarded steering. At 1.1 s it turns onto the upstream
mainline: its front bumper is 0.630 m from `sb1_0`, but 3.087 m from `mlk_on_0`.
`sb1` is outside its declared `mlk_on sb2 sb3 sb4 sb5` SUMO route. SUMO reports no
lane and the conservative route recovery refuses to fabricate one. Thus the
steering conversion is fixed, but **the route/motion transfer is not solved**;
this short trace does not establish highway driving, successful merging, or
improved policy quality. It is not a passing safety-rate result (10% over only
10 decisions). Historical v02 used different demand/motion and is not a paired
comparison.

The first attempt (`policy-01`) raised an unhandled graph error at the same pose;
its evidence and 0.750 s loop time are preserved. The retry (`policy-02`) handles
the missing association explicitly. Both runs together used 1.437 s in their
SUMO loops; this excludes Python imports, tests and development time.

## Evidence and next decision

- [Published compact metrics](evidence/gpudrive-i110-v03/results.json)
- [Configuration](../configs/evaluation/gpudrive-i110-v03.json)
- Full local artifacts: `outputs/gpudrive-i110/v03/policy-01` and `policy-02`.
- Validation: **23 tests plus 4 subtests passed**, including frozen-logit parity,
  continuous motion, limits, following, footprint checks and lane recovery.
  All 10 saved steering requests equal their applied values exactly.
- Existing actor SHA-256:
  `f3f26475def35f375c6c72d8f8f20b2b091f77175010345dc3fa968a860521b7`.

No policy is promoted. Further GPUDrive transfer would require deliberate route
reassociation for continuous road transitions and renewed safety validation.
Avoid further tuning/render campaigns merely to make this actor look successful;
the existing SUMO-native MAPPO path remains the project's primary training path.
Google engineering-guide synchronization for this incremental note is pending;
the generated mirror was not hand-edited.
