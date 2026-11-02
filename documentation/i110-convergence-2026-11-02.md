# I-110 convergence and jerk-feasibility diagnostic — 2 November 2026

Completed by Codex for Sam after the repository-review roadmap and “Let’s continue.”

Current topology exposes the two late-ramp targets before leader entry, but the 120 m conditional stopping margins are insufficient in ramp-44 and fragile in ramp-50. Expanded 240 m context offers earlier supported projections; it is an additional information requirement, not demonstrated sensing or a validated control fix. A deficit marks failure of this response profile, not proof that no other response is possible.

## What changed

`traffic/i110_convergence.py` adds an offline current-state path projection and a continuous jerk-bounded stopping envelope. The existing controller, graph actor ABI, SUMO safety modes and all recorded trajectories are unchanged. The reporter reads only the six frozen development cases (seed 732001, 0.1 s steps, 90 s primary horizon plus original separate drain). No actor was loaded and no simulation or training ran.

The network connects `mlk_on_0` through an internal lane into auxiliary lane `sb2_0`, beside through lane `sb2_1`. The projection identifies this conditional lateral interaction before native leader entry; it does not infer that a ramp car will change lanes. Static reachability includes branches without consuming another vehicle’s planned route. Internal-link lengths and edge-position resets are included.

## Frozen method and response assumptions

The plan was saved and the activity registered before new derivation. Both 120 m and 240 m Euclidean ranges, a 300 m directed path limit, both lead-motion assumptions and both gap definitions were retained. The 240 m result requires additional available state; no sensor or communications capability at that range is established. The 120 m result has the same nominal radius as prior work, but a different candidate/distance rule, so it is not an isolated range-matched ablation.

A common non-internal edge with same/adjacent reachable lanes supplies an anchor. Signed distances to its start produce a virtual front-position difference and projected gap. The closest such edge by summed forward distance wins, with deterministic ties. Different-path gaps are conditional projections, not observed longitudinal clearances. Positive front-position differences receive an envelope; rear and exactly side-by-side candidates remain unassessed. Unknown lane context or an initial state outside the bounded stopping profile is never labeled safe.

The ego holds its current acceleration for 0.1 s, ramps acceleration down at 2 m/s³ to no less than −2.5 m/s², holds as needed, and ramps it back to zero at 2 m/s³ while reaching zero speed. This avoids a hidden acceleration jump at rest. Initial deceleration already stronger than the comfort bound, or insufficient speed to release existing braking smoothly, yields an unknown profile.

After the reaction period, with speed v and acceleration a, the triangular peak deceleration is sqrt(2v + a²/2); it is capped at 2.5 m/s² and a hold is added if needed. Piecewise integration and roots of the clearance derivative locate continuous-time extrema. Required projected clearance is 2.5 m plus max over time of [ego displacement − other displacement + h × ego speed], for h = 0 or 2 s. Tests independently integrate the jerk inputs and compare against these analytic extrema.

The other vehicle either retains its current speed or brakes immediately at 9 m/s² to rest. These are assumptions. h0 preserves only a 2.5 m residual; h2 also preserves a 2 s dynamic gap. No full traffic interaction, native shield response, perception delay beyond the declared reaction, or uncertain merge timing is simulated. A positive margin supports this conditional profile, not guaranteed safety.

## Late-ramp results

All times below concern the actual later leader, selected only during retrospective scoring after the signal journal was written. First support is distinct from the first negative-margin warning.

| Case | Range m | First supported time s | Lead before native leader entry s | Projected gap m | Constant speed h0 margin m | Constant speed h2 margin m | Hard brake h0 margin m |
|---|---:|---:|---:|---:|---:|---:|---:|
| ramp-44 | 120 | 51.9 | 0.6 | 113.424 | -10.564 | -18.873 | -27.519 |
| ramp-44 | 240 | 46.3 | 6.2 | 233.592 | 135.359 | 130.522 | 135.359 |
| ramp-50 | 120 | 57.6 | 1.8 | 114.758 | 5.708 | -5.612 | -25.252 |
| ramp-50 | 240 | 51.4 | 8.0 | 234.481 | 147.879 | 143.041 | 147.877 |

At 120 m, ramp-44 first exposes the target at 51.9 s, before leader entry at 52.5 s, but the h0 constant-speed profile needs 10.564 m more than the projection provides. Ramp-50 first exposes its target at 57.6 s, before entry at 59.4 s; the +5.708 m h0 constant-speed reserve becomes negative with a 2 s headway or hard-braking assumption. Its h0 margin first turns negative at 57.8 s on the unchanged trajectory.

At 240 m, the targets are supported 6.2 and 8.0 s before native leader entry. Those early positive margins are available context, not deficit warnings. Ramp-44’s constant-speed h0 deficit begins at 50.8 s, while it is still outside 120 m; ramp-50’s h0 constant-speed deficit still begins at 57.8 s, while the hard-brake h0 deficit begins at 56.5 s. Waiting for a margin to become negative is not itself a proactive comfort policy.

Recorded peak jerks remain 48.1704 and 47.1956 m/s³. This diagnostic has not reduced either value. Full episode and leader-switch scoring is retained in `analysis-v1/summary.json`; these headline targets do not replace the other events.

## Cut-in and control exposure

The cut-in target is already an adjacent forward candidate at 0.1 s. Constant-speed h0 produces no target deficit before the 2.2 s jerk peak, while h2 first flags a deficit at 0.8 s and hard-brake h0 at 0.1 s. All 80 cut-in on/off signal records through 2.0 s are identical, so these signals cannot identify the future scripted lane change.

| Run | Controlled sample seconds | 120 m constant h0 deficit seconds | 120 m constant h2 deficit seconds | 120 m hard brake h0 deficit seconds | Unknown profile/context seconds |
|---|---:|---:|---:|---:|---:|
| ramp-0 | 136.4 | 0.0 | 0.0 | 0.0 | 0.0 |
| ramp-38 | 136.8 | 9.9 | 31.6 | 32.2 | 0.0 |
| ramp-44 | 142.3 | 6.8 | 24.0 | 31.6 | 3.1 |
| ramp-50 | 140.8 | 6.3 | 6.3 | 24.1 | 2.8 |
| cutin-smooth_acc-0 | 139.6 | 10.3 | 38.1 | 50.1 | 0.0 |
| cutin-smooth_acc-1 | 140.2 | 19.5 | 65.8 | 77.0 | 0.0 |

These are exposure counts, not labeled false positives. In the cut-in-off control, h0 constant-speed deficits occupy 10.3 vehicle-seconds; h2 occupies 38.1, and hard-brake h0 50.1. Ordinary ramp-0 has no positive projected-front candidates, so its zero count is absence of this assessment—not demonstrated safety. It also excludes the exactly side-by-side zero-gap cases that contributed to the earlier nearest-neighbor screen. Changing both the candidate semantics and the envelope prevents a claim of improved warning precision from these totals.

## Verification and provenance

- 37 targeted tests passed: topology/internal links, unknowns, bounded braking continuity, exact terminal state, closed-form stopping distance, independent numerical integration, and absence of route/event-label dependence; existing graph/anticipation tests also pass.
- Immutable source-run inputs, outputs, copied source and network identity verified before each derivation. Both fresh analysis editions completed; all six journals and the complete summary match byte-for-byte.
- Original full primary/drain, role/origin, ramp, external-wait, collision and event ledgers compare exactly with the preceding analysis. There are no new counterfactual burden measurements.
- Plan: `experiments/configs/i110_convergence_v1.json`; raw package: `outputs/i110-convergence-20261102/`; compact record: `documentation/evidence/i110-convergence-20261102.json`. Raw traces and signal journals are ignored local artifacts, not included in a public clone.
- Tests: `outputs/i110-convergence-20261102/tests-v1.xml`; replay/prefix/ledger verification: `verification.json`, generated by the retained `verify.py`. Report tables and compact evidence were generated by `close-records.py` from verified results. No derived failures were discarded.

Reproduce using the isolated traffic Python from the repository root, choosing a fresh output directory:

```text
python -m pytest -q tests/test_i110_convergence.py tests/test_i110_policy_interface.py tests/test_i110_anticipation.py
python scripts/report_i110_convergence.py --output outputs/i110-convergence-20261102/analysis-NEW
```

Each analysis manifest records the source commit, dirty status/diff, executed source copies/hashes, exact command, Python/platform, declared configuration and input/output hashes. The input run manifests carry the original SUMO environment and seed provenance.

## Next task and claim boundary

Predeclare a bounded preview-control contract: observation range and uncertainty, conditional lateral-path handling, positive braking reserve, other-motion assumptions, recovery and native safety authority. Then test matched development interventions with full controlled/human/ramp/external burdens before any comfort acceptance; keep geometry qualification and v04 handoff separate.

The immediate design decision is whether 120 m is a hard observation limit. If so, the ramp-44 result motivates an upstream speed/comfort constraint or an explicit inability to guarantee this profile, rather than pretending the late warning solves the encounter. If 240 m exact state is permitted for a synthetic development comparator, label that information advantage and test its traffic cost. Choose the headway and other-motion assumptions before evaluation, not by selecting whichever model makes these cases pass.

This completes the roadmap’s offline convergence/feasibility diagnostic. It does not adopt a controller, close a comfort or scientific gate, prove human gap behavior, establish communication benefit, or admit v04 transfer. P01–P05/P07, factual geometry/traffic qualification and the delivered-v04 contract remain open. The shared Google Doc remains untouched under Sam’s write hold.
