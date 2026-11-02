# Current-state anticipation and frozen-actor transfer review

29 October 2026 · Codex for Sam · retrospective development diagnostics.

The declared shadow checks are complete. A larger adjacent-vehicle headway
margin detects the scripted cut-in earlier, but also warns extensively without
the cut-in. Neither margin anticipates the two late ramp leaders before they
become connected leaders. **No comfort fix or I-110 actor transfer is adopted.**

Sam asked to continue from the existing checkpoint. This work follows the
[ramp/cut-in diagnosis](i110-ramp-cutin-2026-10-29.md), rather than repeating its
completed traffic tests. The protocol was saved before this new derivation in
`experiments/configs/i110_anticipation_shadow_v1.json`; the development outcomes
were already known. This is not a blind test, a new held-out comparison or a
controller intervention.

## Question, method and evidence

The question is whether current observable state contains useful warning before
the abrupt response, and what irrelevant warning exposure that entails. The six
fixed smooth-ACC development runs at seed 732001 are ramp departures 0/38/44/50 s
and the paired cut-in lateral-off/on cases. No old run, baseline command, safety
mode, demand or network was changed; no simulator or actor was launched.

The signal uses only current ego speed and graph front-neighbor gap/relative
speed. For headways 1.5 and 2.0 s, it computes

`margin = gap - [2.5 + headway * speed + max(0, closing_speed)^2 / (2 * 2.5)]`.

A negative margin warns. The two scopes are own lane and own plus adjacent
front neighbors. The closing term assumes 2.5 m/s² constant deceleration; this
screening formula is **not a safety guarantee or jerk-feasible response model**.
An adjacent vehicle is a hypothetical insertion, not inferred intent. Unknown
adjacency stays unknown; other-path Euclidean proximity is not a leader gap.
Clipped zero adjacent gaps can describe side-by-side vehicles and contribute
warnings even without an observed lane crossing.

The pure signal function has no event, future frame, scripted schedule or
lane-change-intent input. A separate scoring pass looks backward up to 3 s from
observed leader switches, peak realized jerk and every contiguous exceedance
episode. Target-specific lead excludes same-time and future warnings. Both
earliest and latest warning are retained so intermittent warning is not mistaken
for continuous lead. Aggregate warning seconds sum over controlled vehicles.

Every source run's input/output/copied-source hash was verified before analysis.
Jerk was recomputed from successive measured accelerations and matched the
recorded peak. Full primary/drain trip ledgers, role/origin groups, external
waiting, exposure, minimum gaps/TTC and event results are carried into the
derived summary. These are the unchanged baseline's burdens; there is no
measured burden for a hypothetical warning-triggered controller.

Raw evidence: `outputs/i110-anticipation-shadow-20261029/analysis-v2/`.
Its `results.md` is generated from all six runs and both headways; `summary.json`
contains all labels and full ledgers, and per-run warning journals contain the
current-only signals. The initial `analysis-v1` table displayed only 1.5 s,
although its journals/summary included both settings. V2 expands the table;
all six journals and the entire summary repeat byte-for-byte. V1 is retained.

## What the warnings establish

| Case | 1.5 s margin | 2.0 s margin | Interpretation |
|---|---|---|---|
| Cut-in | No target-specific prior warning in either scope | Adjacent scope starts 1.0 s before the 2.2 s jerk peak, 0.9 s before leader entry; own lane only 0.1 s before peak | Additional current adjacent state provides warning under the broader criterion; it does not identify intent |
| Ramp departure 44 s | First target warning 0.1 s before peak | Same | No warning before leader change at 52.5 s; peak follows at 52.6 s |
| Ramp departure 50 s | First target warning 0.1 s before peak | Same | No warning before leader change at 59.4 s; peak follows at 59.5 s |

The cut-in's realized peak remains 33.3704 m/s³. The late-ramp development peaks
remain 48.1704 and 47.1956 m/s³; these are not the earlier verification-seed
ranges. All exceed the unchanged 2.05 m/s³ bound. The 0 and 38 s ramp controls
and lateral-off cut-in remain at 2.0 m/s³.

For the **lateral-off** control, the 2 s own-plus-adjacent rule warns for 97.2
vehicle-seconds, 69.63% of controlled samples through H. Even the unshifted ramp
control warns for 132.0 vehicle-seconds, 96.77%. These are warning-exposure
figures, not labeled false-positive rates: counterfactual lane-change intent
is unknown. They strongly caution against treating every adjacent front car as
an immediate leader. The on/off pre-cut-in prefix check is retained separately
in `prefix-check.json`; matching pre-event inputs cannot reveal which future
scripted lateral command will be issued.

At the late-ramp peaks the new leaders are slow ramp vehicles entering graph
reachability while agent_0 remains upstream on sb1. Increasing the same
headway threshold cannot create an earlier observation of a missing candidate.
The current nearest-adjacent-front contract does not establish a prediction of
that cross-road convergence. This supports investigating convergence-aware
current state next; it does not uniquely identify SUMO's internal braking branch.

## Explicit frozen-actor semantic review

Original sources were rechecked against the delivered evaluation manifest, with
source-function line anchors and hashes in `transfer-v1/review.json`. The actor
was not loaded. The original fixture has a single 5 km, three-lane road. The
current I-110 mainline has four/five lanes and ramp/exit links. Across these six
development runs, 8,361 packets comprise 2,333 four-lane and 6,028 five-lane
packets, including three unknown-adjacency packets. None is a direct original
three-lane packet. This sampled fact alone is not a proof that every possible
restricted adapter is impossible.

| Contract | Verified source meaning | Required disposition |
|---|---|---|
| Lane coordinates and bounds | Original uses lane index / 2 and hardcoded bounds 0–2; graph uses current count and explicit adjacency | Do not clip lane 3/4, hide real lanes or invent a three-lane boundary |
| Longitudinal coordinates | Original uses one-road front-bumper position and `1 - position/5000`; graph uses connected paths and `remaining/(traveled+remaining)` | Shortening the route denominator or resetting edge position changes semantics; declare coordinates explicitly |
| Neighbor reachability | Original scans same lane index on its only road; front uses delta >= 0, rear delta < 0; graph follows directed route-compatible paths | Junctions and ramps need explicit support limits; equal-position tie behavior also differs |
| Allowed speed | Both implementations ultimately read TraCI allowed speed; the original names it speed_limit | The older comparison's names do not imply different APIs. Vehicle parameters and values still differ and must be matched |
| Prior action | Original records bounded, pre-shield **actor-requested** acceleration / 4.5, reset to zero; graph logging clips raw ACC request / 4.5 | ACC history cannot be reused as an actor's prior action; retain distinct requested/applied/realized values |
| Time and lifecycle | Original step count starts at zero after all four agents spawn, then /600; graph uses absolute simulation time / supplied horizon (90 s here) | Time origin, delayed spawn, termination and reset need an explicit adapter contract |
| Unknown state | Original has absence zeros without graph uncertainty masks; graph carries adjacency/remaining-route-known fields | Never drop unknown masks and call the input supported; reject unsupported context |
| Action and shield | Original normalized scalar maps affinely to −4.5…+2 m/s² with own-lane headway/speed/zero-speed guards; smooth ACC uses different comfort shaping | No silent shield substitution. Define common safety authority and log both external and realized intervention |

Decision within this review: **no semantically exact supported I-110 restriction
has been established for the unchanged original ABI/configuration.** Keep the
existing schema rejection. A virtual local three-lane window around an interior
ego could be a separately named proposal, but it would need truthful lane bounds,
neighbor completeness, route/time coordinates, action history and fail-closed
support checks. The name or 27-element shape would not establish transfer
validity. Trimming the corridor to three lanes would be a new fixture, not a
full I-110 result. No such execution is admitted by this review.

The independent original numerical gate remains unchanged: acceptance passed,
strict all-field comparison failed with 34 values outside frozen 1e-9 tolerance.
No runtime replacement, tolerance change or held-out tuning occurred.

## Next concrete work and boundaries

1. Predeclare a current-state convergence diagnostic for vehicles on incoming
   paths that join the ego corridor, preserving path identity, signed distance,
   reachable conflict point, current speed and explicit uncertainty. Score it
   on these development cases without event IDs or future route-change truth.
2. Define a jerk-feasible braking envelope and test whether any current-state
   warning leaves enough time to act. Only then run a separately named matched
   controller diagnostic with unchanged smooth baseline, safety authority and
   complete traffic/ramp/external burdens. Do not turn the shadow warning into
   a claimed comfort fix or tune on held-out seeds.
3. Keep numerical parity and actor transfer as separate gates; no new training
   or I-110 actor inference. Continue the ordered event-adapter, geometry,
   optional offline PeMS and layered USD tracks after the appropriate traffic
   checks. Independent human gap-opportunity accounting remains open.

Validation: 18 targeted tests pass, including new margin units/direction,
unknown-context handling, event-field independence, temporal/identity scoring
and changed-source rejection, plus existing graph-interface tests. No completed
traffic or actor suite was restarted. The live master and Sam refresher were
read; latest shared modification remains October 22. P01–P05/P07 and the shared
write hold remain. No empirical safety, physical driving, communication benefit,
team assignment or team adoption is inferred.

Reproduce from the active checkout with the research root's traffic Python:

```text
python scripts/report_i110_anticipation.py --output outputs/anticipation-new
python scripts/audit_i110_transfer_support.py --analysis outputs/anticipation-new --output outputs/transfer-audit-new
```

All output directories must be new. Raw packages stay local and ignored; the
source/docs commit does not include them. The compact versioned evidence record
is `documentation/evidence/i110-anticipation-shadow-20261029.json`.
