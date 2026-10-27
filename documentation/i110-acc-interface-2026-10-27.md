# I-110 ACC comfort tuning and policy interface

27 October 2026 · Codex for Sam · engineering verification, no learned-policy result.

Sam asked for smoother acceleration/braking, an I-110 policy interface while
Hudson's weights are pending, and traceability from every result to its cause.
This follows the [checkpoint/network audit](i110-sumo-checkpoint-2026-10-27.md),
whose braking fixture reached 65 m/s³ peak controlled-fleet jerk.

## What changed and why

The original SUMO passing rule could brake a car alongside a slower vehicle on
its left, even without a leader in its own lane. A matched three-arm comparison
separates that effect from the new controller. `parallel` explicitly enables
`--lanechange.overtake-right true`; it is a stated simulation assumption, not a
claim about every real-world passing situation.

The new filter uses SUMO's ACC following-speed request, approaches the speed
target gradually, and limits requested acceleration changes relative to the
**measured** previous acceleration. Defaults are +1.5 m/s² acceleration,
2.5 m/s² comfortable braking, 2 m/s³ jerk, and 25 m/s desired speed. SUMO retains
final safety authority (`speedMode=31`, controlled `laneChangeMode=512`). An
emergency intervention can exceed comfort limits; actual response and request
mismatches are logged rather than hidden. Native ACC remains available unchanged.

| Braking fixture, seed 730201 | Peak fleet jerk | RMS jerk |
|---|---:|---:|
| Native ACC, default passing | 65.00 m/s³ | 11.482 m/s³ |
| Native ACC, parallel passing | 20.00 m/s³ | 1.305 m/s³ |
| Filtered ACC, parallel passing | 2.010 m/s³ | 0.348 m/s³ |

These arms have identical network, demand and seed. The passing change explains
part of the improvement; the filter explains the further reduction in this
fixture. It would be incorrect to attribute the entire reduction to tuning ACC.

## Verification and limits

The [frozen plan](../experiments/configs/i110_acc_comfort_v1.json) was declared
after development probes and before verification. Nine bounded runs comprise
the two native attribution arms, three smooth cases at two seeds, and one smooth
repeat. Every run exited naturally. All seven smooth runs passed, with zero
collisions, teleports or removed trips; fixed through-lane identities and complete
trip accounting held. All requested trips finished during the separately reported
drain, not by the 60 s primary horizon. The repeat trace matches exactly.

| Smooth case | Requested | Completed at 60 s, seeds 730201 / 730202 | Completed after drain | Peak jerk |
|---|---:|---:|---:|---:|
| Ordinary | 44 | 25 / 24 | 44 | 2.000 m/s³ |
| Braking leader | 45 | 25 / 24 | 45 | 2.010 m/s³ |
| Stopped leader and restart | 45 | 22 / 20 | 45 | 2.010 m/s³ |

The 2 m/s³ target has a disclosed 0.05 m/s³ realized-response allowance, frozen
after development measured a 0.00954 m/s³ excess. Each braking/stop run has one
small request/realization mismatch. This is an engineering target, not an external
comfort standard. The stop/restart case reaches a minimum measured bumper gap
of 9.08 m and closing TTC of 4.55 s. Ordinary free-flow runs do not measure
following response; their gap/TTC values are unavailable, not zero or infinite.

The traffic remains synthetic and uncalibrated. Two seeds do not establish
statistical generalization. Sudden cut-ins, stronger emergency braking, ramp
interaction, sensing error, dense congestion and physical vehicle dynamics remain
outside this acceptance. No PeMS connection is needed for these runs.

34 targeted tests passed (controller, graph, catalog, network and accounting).
Two existing pytest timeout-marker warnings remain; scenario subprocesses had
independent 120 s bounds. Python 3.11.16 and SUMO/TraCI 1.27.1 were used.

## Interface adaptation

`traffic/i110_policy_interface.py` introduces `i110_graph_v1`, a 30-feature
observation contract. Directed lane-graph paths track front/rear neighbors through
edge-position resets, internal junctions and route branches. Through-lane identity
stays stable when the auxiliary lane changes SUMO lane indices from 0 to 1 to 0.
Ambiguous adjacent context and unreachable remaining-route distance carry explicit
validity masks; proximity on another path is not mislabeled as a longitudinal gap.

Across the nine runs, 13,409 finite observation packets were recorded. Two
ambiguous adjacent contexts in the native arms were explicitly masked. The new
schema is tested plumbing; it does not grant lane-changing or merging capability.
It **rejects Hudson's legacy 27-feature actor contract**. Do not pad, truncate or
silently relabel inputs to make old weights load. The next step is to reproduce
the frozen actor in its original three-lane environment, then agree an explicit
transfer protocol and evaluation before considering training.

The [Hudson handoff request](hudson-checkpoint-request-2026-10-27.md) identifies
the missing artifact and optional supporting evidence. No weights were loaded,
no new policy was trained, and no policy-versus-ACC performance claim is made.

## Evidence, retained failures and rerun

The compact [result record](evidence/i110-acc-interface-20261027.json) is derived
from `outputs/i110-acc-interface-20261027/analysis-v2/summary.json`.
Raw traces, manifests, dirty-source snapshots, commands and metrics live in
`outputs/i110-acc-interface-20261027/verification-v1/`. Four figures and their
explicit input/generator lineage are in `analysis-v2/`; the
[evidence catalog](evidence-catalog/README.md) connects them to this question.

Development failures are retained: the first filter still reached 5.76 m/s³;
the second reached 2.00954 and failed the original strict 2.0 threshold. The
disclosed tolerance was fixed before verification, not retrofitted afterward.
`analysis-v1` failed by comparing raw Windows CRLF bytes with the trace's
canonical LF hash. The corrected report checks both canonical trace identity
and raw manifest byte identity; the failed directory is preserved.

From the repository root, with the configured traffic Python:

```text
python scripts/verify_i110_acc.py --network outputs/i110-sumo-20261027/network-c/network.net.xml --output outputs/i110-acc-verification-new
python scripts/report_i110_acc.py --verification outputs/i110-acc-verification-new --output outputs/i110-acc-analysis-new
python -m pytest tests/test_i110_policy_interface.py tests/test_i110_sumo.py tests/test_trip_accounting.py tests/test_sumo_backend.py tests/test_evidence_catalog.py -q
```

The v1 supervisor now rejects changed plan values that its fixed CLI defaults
would not honor. This guard was added after the recorded runs; their actual
resolved configurations already match the plan. It does not alter controller or
interface code recorded in those runs.

Primary references: [SUMO ACC](https://eclipse.dev/sumo/docs/Car-Following-Models/ACC.html),
[TraCI vehicle control](https://eclipse.dev/sumo/docs/TraCI/Change_Vehicle_State.html),
and the [SUMO right-passing setting explanation](https://www.eclipse.org/lists/sumo-user/msg06263.html).
