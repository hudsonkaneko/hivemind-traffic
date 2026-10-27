# Merge qualification and complete trip accounting

22 October 2026. Codex for Sam. Accounting is verified on engineering fixtures;
I-80/Powell remains a conditional historical candidate, not an admitted empirical
case. Hudson's controller interface is recorded separately in
[the integration contract](hudson-driver-research-contract.md).

## What changed

The research runner `traffic.accounted_baseline` runs to a declared horizon H
even when the existing two controlled agents finish earlier. It then drains the
same fixed cohort for a separately declared budget. Primary-window results stay
frozen; the drain cannot replace them with a more favorable completion-only result.
The training environment's episode semantics are unchanged. Its existing
two-agent scripted controller is reused only on the original curved fixture.
The new open-merge fixture uses native SUMO driving for accounting verification.

Each explicit request is retained before insertion and classified as scheduled,
waiting outside, active, teleporting, completed or removed. Reports split by
controlled/human-proxy role and mainline/ramp origin. Unknown origin is kept
explicit rather than inferred. The primary time sum runs from requested departure
to successful arrival or H. Removed and teleported trips cannot reduce this sum
by disappearing: they remain unserved through H and invalidate the lifecycle
check. `comparison_valid` in raw reports means only no observed lifecycle failures
or collisions; it does not establish scientific validity or a qualified site.

Departures use native SUMO departure times. Arrivals use the event observation
boundary, up to one step (0.2 s here) after native tripinfo's arrival timestamp.
This conservative discretization is reported and crosschecked against native
tripinfo. Initial-cohort accounting is unit-tested from observation time zero;
the runnable adapter currently requires time-zero explicit vehicle demand.
It rejects unexpanded flows, triggered departures, extra demand files and demand
scheduled during the drain. General demand loaders and dynamic learned-agent
entry remain separate work.

The backend now reconciles actual active IDs each step, preventing removed cars
from leaving fabricated zero-valued subscription states. The viewer consumes the
recorded ledger and labels primary versus drain time; its overview and ground
size follow the network bounds.

## Reproduce

Use the existing traffic Python/SUMO environment, from the checkout. Each output
must be new. These commands run no training:

```text
python -m traffic.accounted_baseline --horizon 80 --drain 60 --seed 2001 --output outputs/accounted-curved-new
python -m traffic.accounted_baseline --config scenarios/accounting_merge/scenario.sumocfg --controller native --horizon 40 --drain 90 --seed 2001 --output outputs/accounted-merge-new
python -m pytest tests/test_trip_accounting.py tests/test_accounted_baseline.py tests/test_sumo_backend.py tests/test_highway_parallel_env.py tests/test_visual_twin.py -q
```

Each run stores source copies, the dirty diff, input/output hashes, resolved
requests, runtime versions, native `tripinfo.xml`, replay and primary/drain
reports. Use `build_scene` from `scripts/build_visual_twin.py` with an existing
authorized vehicle wrapper to compose the new recording; no source asset must
be rewritten. See the [viewer guide](visual-twin.md).

## Observed engineering results

Local evidence root: `outputs/merge-accounting-20261022-a/`. It is ignored and
is not supplied by a clone. Runs started from `e4f4dce` with new code uncommitted;
each run's source snapshot and diff preserve its actual implementation.

| Check | Result |
|---|---|
| Old curved replay | Original 270-frame pose prefix unchanged; both controlled cars still leave at 54.0 s |
| Whole curved cohort | Three cars remain at 54.0 s; all five complete by 55.8 s; fixed H remains 80 s |
| Repeat seed 2001 | All frames and ledger counts repeat exactly |
| Seed 2002 | Same aggregate completion result; extended trajectory differs, so seed variation is not discarded |
| Synthetic merge at H=40 s | 12 requested, 2 completed, 10 active/unfinished; 24.8 vehicle-seconds of external insertion waiting |
| Separate merge drain | All 12 complete within the 90 s drain budget; zero reported collisions/teleports/removals |
| Native crosscheck | Every request represented, including a tested stop with undeparted vehicles; departure/arrival bounds pass |
| Hand calculations | External queues, future demand, initial cohort, disappearance and teleport contamination checked |
| Viewer composition | 5,088 vehicle/frame checks, zero position error, 72 bound mesh instances, no active unresolved assets |
| RTX check | 360 updates, two inspected PNGs, seven control/state checks true, saved layers unchanged, process exit 0 |
| Full repository suite | 1,794 passed, 1 private-asset skip, 36 subtests passed; 213.62 s under a 300 s supervisor |

The existing traffic environment still lacks the pytest-timeout plugin, producing
two unknown-mark warnings. The external supervisor supplied the process bound;
no dependency installation or global environment change was made.

The first focused test run exposed only an exact-equality assertion on a floating
sum (`1.9999999999999998` versus `2`); the hand-calculated expectation now uses a
numeric tolerance. The failed log is retained. Native render evidence and callback
checks do not establish manual GUI interaction or rendering throughput.

## Real-merge qualification outcome

The [FHWA I-80 description](https://www.fhwa.dot.gov/publications/research/operations/06137/index.cfm)
identifies the Powell Street on-ramp within the historical Emeryville study.
This remains the one historical candidate under inspection. No second historical
campaign or contemporary-road claim was added.

A bounded public API query acquired 93,618 rows for 212 vehicles in the earliest
120-second window. The last 17:15–17:30 published period was reserved before
analysis and its rows were not retrieved. The sample hash is
`b12a04b421d0f0eef014716ec5bd315377998b93f14c3fef643c03abe08fc1b4`.
The reproducible audit is `scripts/audit_ngsim_sample.py`; local query URLs,
protocol, metadata and `i80-development-audit.json` identify the acquisition.

| Evidence | Finding and consequence |
|---|---|
| Sample integrity | No duplicate vehicle/frame/time keys, nonfinite required values, negative speeds or nonpositive dimensions; all within-track time increments are 100 ms |
| Sample coverage | 119.9 s between first/last samples; 143 tracks are partial within this extraction; these are not automatically source defects |
| Merge candidates | 12 lane 7-to-6 transitions and 2 reverse transitions; these require event/video review and are not an accepted-gap dataset |
| Site schema | The API's generic lane description describes US-101. The I-80-specific original PDF confirms lanes 1–6 mainline, 7 Powell ramp and 9 shoulder |
| Time alignment | First timestamp converts to 15:58:55.3 PDT, earlier than the rounded published 16:00 start; exact file/time alignment remains open |
| Censoring | Original metadata states that boundary-incomplete trajectories were omitted. Camera entries cannot supply latent requested demand or unobserved external queues |
| Original files | PDF recommends original TXT over converted CSV. Official archive byte-range inspection could not parse its ZIP directory; no original TXT crosscheck is claimed |
| Independent evidence | One trajectory day and adjacent late periods do not establish independent-day validation. A/B/C membership, interaction buffers and usable support remain unqualified |
| Release terms | Public API catalog says CC BY-SA 3.0; legacy PDF has different RDE redistribution wording. Raw files remain local pending reconciliation |

Primary metadata and original PDF were read from the
[DOT dataset](https://data.transportation.gov/Automobiles/Next-Generation-Simulation-NGSIM-Vehicle-Trajector/8ect-6jqj)
and its `I-80_Metadata_Documentation.pdf` attachment. The site-specific lane/unit
page was also rendered and visually checked. Original text distinguishes reported
accuracy estimates from the absence of a dataset-wide accuracy assessment.

The Esri-hosted item
[`4729885c4d4949469681be25b4c314ee`](https://www.arcgis.com/home/item.html?id=4729885c4d4949469681be25b4c314ee),
`CTOD_HWY_SHN_Lines`, was inspected with a small bounding-box query. Its owner is
`mark.hedlund_Gov_OPR`, attribution is Caltrans, service layer is 24, and storage
CRS is Web Mercator (3857). Item publication/update metadata is from 2026; source
geometry epoch is not thereby established. It provides highway centerlines for
context, not the 2005 lane/ramp surface. Historical State Plane III coordinates,
US survey feet, two independent anchors and lane geometry still need explicit
alignment. Nothing here establishes a calibrated Esri twin.

**Disposition:** retain I-80/Powell for conditional historical qualification;
do not admit it for factual fitting or final communication comparisons yet.
Next closure package: readable original trajectory/GIS sources, upstream/ramp
detector and queue support, independently reviewed merge opportunities including
rejections, and a viable untouched-audit partition. A synthetic fallback would
require an explicit narrower claim, not silent relabeling of this fixture.

## Research and collaboration boundary

Sam authorized this implementation and public-data qualification. Hudson's new
driving-controller information is user-supplied and source-corroborated, with
reported performance kept distinct from reproduced results. No policy weights,
training, partner branch merge or shared Google Doc edit occurred. The full
traffic study remains SUMO-owned with a bounded physics-response check unless
the team explicitly adopts and qualifies a different simulator architecture.
