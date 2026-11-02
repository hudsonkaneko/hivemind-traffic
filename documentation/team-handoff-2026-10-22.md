# Team handoff and next-session entry point

22 October 2026 · Prepared by Codex for Sam at his end-of-day request.

The [shared research master](https://docs.google.com/document/d/1Vme-G1Zefd092cC3IFJD6ek5BR9F_ynqHZeNvhtg8-c/edit)
now contains a current checkpoint and an attributed entry in Sam's refresher.
Saved-to-Drive state and preservation of all prior text (ignoring whitespace)
were verified; original master/Sam headings remain. This specific closeout was
authorized by Sam. Historical editions below the checkpoint remain history.
Local before/after text, screenshot and checks are retained under
`outputs/session-close-20261022-a/`. No separate teammate message was sent.

## Start here next session

**Qualify the exiD Aachen-Brand source package before rebuilding a road.** The
first deliverable is a small site-access and evidence sheet: access status,
location/recording IDs, carriageway direction, acquisition dates, map/3D edition,
rights, observed boundaries and independent recording blocks. Do not begin by
reinstalling tools, rebuilding I-80 scenery or launching policy training.

1. Read the live master, this handoff and the [site decision](highway-twin-site-decision-2026-10-22.md).
   Check whether teammates have supplied new code, access or accepted results.
2. Check whether Sam has applied for or received exiD access. The
   [application draft](exid-access-request-draft.md) is prepared but unsent;
   university identity and the publisher's license acceptance remain required.
   Do not assume one grant permits sharing source files across the team.
3. If access is granted, inventory the package and exact Aachen-Brand metadata.
   Define calibration/development/untouched-audit partitions before analytical
   inspection. Start with one development recording and its matching map/asset.
4. Check the merge taper, lane connections, physical gap opportunities, observed
   entries/exits and queue visibility. Identify unsupported upstream demand and
   censored trips explicitly. Freeze the road epoch and proposed boundaries.
5. If these checks support the case, make one small checked road/replay segment
   in the existing viewer. Validate coordinates, lane alignment and road contact
   before adding the full surrounding mesh. Full factual admission has additional
   model/partition/regime requirements; one visual segment does not establish it.

If access is pending, finish the access/rights questions and a source manifest
template from public metadata, review PR 16 and define the independent geometry
checks. Keep the exact site fields unresolved. Do not fetch all regional mesh
archives, substitute guessed lanes or treat synthetic demand as observed.

## What teammates can rely on

| Component | Evidence-backed status | Practical limit |
|---|---|---|
| Existing traffic code | Scripted SUMO baseline reproduced; deterministic same-seed replay checked | Not an empirical merge or learned communication result |
| Whole-trip accounting | Fixed horizon plus separate drain; requested/external waiting/active/completed/removed/unfinished accounting and group splits | Real-site requested demand and unseen queues still need evidence |
| Viewer | Supplied-car layered USD replay, telemetry, play/pause/scrub and free/follow/overview cameras; bounded RTX checks | Replay is not a vehicle-dynamics or LiDAR validation |
| I-80/Powell | Original source alignment and historical inspection established | GIS lines over a flat slab; not a realistic reconstructed road; full study not admitted |
| Preferred next site | Codex recommends recording-era exiD A44 Aachen-Brand under Sam's delegated selection | Restricted package not acquired; exact direction/date and factual admission pending; not recorded team-wide adoption |
| Esri geography | NRW I3S service metadata retrieved; public terrain/imagery sources identified | No Aachen mesh imported; conversion, date matching and surface accuracy untested |
| Hudson's learned driver | Forwarded experimental RSL-RL PPO report and source interface inspected | 42 state observations, engineered braking target, two actions at 60 Hz; no accepted full-scene policy or following/merge validation |
| Communication study | Approved research design and finite protocol register available | Matched CTDE learner, admitted human models, frozen budget and final evaluations remain work to do |

The low-speed learned-driver development checks and separate scripted driving
success are different evidence. Do not describe the current learned policy as
LiDAR-based, highway-speed validated or merge-ready. No policy weights were
acquired by Sam's qualification work.

## Research fit and collaboration

The study remains one open merge with uncertainty in human gap acceptance,
three admitted settings of one fixed model, matched local/communicating shared
PPO with equal centralized-critic opportunity, strong scripted baselines, one
fleet share and count-matched dispersed/clustered arrivals. Count all requested
trips and waiting, including humans, ramp vehicles and unfinished/failed trips.
Preserve independent factual audit and independent training/traffic uncertainty.

Hudson's base-driver work can progress alongside geography and accounting. Once
qualified, freeze the same driver/sensing/safety stack across communication arms.
The immediate handoff needed from his work is the accepted checkpoint/config,
observations/actions/units/rates, reset/command interface, supported operating
envelope and held-out full-scene evidence, followed by following/merge evidence.
These are integration needs, not newly assigned work or a claim that they exist.

The last recorded highway-generation focus for Lorenzo is historical; no new
completion or availability is inferred. Geographic contributors should share
source IDs, dates, rights, CRS/vertical datum, lane graph and independent alignment
checks. Sam's current contribution connects site qualification, viewer/accounting
and research evaluation. Confirm leads/reviewers together rather than infer them.

SUMO remains the traffic owner in Sam's scoped study; Isaac supplies the bounded
physical response check and a validated surrogate for traffic reruns. The broader
physical-fleet roadmap is separate. Perception training and a full physical fleet
are not silently added to the study. No unattended training is authorized.

## Repository, evidence and review

Review [PR 16](https://github.com/hudsonkaneko/hivemind-traffic/pull/16), branch
`codex/sam-visual-twin` on `samfrausto/hivemind-traffic`. It remains a feature PR;
do not assume these changes are merged into upstream `main`. The site-plan
checkpoint is `ca1b2c8`; camera change `81e2fb7`; I-80 qualification `aa542b1`;
accounting `957a064`. The closeout documentation follows those commits.

Public code/docs are in the PR. Private supplied cars, checkpoints, generated
scenes, raw datasets, installed runtimes and local evidence are not included in a
clone. Review source online or check out the PR in an appropriate clean checkout;
do not overwrite teammate work to match Sam's environment. Traffic, Isaac and GIS
Python environments stay separate.

Evidence routes:

- [Viewer instructions](visual-twin.md) and [initial render evidence](visual-twin-2026-10-22.md).
- [Complete accounting](merge-accounting-2026-10-22.md) and [driver contract](hudson-driver-research-contract.md).
- [I-80 qualification](i80-powell-qualification-2026-10-22.md).
- [Site decision, source stack and full build gates](highway-twin-site-decision-2026-10-22.md).
- Local evidence: `outputs/merge-accounting-20261022-a/`,
  `outputs/i80-qualification-20261022-b/`, `outputs/i80-free-camera-20261022-a/`
  and `outputs/i80-geospatial-rebuild-20261022-a/`.

The I-80 milestone's full repository check passed 1,797 tests with one private
asset skip and 36 subtests, after a retained timeout/retry. It is historical
validation of that code checkpoint, not a fresh test of today's documentation.
Camera startup checks subsequently passed. No new simulator or training run is
needed for this documentation closeout.

## Resume prompt

> Read PROJECT_REFERENCE.md, the latest shared master and Sam refresher, and
> documentation/team-handoff-2026-10-22.md in the existing repository. Resume with
> exiD access and Aachen-Brand qualification. Check for new teammate evidence,
> confirm exact recording/map/3D dates and rights, protect an untouched factual
> audit partition, and produce the site/boundary/queue evidence sheet. If access
> is pending, complete the public-source manifest and geometry acceptance plan.
> Reuse the existing viewer and complete accounting. Do not start training or
> claim a validated digital twin before the relevant gates pass.
