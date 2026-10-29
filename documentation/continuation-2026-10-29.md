# Ramp, cut-in and delivered actor: current handoff

29 October 2026 · Codex for Sam · local checkpoint, not published.

Sam authorized targeted ramp interaction tests, abrupt cut-in investigation and
continued evidence/handoff upkeep. He then supplied Hudson's exact checkpoint,
reference evaluation and checksum URLs. This authorizes the previously ordered
hash-gated original-fixture reproduction; it does not authorize training or an
unreviewed I-110 actor transfer. This handoff supersedes the older missing-actor
next action while preserving all historical reports and failures.

Use the existing `research/digital-twins/repository-review/source` checkout on
`codex/sam-visual-twin`. The starting checkpoint was `cc2a797`; preserve the
unpublished history, local outputs and separate traffic/Isaac environments.
Read root PROJECT_REFERENCE.md, applicable AGENTS.md, the research-custodian and
organize-av-evidence skills before continuing. Refresh the live team master/Sam
refresher; its latest observed modification this session was October 22.
Sam's newer I-110 directions remain controlling. No new team assignment or
team-wide adoption is inferred.

## Verified this session

- [Ramp/cut-in report](i110-ramp-cutin-2026-10-29.md): nine development and
  22 verification runs; all delayed ramp cases exposed, exact repeats, complete
  post-drain cohorts and no collisions/removals/teleports. Later smooth ramp
  encounters reach 47.287–47.839 m/s³, failing the unchanged comfort bound.
  A diagnostic command cap removes the cut-in mismatch but leaves motion
  identical and peak jerk at 33.3704. It is not an adopted fix. Native sampled
  blocked-desire states are not independent gap-opportunity accounting.
- [Hudson reproduction report](hudson-reproduction-2026-10-29.md): bytes and
  14 original sources/config verified; original smoke/repeat and fixed 20 seeds
  pass original acceptance. Strict per-field comparison fails: 153 floating
  differences, 34 beyond the frozen tolerance, with all non-floating fields equal.
  Runtime differences are recorded; their causal role is unproven. No I-110
  actor run, training, communication or physical-driving claim follows.
- 71 targeted tests pass in `outputs/i110-ramp-cutin-20261029/tests-final-v1.xml`.
  The two existing unknown pytest timeout-marker warnings remain; simulation
  supervisors enforce independent process bounds. TorchScript emits a
  deprecation warning; the hash-verified original actor format is preserved.
  Both analysis figures were visually checked. Raw packages are local/ignored,
  not recoverable from the source/docs commit alone.

## Next actions in order

1. Predeclare current-state anticipation/headway-margin diagnostics for the
   cut-in and later ramp encounters. Use development cases first, retain the
   unchanged smooth baseline and safety authority, and measure realized jerk,
   exposure, external waiting and every cohort's burden. Do not provide future
   event truth to the controller. Do not adopt the diagnostic follow cap as a fix.
2. Review the explicit original-versus-I-110 semantics: original lane coordinates,
   remaining distance, previous action, neighbor reachability, unknown context,
   action caps and safety guards. The 27-feature actor cannot be padded/truncated
   into the 30-feature graph contract. Decide whether a separately named,
   supported restricted diagnostic exists before running any transfer.
3. Retain the numerical reproduction failure as its own gate. If deeper parity
   work is needed, specify an isolated matched-runtime experiment and use any
   subsequently supplied original action journal for first-divergence analysis.
   No need to re-download or search for the now-present policy.
4. Continue the remaining [event/geometry/calibration/USD tracks](i110-transfer-and-next-tracks-2026-10-28.md):
   independent stopped/closure/police-break SUMO adapters; traceable geometry
   uncertainty; optional offline PeMS; then aligned layered USD replay after
   traffic checks. Keep native gap-opportunity limitations and P01–P05/P07 open.
5. Add explicit lineage for every new result/figure, retain failed editions,
   rebuild the catalog and prepare local continuity updates each session.

## Boundaries and close-out procedure

No training, public push, teammate message or shared Google Doc write was
performed. The October 16 shared-write hold remains; an attributed document-ready
update is prepared locally. Physical driving, calibrated ramp behavior, site
admission, communication benefit and team-wide adoption remain unverified.

The final regression command was:

```text
python -m pytest tests/test_i110_events.py tests/test_i110_sumo.py tests/test_i110_policy_interface.py tests/test_trip_accounting.py tests/test_sumo_backend.py tests/test_evidence_catalog.py tests/test_i110_interactions.py tests/test_i110_interaction_report.py tests/test_hudson_actor_evaluation.py tests/test_hudson_evaluation_comparison.py -q --junitxml=outputs/i110-ramp-cutin-20261029/tests-final-v1.xml
```

`outputs/i110-ramp-cutin-20261029/close-records.py` derives the versioned compact
evidence records and registry links from retained source manifests/results.
`documentation/evidence-catalog/activities.json` is the curated registry.
The latest generated catalog is `outputs/evidence-catalog/20261029-final-v2/`;
consult its coverage report for unresolved historical backfill rather than
assuming all inventoried material has proven lineage. Root continuity records
name the resulting local commit and index-refresh evidence.

The first catalog validation rejected empty acquisition-input lists; exact
supplied URLs were added. A subsequent attempt correctly refused to overwrite
that existing empty edition. Both attempts are retained; the fresh v2 edition
contains the corrected catalog. The baseline's stale missing-checkpoint metadata
was corrected after the runs, with no controller or acceptance change.
