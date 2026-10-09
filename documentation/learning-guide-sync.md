# Engineering guide synchronization

## Pending: loop traffic showcase

The [showcase record](loop-traffic-showcase.md) and
[version folders](../showcases/loop_traffic/README.md) contain the current plan,
commands, explanations, physical tests and failure fixes. This milestone uses a
scripted controller, privileged object positions, one dynamic prepared car and
kinematic background cars; it does not introduce RL or LiDAR perception.
Isaac Lab work is postponed for the requested presentation. Google tabs have
not been edited for this milestone. A fresh required trusted-reader attempt on
October 9, 2026 failed with `workspaceRoot must be an absolute path`, despite the
canonical absolute Windows path. No cloud write was attempted. The Google Docs
skill requires that structural inspection before editing the existing guide.
Repository/GitHub documentation is updated
independently, and a cloud-sync success is not claimed.

## Pending: mentor-guided continuous highway loop

The local [loop implementation and evidence note](continuous-highway-loop.md)
and [branch roadmap](mentor-loop-roadmap.md) are the current source for this new
milestone. Existing Google tabs were not modified. A fresh attempt to use the
required file-backed reader failed with `workspaceRoot must be an absolute path`
for the canonical Windows directory. Native write/format/readback verification
therefore remains pending; the verified synchronization below is historical.

Last verified: October 6, 2026.

Latest source milestone: commit `e74b574`, published on `codex/realtime-preview`.
[Pull request #5](https://github.com/hudsonkaneko/hivemind-traffic/pull/5) records
the implementation, tests, evidence and open GUI timing gate. It is stacked on
`codex/demo-readiness`; `main` is unchanged.

## Latest update: live preview timing

Added **33 Live preview timing**, tab `t.e9ctk5gfkhg`, to the
[same Google guide](https://docs.google.com/document/d/1qeYucQnP4nznmLNJrlUz1hCIcY7KOigRPuhBVH-Zo0c/edit?tab=t.e9ctk5gfkhg).
It includes portable launch commands, demo controls, rendering/physics/sensor
boundaries, real-time-factor and lag definitions, actual passing/failing
measurements and the next profiling task. It explicitly states that consistent
real-time GUI playback is not yet verified; a headless pass is not a GUI claim.

Native readback verified exact text, all **34 tabs**, eight title/heading
paragraphs, six shaded Consolas command blocks, five highlighted terms/status
spans and one immutable source hyperlink. Link typography was checked using
local style plus inherited NORMAL_TEXT: Arial 11, black, not underlined.
All 33 earlier tab bodies, titles, order and parents remain unchanged.
The simulator GUI capture was inspected separately; Google Docs pagination
was not visually inspected. No existing tab was rewritten to hide older results.

Trusted read: ignored `outputs/doc-sync/trusted-read-realtime-01`, with verified
byte/hash persistence through the supported Windows fileIO adapter. No protected
controls were detected in the preceding milestone scope. Verification:
`outputs/doc-sync/realtime-preview-verification.json`. Final revision:
`ANLCKQn0vDAW1c7kCqA3j88pZRQfW_qjhd9k3iuEzla63iNIi_kIy95J_jCfmn-qvhNHHzvAdbDpPzDCTmmfM6eoLByAn39zoqm2vboWoBc`.
Raw evidence remains local and ignored by Git. No new training was started.

## Previous synchronization: demo readiness and profiling

Added **32 Demo readiness and profiling**, tab `t.9wkfs52ky6df`, to the
[same Google guide](https://docs.google.com/document/d/1qeYucQnP4nznmLNJrlUz1hCIcY7KOigRPuhBVH-Zo0c/edit?tab=t.9wkfs52ky6df).
It separates SUMO/frozen PPO, full-size scripted PhysX/RTX bypass, and RC-scale
Leatherback PPO inference; explains the measured logging/safety speedup; records
matched seeds, failure tests, startup and GUI timing limits; and preserves the
user-participation gate before new training.

Native readback verified exact text, 33 tabs, nine title/heading paragraphs,
eight shaded Consolas command blocks, six highlighted definitions and three
source hyperlinks with matching effective typography. All 32 prior tab bodies,
titles, order and parents remain unchanged. Representative simulator captures
were inspected; Google Docs pagination was not visually inspected.

The required read initially failed because the checked bridge's POSIX/TTY file
transport did not work on Windows. No document write occurred during those
failures. Its supported local fileIO adapter subsequently persisted byte/hash-
verified artifacts without changing the trusted detector or external skill.
Trusted read: ignored `outputs/doc-sync/trusted-read-10`; no protected controls.
Verification: `outputs/doc-sync/demo-readiness-verification.json`.
Verified revision:
`ANLCKQkGgQ3HvawyKnhFxNMtsxFtVz9Bls41TgOVCmztFLBL1S-Hd408f-qrBjCqkvh7wMrJ2h-oVtj-UyGR_XWK4aTOzy9bjZwOQLlfr3o`.
The source hyperlinks use immutable commit `703961b`. Raw simulation and
document-read artifacts remain local and ignored by Git.

## Previous synchronization: adaptive obstacle bypass

Added **31 Adaptive obstacle bypass**, tab `t.cbgq2dwnrach`, to the
[same Google guide](https://docs.google.com/document/d/1qeYucQnP4nznmLNJrlUz1hCIcY7KOigRPuhBVH-Zo0c/edit?tab=t.cbgq2dwnrach).
It explains LiDAR-triggered route selection, cyan driver-path preview, driver
versus physics roles, privileged odometry/known-map/extent-prior assumptions,
fault cases, actual metrics, retained failures, portability and training limits.

Native readback verified exact text, 32 tabs, ten title/heading paragraphs,
five shaded Consolas command blocks, six highlighted definitions and two full
source hyperlinks with matching effective typography. All 31 prior tab bodies,
titles, order and parents remain unchanged. Simulator captures were inspected;
Google Docs pagination was not visually inspected.

Trusted read: ignored `outputs/doc-sync/trusted-read-08`; no protected controls.
Verified revision:
`ANLCKQlHXiCPFPbOTDaTUveFDi2GR6nwQFk13xZewjJD0mYw5yUSqDq2aqGuWg_OvYswRaJ9Zhf3sg40h5GVvP7XY5nxWAm2kEsbHTQFUsw`.
The final appended evidence sentence records the passing unchanged-code GUI
retry, its measured RTF near 0.36, and the old-demo regression. Text and prior
tab preservation were verified again after that insertion.
Implementation source links use immutable `07a373b`; the PR carries later
validation updates. Bulk simulation evidence remains local and ignored by Git.

## Previous synchronization: wheel orientation correction (252c6de)

Added **30 Wheel orientation fix**, tab `t.ivzd9eyrxgjj`, to the
[same Google guide](https://docs.google.com/document/d/1qeYucQnP4nznmLNJrlUz1hCIcY7KOigRPuhBVH-Zo0c/edit?tab=t.ivzd9eyrxgjj).
It explains the cylinder/attachment basis mismatch, explicit axle correction,
unchanged physical/control traces, corrected coordinate-frame test, retained
failed attempt, measured errors, scope limits and restart instructions.

Native readback verified 31 tabs, exact inserted text, eight title/heading
paragraphs, three shaded Consolas commands, four highlighted definitions and
three source hyperlinks with preserved effective typography. All 30 earlier
tab bodies, titles, order and parents remained unchanged. Google Docs pagination
was not visually inspected; simulator captures were inspected separately.

Trusted read: ignored `outputs/doc-sync/trusted-read-07`; no protected controls.
Final verified revision:
`AHj4eMTpDBPYwDNssfWSW2hfDlFaOA1ObNBPOyhMqQeJi900Hj5KWmLHKxUHIVAFoLDjMCEmLcwAY4SWpa85ySBe7nsGS561JqFSMRNZJP8`.
Source links point to immutable `252c6de`; raw simulation evidence remains local.

## Previous synchronization: visible physical car and LiDAR view (531f748)

Added **29 Physics car and LiDAR view**, tab `t.rb5nks6ogqck`, to the
[same Google guide](https://docs.google.com/document/d/1qeYucQnP4nznmLNJrlUz1hCIcY7KOigRPuhBVH-Zo0c/edit?tab=t.rb5nks6ogqck).
It explains the visible road/cameras/path/points, control-versus-physics boundary,
privileged map/odometry versus sensor inputs, multi-rate clock, raw-scan safety,
retained timing/serialization/writer/display failures, independently audited
braking results, performance limits and user participation before training.

Refreshed the three relevant **00 Start here** paragraphs and preserved their
existing styles. Native verification confirmed 30 tabs, unchanged topology and
body contents for all 28 earlier milestone tabs, exact inserted text, 11
title/heading paragraphs, four shaded Consolas commands, 11 highlighted spans,
and five full-label hyperlinks with preserved effective typography. Inherited
Arial/11-point/black/un-underlined styles were checked against NORMAL_TEXT.
Native content/styles were verified; rendered Google Docs pagination was not
visually inspected. Simulator viewport captures were inspected separately.

Trusted read: local ignored `outputs/doc-sync/trusted-read-06`; no protected
controls detected. Final revision:
`ANLCKQlj7SWapZ_gsiS8MfphCwTClixXLTc-WI36rf7LE1BCqINHIRSR1k_XX0_sa-EsO0T2go0dGASt7IRLzElJS7DecKMT_wBKkIVD7Pc`.
Repository evidence remains authoritative; raw outputs are local and not part
of a clone. No training, dependency installation or external-runtime edit occurred.

## Previous synchronization: scripted lane following (e4404d3)

Added **28 Scripted lane following**, tab `t.70inmvd9orgv`, to the
[same Google guide](https://docs.google.com/document/d/1qeYucQnP4nznmLNJrlUz1hCIcY7KOigRPuhBVH-Zo0c/edit?tab=t.70inmvd9orgv).
It explains geometry, behavioral/planning/control separation, privileged-state
versus sensor inputs, rates/expiry, the contact-report positive control, measured
lane/stop/repeatability outcomes and the retained failed lifecycle gate. It also
records the user's direct participation before future Isaac Lab policy training.

Refreshed **00 Start here** to point to tabs 25–28. Native connector verification
confirmed 29 tabs, unchanged topology/body contents for all 27 earlier milestone
tabs (01–27), ten title/heading paragraphs, three shaded Consolas portable commands,
eleven highlighted term spans, and five hyperlinks with preserved typography.
The text matched the intended insertion. Native content and styles were checked;
rendered pagination was not visually inspected. Commands remain code-style text,
not native executable widgets. No training was started.

Trusted read: local ignored `outputs/doc-sync/trusted-read-05`; no protected
controls were detected. Final revision:
`ANLCKQn7R_dFfdVathqyfvSAiHO69l8gfoU_x6aCmvQdFX_2irh7TmpOPYYfVOgt2sSagF0fmAy7T9Hp6V_odFrcw5nz7_EMhzSKd-SNqyc`.
Repository code and raw evidence remain authoritative; previous milestone tabs
retain their historical statements rather than being rewritten as current status.

## Previous synchronization: dynamics, resets and Isaac Lab (03e9b3e)

Added **27 Dynamics, resets and Isaac Lab**, tab `t.egh5rmb516m4`, to the
[same Google guide](https://docs.google.com/document/d/1qeYucQnP4nznmLNJrlUz1hCIcY7KOigRPuhBVH-Zo0c/edit?tab=t.egh5rmb516m4).
It explains the control/physics/Lab boundaries, physical measurements, passing
subchecks versus failed overall warning gates, unsuccessful reset experiments,
performance limits, terminal-observation semantics and remaining runtime errors.
It includes five portable shaded commands and immutable source links.

Refreshed **00 Start here** to point to tabs 25–27 and distinguish verified Lab
reset/step compatibility from future training, sensing and hybrid traffic.
Connector readback verified 28 tabs, preserved original topology, unchanged
bodies for all 26 existing milestone tabs (01–26), ten new title/heading
paragraphs, five shaded Consolas commands, highlighted terms and five source
links with preserved typography. Verification used the native structure and
inherited text styles; PDF pagination was not visually inspected. Commands are
code-style text, not native executable widgets.

The initial trusted read is retained in local ignored
`outputs/doc-sync/trusted-read-04`. No old milestone tab was rewritten to make
its historical results look current. Repository tests/evidence remain authoritative.

## Previous synchronization: original foundation (955e47d)

The [Google engineering and learning guide](https://docs.google.com/document/d/1qeYucQnP4nznmLNJrlUz1hCIcY7KOigRPuhBVH-Zo0c/edit)
was updated with:

- **25 Hybrid roadmap and GitHub workflow**: the new development sequence,
  ownership boundaries, experiment safeguards and source-of-truth workflow.
- **26 Physics-driven car foundation**: controller/physics separation, units and
  timing, measured outcomes, both retained attempts, open reset warning and
  portable headless demonstration commands.
- Updated **00 Start here** overview: distinguishes historical milestones,
  current SUMO-motion demos and the new separate physical-car test.

Connector readback verified 27 tabs, preserved original titles/order/nesting,
new heading styles, shaded Consolas command paragraphs, highlighted key terms
and source links. The 24 historical milestone tab bodies (01–24) were compared
against the initial protected snapshot and remained unchanged. The source index
tab was intentionally updated. Native content/style verification passed;
rendered pagination was not visually inspected, and the styled commands are
code-style text rather than native executable/code-block widgets.

The protected read snapshot stays local in ignored `outputs/doc-sync/`. It is
not part of the GitHub payload. Repository implementation and evidence remain
authoritative; older notebook tabs describe their historical findings rather
than the current state of every subsystem.
