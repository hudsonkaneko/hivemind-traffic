# Engineering guide synchronization

Last verified: October 4, 2026.

Latest source milestone: commit `531f748`, published on `codex/physics-vehicle-foundation`.
[Pull request #1](https://github.com/hudsonkaneko/hivemind-traffic/pull/1) records
the implementation, tests, evidence and remaining gates. `main` is unchanged.

## Latest update: visible physical car and LiDAR view

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
