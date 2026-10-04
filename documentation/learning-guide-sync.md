# Engineering guide synchronization

Last verified: October 3, 2026.

Latest source milestone: commit `03e9b3e`, published on `codex/physics-vehicle-foundation`.
[Pull request #1](https://github.com/hudsonkaneko/hivemind-traffic/pull/1) records
the implementation, tests, evidence and remaining gates. `main` is unchanged.

## Latest update: dynamics, resets and Isaac Lab

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
