# I-110 team handoff — 2 November 2026

## Preview v2/v2.1 follow-up — 2 November 2026

V2.1 passes all six primary development acceptance checks. The rejected v2 edition is retained. Two separately frozen matrices
completed 32 runs, with exact baseline/repeat traces and 54 passing targeted
CPU tests. See [comparison and original-car status](i110-preview-v2-2026-11-02.md).
The original supplied car/source remain intact locally; the public replay uses
procedural cars because redistribution rights are unqualified. The released v1
ZIP is unchanged. Freeze new, uninspected verification seeds and merge/cut-in conditions before further tuning; test range sensitivity and realization limits. Keep smooth ACC as the reference baseline.

## Published handoff — 2 November 2026

Sam approved publication after reviewing the prepared package. Source checkpoint
`1c9c27d` is now on `samfrausto/hivemind-traffic`, branch
`codex/sam-visual-twin`. [PR #16](https://github.com/hudsonkaneko/hivemind-traffic/pull/16) now describes the I-110 work and its
actual acceptance failures; its existing open/non-draft review status is preserved.
The branch has not been merged into main.

Download the [portable road and paired Isaac replay](https://github.com/samfrausto/hivemind-traffic/releases/tag/sam-i110-handoff-20261102-v1)
(`team-package-v1.zip`, 8,024,356 bytes). GitHub's asset SHA256 matches the verified
local ZIP: `ed0052aed1bfc6c4819c9f7049eb74fa713e17c29a1842cffca8bfe624c9347f`.
Extract it, set `HIVEMIND_ISAAC_PYTHON` to your existing Isaac Sim Python, and run
`Open Comparison.cmd`; the road alone is `scene/road/world.usda`.
The immutable ZIP contains the preparation-time progress notes. This repository
handoff and release page provide the subsequent publication status.

Preview v1 remains rejected; smooth ACC remains the baseline. The next engineering
task is current-path relevance and native ACC realization/recovery before a fresh
predeclared v2 comparison. Shared Google Doc writes remain on hold.
Verification: [publication record](evidence/i110-publication-20261102.json).

Prepared by Codex for Sam. This handoff introduces no permanent team assignments
or new scientific acceptance.

## Current entry points

- [Latest controller comparison](i110-preview-2026-11-02.md): preview v1 is tested
  but rejected as the baseline; retain smooth ACC. It improves difficult ramp
  peaks but creates new control-case jerk and 21–31% cut-in cohort-time costs.
- [Road geometry/contact evidence](i110-geometry-contact-refinement.md): current
  image/lidar-fitted road, explicit interpolation and provisional lane/taper
  geometry; bounded contact checks are separate from traffic/driver acceptance.
- [Complete roadmap](sam-roadmap-2026-11-02.md) and
  [Hudson repository catch-up](repository-review-2026-11-02.md): v04r2 update 175 is
  the reported accepted original passing candidate; I-110 support is unproven.
  Six-class perception preparation exists, but training was disabled in the
  reviewed checkpoint. Physical/sensed driving and communication benefit remain open.
- [Current technical handoff](continuation-2026-10-29.md) preserves the full chain
  of baseline, comfort failures, delivered-v02 parity and v04 support dependencies.

## Open the paired road replay

The prepared portable package is under
`outputs/i110-preview-20261102/team-package-v1/`, with a sibling ZIP and hashed
package record. It contains the self-contained road layers, both recorded motion
streams on a shared clock, procedural vehicle assets, a standalone viewer and
these progress notes. It needs an already installed compatible Isaac Sim Python;
no SUMO process, training environment, PeMS account or private supplied car is
needed for playback. Set `HIVEMIND_ISAAC_PYTHON` to that Python, then run the package
launcher. In Isaac’s file UI, the road alone can be opened from `scene/road/world.usda`.

Left/blue is the frozen baseline; right/green is the 240 m preview. Orange means
ramp-origin traffic. This is a recorded SUMO replay; no Isaac physics drives the
cars. Original x/y/z and timestamps are preserved with two documented display
translations. Yaw/pitch is reconstructed from the lane graph, not sensor truth.

## Run or extend the traffic experiment

Use the existing isolated traffic environment and the committed source/config.
`scripts/build_i110_sumo.py` builds from `scenarios/i110/sumo-road-profiles.json`.
The portable package also carries the exact frozen network used in this comparison.
Start with ordinary smooth ACC and complete trip accounting. Review every
controlled, human, ramp, external-wait and unfinished burden before claiming an
improvement. The old raw baseline suites are local artifacts, so a public clone
alone does not reproduce the historical hashes; the prepared package contains
the demonstrated pair, while full historical raw studies remain local.

Next engineering work: current-path relevance, native ACC realization and preview
recovery, followed by a freshly declared bounded v2 comparison. Keep factual
geometry/traffic qualification and the v04 handoff separate. Do not feed the
three-lane actor an invented I-110 observation mapping or begin training without
the agreed contract and budget.

## Preparation checkpoint (historical, before publication)

Verified remote feature head: `b23f24c07711080045c65f41c5254ab90df5929a`.
It has no I-110 files; PR 16 still reflects the older I-80/exiD work. Thirteen
local commits already followed it at task start, plus this milestone. The road
package and current progress notes are prepared locally, not uploaded. Proposed
publication is the tested feature-branch update, an accurate PR 16 description,
and the self-contained road/replay ZIP as a versioned GitHub artifact. Do not
merge into main as part of that handoff.

The shared Google research document remains on Sam’s existing write hold; its
latest readable checkpoint is 22 October. Local document-ready updates are kept.
No teammate was messaged, and no unrecorded teammate milestone is claimed.
