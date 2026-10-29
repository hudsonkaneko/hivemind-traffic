# Engineering guide synchronization

## October 25: sensor prototype checkpoint

Final native result was subsequently appended in place: 8.52 m travel, 2.57 m
clearance, fault braking/recovery, 163 CPU tests and 18 USD tests, with the failed
attempts and revised 3 m stopping criterion disclosed. Full readback preserved
all 41 unaffected tab bodies. The final mirror/check uses
`outputs/doc-sync/sensor-perception-v01-final/verified-source.json`; the earlier
pending checkpoint remains historical evidence rather than being rewritten.

Appended the forward sensor v01 implementation and explicitly pending native
validation to tab **41 Isaac Lab lane task**. Updated the start-tab mirror link
to `codex/sensor-perception-v01`. Revision-guarded native readback verified the
exact addition, all 42 tabs and unchanged bodies for the 40 unaffected tabs.
The complete Markdown mirror passed regeneration/check from
`outputs/doc-sync/sensor-perception-v01/verified-source.json`. Native document
pagination was not inspected. [Draft PR 21](https://github.com/hudsonkaneko/hivemind-traffic/pull/21)
records the CPU-tested checkpoint; no native driving or training result is implied.

## Chat-managed Markdown mirror

The complete [Engineering & Learning Guide](engineering-guide.md) can now be read
on GitHub. Its source is the existing native Google document
`1qeYucQnP4nznmLNJrlUz1hCIcY7KOigRPuhBVH-Zo0c`; its binding, last verified source
revision, tab inventory and Markdown SHA-256 are in
[`engineering-guide.sync.json`](engineering-guide.sync.json).

The user authorized this ongoing workflow on October 22, 2026. A guide update
requested through the project chat must update both copies, without another
permission prompt. This does not create a background poller, scheduled task,
Google webhook, or automatic bidirectional GitHub integration.

1. Read the current document with the Google Drive connector's `get_document`.
   Read all tabs, even if the browser URL points at only one. Compare the tab
   inventory/revision with the manifest and reconcile any intervening changes.
2. Make the requested scoped native changes using `batch_update_document` with
   `write_control.requiredRevisionId`. Preserve unrelated text, native formatting,
   tabs and history. If the request starts from an edit to Markdown, reconcile
   it into the corresponding native section before regenerating the mirror.
3. Read the full native document back once. Verify the intended edit and
   preservation of unaffected tabs. Save the returned `structuredContent` JSON
   in a new directory under ignored `outputs/doc-sync/`; do not commit raw reads,
   access tokens, signed download URLs or comments.
4. Render and verify from the repository root with normal traffic Python:

   ```text
   python scripts/sync_engineering_guide.py --source outputs/doc-sync/<run>/source.json
   python scripts/sync_engineering_guide.py --source outputs/doc-sync/<run>/source.json --check
   ```

   The script has no Google credentials and performs no network writes. It binds
   the exact native document to the fixed Markdown/manifest paths. A partial
   read, missing existing tab, unsupported element or locally edited mirror
   stops the refresh instead of silently losing content. Do not delete the
   manifest to bypass a conflict; preserve the edits, reconcile both versions,
   and review any explicit repair before refreshing.
5. Review the rendered headings, tables, commands and links. The renderer retains
   wording, tab order, headings, bold/italic text, hyperlinks, simple tables and
   monospace command blocks. Page layout, fonts, colors, comments and suggestion
   workflows are not reproduced. New images, lists, footnotes or merged tables
   require explicit converter support and preservation checks before refreshing.
6. Update this sync record and the changelog for a meaningful milestone, then
   commit/push the Markdown, manifest and related changes together. Follow the
   normal feature-branch/PR workflow. A local commit alone is not a GitHub update.
   The Google guide's GitHub link should follow the active published branch and
   be updated to `main` after this documentation feature is reviewed and merged.
7. If native editing or GitHub publication fails, name the pending copy in the
   final response. An unchanged local manifest does not prove that Google Docs
   has not changed since the last read. Edits made outside this chat are imported
   at the next requested synchronization, not continuously monitored.

## October 25: SUMO MAPPO setup

Appended the separate headless SUMO/MAPPO setup, exact proposed training
parameters, no-learning preflight results, failed scripted override criterion
and future transfer boundaries to **41 Isaac Lab lane task**. No training or
learned-driving result is claimed. The start-tab Markdown link follows
`codex/sumo-mappo-setup`. Revision-guarded editing and full readback verified the
exact insertion, unchanged historical last-tab body and all 40 unaffected tabs.
All 42 tabs were regenerated and checked from
`outputs/doc-sync/sumo-mappo-20261025/verified-source.json`. Native pagination
was not inspected. See [the setup and final proposal](sumo-mappo-setup.md).

## October 25: CIRCLES highway preparation

Appended the prepared 60 mph ceiling / CIRCLES main-car / two PPO lane-car
protocol and its deferred driving status to **41 Isaac Lab lane task**. The
note records the failed zero-step native attempt, the scene-export repair and
55 passing CPU/OpenUSD checks without implying a successful highway drive.
The start-tab Markdown link now follows `codex/highway-60mph-ppo-traffic`.
Revision-guarded editing and full native readback verified the exact additions,
unchanged historical last-tab content and all 40 unaffected tabs. All 42 tabs
were regenerated and checked from
`outputs/doc-sync/highway-60mph-20261025/verified-source.json`. Native pagination
was not inspected. See [the prepared experiment](highway-circles-60mph.md).

## October 22: CIRCLES control smoke

Appended the bounded seven-export physical-car result to **41 Isaac Lab lane
task** and changed the start-tab Markdown link to `codex/circles-control-smoke`.
The note distinguishes six limited actuation passes from zero strict stopping
successes and preserves the interpolated controller's obstacle collision.
Native readback verified the exact appended text, the start-tab link replacement,
and all 40 unaffected tab bodies/metadata. The 42-tab mirror was regenerated and
checked from `outputs/doc-sync/circles-20261022/verified-source.json`.
The manifest records the verified revision; native pagination was not inspected.
See [the experiment record](circles-control-smoke.md) for the measured scope.

## October 22: initial complete conversion

Converted all 42 existing tabs and 14 native tables with 182 monospace command
paragraphs. There were no embedded images, native lists, footnotes, headers or
footers; existing screenshot hyperlinks remain links to the original assets.
Added a reciprocal GitHub link and synchronization explanation to **00 Start
here**. Historical milestone content is retained, including the October 19
training proposal; conversion does not revise those earlier claims to describe
later training. The exact source revision and final counts are in the manifest.

The native source and Google's direct Markdown export are retained locally under
`outputs/doc-sync/markdown-20261022/`. Native structure is used for the published
conversion because the direct export flattens heading levels and loses command
styling. Targeted tests cover text/table/code preservation, all-tab rendering,
revision checks, unsupported features and refusal to overwrite local edits.

Verification: all 10 targeted tests passed. GitHub's own Markdown renderer
preserved all 1,742 nonempty native paragraphs, 42 tab anchors, 14 tables and
182 command blocks. Native readback confirmed the exact added text and unchanged
bodies for the other 41 tabs. Local render output and the comparison report are
retained in the same ignored directory; no Google pagination audit is claimed.

## Verified October 12: project-local skills

Added **38 Project skills in GitHub**, tab `t.124coz3q4xks`, to the
[Google engineering guide](https://docs.google.com/document/d/1qeYucQnP4nznmLNJrlUz1hCIcY7KOigRPuhBVH-Zo0c/edit?tab=t.124coz3q4xks).
It explains the eight repo-local domain skills, use/discovery, provenance,
architecture and training boundaries, private-data exclusions and validation.

Revision-guarded writes and native readback verified exact inserted text,
title/headings, two shaded Consolas examples and two source links. All 38
previous tab bodies, titles and order were preserved; there are now 39 tabs.
No rendered pagination audit is claimed. Source: [skill bundle](project-skills.md).
No runtime, asset, simulation or training changes accompanied this publication.

## Verified October 10: showcase performance

Added **35 Showcase performance**, tab `t.dwsxfv73kedy`, to the
[Google engineering guide](https://docs.google.com/document/d/1qeYucQnP4nznmLNJrlUz1hCIcY7KOigRPuhBVH-Zo0c/edit?tab=t.dwsxfv73kedy).
It explains v06's shared reads, exact-math reuse and lighter display work;
presentation versus full evidence; profiling/RTF definitions; portable shaded
commands; repeated physical results; failures and the still-open smooth timing
gate. No earlier result was rewritten to imply improved timing.

Revision-guarded native writes and final readback confirmed exact inserted text,
headings, four shaded Consolas command blocks, the PR #10 link, and all 35
previous tabs unchanged. The document now has 36 tabs. Simulator screenshot
inspection was separate; Google pagination/pixel verification is not claimed.
Local ignored verification: `outputs/doc-sync/showcase-performance-verification.json`.
Source: [v06 performance evidence](showcase-performance.md). No training started.

## Verified October 10: loop traffic showcase

Added **34 Loop traffic showcase**, tab `t.9ah0315nta8b`, to the
[Google engineering guide](https://docs.google.com/document/d/1qeYucQnP4nznmLNJrlUz1hCIcY7KOigRPuhBVH-Zo0c/edit?tab=t.9ah0315nta8b).
It covers the verified v04 milestone and v05 density follow-up: purpose,
movement ownership, scripted/privileged observation limitations, portable
commands, highlighted definitions, measured evidence, real-time limitations,
retained failures and the explicit fleet shutdown. PR #10 is linked.

The current connected Google Docs workflow succeeded through structural native
reads and a revision-guarded write; the older October 9 trusted-reader failure
is historical. A full native readback confirmed the inserted text, title and
heading styles, all five shaded Consolas command blocks, and that all 34 older
tabs were unchanged. No pixel/pagination audit is claimed. Sources:
[v04 record](loop-traffic-showcase.md), [dense v05 record](dense-loop-showcase.md)
and [version folders](../showcases/loop_traffic/README.md). No training launched.

## Pending: mentor-guided continuous highway loop

The local [loop implementation and evidence note](continuous-highway-loop.md)
and [branch roadmap](mentor-loop-roadmap.md) are the current source for this new
milestone. Existing Google tabs were not modified. A fresh attempt to use the
required file-backed reader failed with `workspaceRoot must be an absolute path`
for the canonical Windows directory. Native write/format/readback verification
therefore remains pending; the verified synchronization below is historical.

Previous verified milestone: October 6, 2026.

Latest source milestone: commit `e74b574`, published on `codex/realtime-preview`.
[Pull request #5](https://github.com/hudsonkaneko/hivemind-traffic/pull/5) records
the implementation, tests, evidence and open GUI timing gate. It is stacked on
`codex/demo-readiness`; `main` is unchanged.

## Historical update: live preview timing

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


## October 26: MAPPO v03 communication and wave setup

Appended setup, final proposed pilot, actual preflight and limits to tab41; updated start-tab mirror link to codex/mappo-communication-waves. Before editing, full native source matched the existing local mirror, reconciling prior native guide updates. Revision-guarded append and complete native readback preserved all42 tabs and40 unaffected tab bodies. Regeneration/check succeeded from outputs/doc-sync/mappo-waves-v03/verified-source.json. Native pagination not inspected. No training launched.

## October 29: MAPPO v04 setup and v02 replay/release follow-up

Full native read matched the existing mirror before editing. Appended verified v04 setup, final proposed pilot and actual preflight evidence to tab 41, plus the earlier v02 sensor/release milestones; updated the start-tab mirror URL to codex/mappo-v04-cooperative-passing. Revision-guarded edit and full native readback preserved all 42 tabs and 40 unaffected bodies. Mirror regeneration/check succeeded from outputs/doc-sync/mappo-v04/verified-source.json. Native pagination was not visually inspected. No training launched.

## October 29: completed v04 training pilot

Appended actual training outcome, failed learned acceptance, best-versus-latest checkpoint comparison, scripted baseline, retained metrics and budget status to tab 41. Fresh revision-guarded native editing and full readback preserved all 42 tabs and all 41 unaffected tab bodies. Regenerated and checked the complete mirror from `outputs/doc-sync/mappo-v04-training/verified-source.json`. Native pagination was not inspected. No accepted-checkpoint or physical-transfer claim was added.
