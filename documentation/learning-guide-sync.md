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

## October 29: v04r1 retraining and results

Appended the imitation-assisted training method, exact budget/splits, measured passing and intervention results, one excessive-lane-switching episode and retained evidence to tab 41. Revision-guarded edit and full readback preserved all 42 tabs and all 41 unaffected tab bodies. Regenerated and checked from `outputs/doc-sync/mappo-v04-refinement/verified-source.json`. Native pagination was not inspected. No full acceptance, matched-seed gain or physical-transfer claim was added.

## October 29: v04r2 accepted lane-stability result

Appended the training refinement, exact budget/splits, accepted held-out result, selected-versus-final checkpoint distinction and retained metrics to tab 41. Revision-guarded native edit and full readback preserved all 42 tabs and all 41 unaffected tab bodies. Regenerated and checked from `outputs/doc-sync/mappo-v04-stability/verified-source.json`. Native pagination was not inspected. Simulator acceptance is explicitly separated from physical transfer and communication-benefit claims.


## October 30: Semantic mask and LiDAR fusion v01

Appended implementation, actual static/moving results, timing corrections,
preserved failures and limitations to tab 41. Updated the start-tab mirror link
to `codex/semantic-lidar-fusion`. Revision-guarded native editing and full readback
verified the exact addition, existing target history, all 42 tabs and all 40
unaffected tab bodies. Regenerated and checked the Markdown/manifest against
`outputs/doc-sync/semantic-lidar-fusion-v01/verified-source.json`. Native pagination
was not inspected. Actual screenshots and compact metrics are in the
[fusion evidence note](semantic-lidar-fusion-v01.md).


## October 30: Pretrained segmentation preview v01

Appended the frozen model, real prediction results, class mapping/exclusions,
actual screenshot links and no-training/no-new-simulation scope to tab 41.
Updated the start-tab link to `codex/pretrained-segmentation-preview`. Guarded
native editing and full readback preserved all 42 tabs, target history and 40
unaffected tab bodies. Regenerated and checked the Markdown/manifest from
`outputs/doc-sync/pretrained-segmentation-preview-v01/verified-source.json`.
Native pagination was not inspected; the actual four-panel PNGs were inspected.


## October 31: Saved-capture segmentation refinement v02

Appended measured preprocessing/boundary/occupancy results, limitations and the
disabled training proposal to tab 41; updated the start-tab mirror link. Guarded
native editing and full readback preserved all 42 tabs, existing target history
and 40 unaffected bodies. Regenerated/checked the mirror from
`outputs/doc-sync/segmentation-refinement-v02/verified-source.json`. Native
pagination not inspected; actual comparison/error screenshots were inspected.

## October 31: Segmentation training preparation v02

Appended actual dataset, trainer, no-update GPU results, disabled final configuration
and explicit limits to tab 41; start-tab link follows
`codex/segmentation-training-preparation`. Revision-guarded native editing and full
readback preserved all 42 tabs and 40 unaffected tab bodies. Regeneration/check
passed from `outputs/doc-sync/segmentation-training-preparation-v02/verified-source.json`.
Native pagination was not inspected. Actual screenshots remain in the
[preparation note](segmentation-training-preparation-v02.md).

## November 2: Highway curriculum with I-110

Appended selected Sam-source provenance, the 20-case driving curriculum, new actor
contract, actual no-learning results, high intervention limitation and disabled
pilot configuration to tab 41. Updated the start-tab mirror link to
`codex/i110-merge-training`. Revision-guarded full native readback preserved all
42 tabs and 40 unaffected tab bodies. Mirror regeneration/check passed from
`outputs/doc-sync/highway-curriculum-v01/verified-source.json`; native pagination
was not inspected.


## November 2: completed highway curriculum training pilot

Appended the approved run outcome, actual failed quality gates, original versus
final checkpoint comparison, ranking/recovery revisions and retained artifact
paths to tab 41. Revision-guarded native editing and full readback preserved all
42 tabs, all 41 unaffected tab bodies and prior target history. The exact append
was verified. Full mirror regeneration/check passed from
`outputs/doc-sync/highway-curriculum-training-v01/verified-source.json`.
Native pagination was not inspected. No acceptance, held-out adaptation or
additional training claim was added. See [results](highway-curriculum-training-v01.md).


## November 2: persistent sensor-driven training roadmap

Appended the three-stage plan, actual starting limits, proposed budgets, research
basis, project-memory procedure and low-usage execution rules to tab 41. Guarded
native editing and exact full readback preserved all 42 tabs, all 41 unaffected
tab bodies and prior target history. Full mirror regeneration/check used
`outputs/doc-sync/sensor-driven-roadmap-v01/verified-source.json`.
Native pagination was not inspected. No new training or simulator capability
was claimed; proposed budgets remain unapproved.
See [living roadmap](sensor-driven-training-roadmap.md).


## November 2: Stage 1 structured highway preparation v02

Appended the actual intervention diagnosis, 93-field contract, teacher/runner
preparation, failed first preflight, corrected ramp geometry and final 30-case
results to tab 41. The learning configuration remains disabled. Updated the
start-tab mirror link to `codex/highway-stage1-preparation`. Revision-guarded
native editing and full readback preserved all 42 tabs, all 40 unaffected tab
bodies and prior target history; exact append and start-link replacement verified.
Regenerated and checked the complete mirror from
`outputs/doc-sync/highway-stage1-v02/verified-source.json`. Native pagination
was not inspected. See [preparation evidence](highway-stage1-preparation-v02.md).


## November 3: completed Stage 1 training and versioned policy catalog

Appended the authorized run outcome, actual failed intervention gates, selected
versus lower-intervention phase comparison, retained weights/metrics and requested
versioned policy catalog to tab 41. Revision-guarded native editing and full
readback preserved all 42 tabs, 41 unaffected bodies and prior target history;
the exact append was verified. Regenerated and checked the complete mirror from
`outputs/doc-sync/highway-stage1-training-v02/verified-source.json`. Native
pagination was not inspected. No second training or held-out evaluation ran.


## November 3: Stage 1 v03 diagnostics and disabled revision

Appended the saved-journal audit, frozen training-label fit, matched teacher
comparison and measured speed tradeoff, ranking revision, disabled final config
and actual 98-test/30-case results to tab 41. Updated the start-tab mirror link
to `codex/highway-stage1-v03`. Revision-guarded native editing and full readback
preserved all 42 tabs, 40 unaffected bodies and prior target history; the exact
append and start-link replacement were verified. Mirror regeneration/check passed
from `outputs/doc-sync/highway-stage1-v03/verified-source.json`. Native pagination
was not inspected. No v03 learning or held-out test traffic was run.


## November 3: Stage 1 v03 completed training and recovery

Appended the authorized run, retained Windows checkpoint failure/resume, actual
validation improvements and regressions, intermediate-selection tradeoff and
experimental policy catalog entry to tab 41. Guarded native editing and exact
full readback preserved all 42 tabs, 41 unaffected bodies and prior target
history. Regenerated and checked the complete mirror from
`outputs/doc-sync/highway-stage1-training-v03/verified-source.json`. Native
pagination was not inspected. No held-out acceptance or physical transfer
was claimed, and no further learning attempt was launched.


## November 4: Stage 1 v04 improved training plan

Appended the proposed diagnosis, matched correction-memory experiment, budget,
selection and screening/acceptance distinction to tab 41. Updated the start-tab
mirror link to `codex/highway-stage1-v04-plan`. Revision-guarded native editing
and full readback preserved all 42 tabs, 40 unaffected bodies and prior history;
the exact append and scoped link change were verified. Regeneration/check passed
from `outputs/doc-sync/highway-stage1-v04-plan/verified-source.json`. Native
pagination was not inspected. No implementation or training outcome was claimed.


## November 4: variable-population MAPPO pilot

Appended the actual population implementation, final reviewed configuration, failed preparation checks, completed training, regressed matched results and native emergency-counter discrepancy to tab 41. Updated the start-tab GitHub link to codex/highway-population-v01. Revision-guarded native editing/readback preserved all 42 tabs, historical target content and the 40 unaffected tabs. The mirror regenerated and checked from `outputs/doc-sync/highway-population-v01/verified-source.json` (2,533 paragraphs, 14 tables, 182 code blocks). Native pagination was not inspected. No held-out, sensor, physical or live-handoff capability is implied.


## November 4: baseline review and population v02 catch-up

Appended the v02 failure result and restored-reference/public-AV comparison to tab 41. Revision-guarded native edits/readback preserved all 42 tabs, forty unaffected tab bodies and historical target content. Start link follows codex/policy-baseline-review. Full saved JSON exactly matches native readback; mirror regeneration/check passes (2,548 paragraphs, 14 tables, 182 code blocks). Source: outputs/doc-sync/policy-baseline-review/verified-source.json. Native pagination was not inspected. Earlier pending-sync notes remain historical.


## November 4: frozen population control and loss diagnostics

Appended no-learning acceleration measurements, objective-gradient findings, historical native-emergency reconciliation and proposed corrections to tab 41. The start-tab mirror link follows `codex/population-control-diagnostics`. Revision-guarded edits and full native readback preserved all 42 tabs, 40 unaffected tab bodies and the historical target-tab prefix. Corrected the event phase breakdown to 9 training / 1 initial validation / 2 final validation after independent review. Regenerated and checked the mirror from `outputs/doc-sync/population-control-diagnostics-v01/verified-source.json`. Native pagination was not inspected. [Results and measured plots](population-control-diagnostics-v01.md).


## November 4: population v03 control correction and final configuration

Appended actual near-limit/merge corrections, versioned observation semantics, shared targets, separate diagnostics, bounded results and the disabled final configuration to tab 41. Revision-guarded full native readback preserved all 42 tabs, 40 unaffected tab bodies and historical target content. Start-tab mirror link follows `codex/population-control-v03`. Saved source `outputs/doc-sync/population-control-v03/verified-source.json` exactly matches native readback; regeneration/check passed (2,565 paragraphs, 14 tables, 182 code blocks). Native pagination was not inspected. [Results and configuration](population-control-v03.md).

## November 5: frozen MAPPO physical adapter calibration

Appended the actual adapter, failed full-range brake fit, corrected calibration,
scripted physical passes and frozen-actor intervention failures to tab 41.
Changed the start-tab mirror link to `codex/mappo-physical-adapter-v01`.
Revision-guarded editing and full native readback preserved all 42 tabs, the
40 unaffected bodies and all prior content in the milestone tab. Regenerated
and checked the complete Markdown mirror against
`outputs/doc-sync/mappo-physical-adapter-v01/verified-source.json`.
Native pagination was not inspected. No policy training or promotion occurred.
[Results and actual screenshots](mappo-physical-adapter-v01.md).


## November 5: population v04 near-limit preparation

Appended the raw-acceleration diagnosis, localized physical-unit objective, four
speed ceilings, 171 tests, zero-update preflight, retained background emergency
and exact disabled training configuration to tab 41. The start-tab GitHub link
now follows `codex/population-near-limit-v04`. Revision-guarded native editing
and full readback preserved all 42 tabs, the 40 unaffected tab bodies and the
historical content of the milestone tab. Regenerated and checked the mirror
against `outputs/doc-sync/population-near-limit-v04/verified-source.json`. Native
pagination was not inspected. No experiment training or policy promotion.
[Final configuration and evidence](population-near-limit-v04.md).


## November 5: I-110 frozen-policy merge visual test

Appended the merge-specific policy selection, fixed-seed results, failed intervention gate, completed video/screenshot verification and physical-validation limits to tab 41. The start-tab mirror link follows `codex/i110-merge-visual-v01`. Revision-guarded editing and full native readback preserved all 42 tabs, forty unaffected tab bodies and the milestone tab's historical content. Regenerated and checked the mirror from `outputs/doc-sync/i110-merge-visual-v01/verified-source.json` (2,589 paragraphs, 14 tables, 182 code blocks). Native pagination was not inspected. [Results and actual screenshots](i110-merge-visual-v01.md). No training or policy promotion.


## November 5: environment v05 training and SimReady replay

Appended implementation, all preserved failures, budget-preserving recoveries, actual training results, rejected-policy artifacts and both SimReady visual cases to tab 41. The start-tab mirror link now follows `codex/highway-environment-v05`. A revision-guarded edit and full native readback verified all 42 tabs, the exact historical target prefix and 40 unchanged unrelated tab bodies. The generated mirror and its check pass from `outputs/doc-sync/highway-environment-v05/verified-source.json` (2,603 paragraphs, 14 tables, 182 code blocks). Native pagination was not inspected. [Measured results](highway-environment-v05-results.md) explicitly retain the intervention regression and I-110 failure; neither copy claims an accepted policy.


## November 6, 2026: I-110 merge blockage correction

Appended the network-conflict diagnosis, matched frozen-policy results, failed probes and getter caveat, SimReady replay, remaining intervention failure and no-training next step to tab 41. The start-tab Markdown link now targets `codex/i110-merge-blocking-fix`.

A revision-guarded native write and complete readback preserved all 42 tabs, the 40 unrelated tab bodies and the historical target prefix. Regeneration/check passed from `outputs/doc-sync/i110-merge-blocking-v01/verified-source.json`: 2,612 paragraphs, 14 tables and 182 code blocks. Native page layout was not inspected. Both copies contain the same verified source; source assets, raw captures and videos remain local.


## November 6, 2026: remaining overrides and smooth-reference proposal

Appended the remaining intervention attribution, observed approach slowdown and jerk, train-only teacher comparison, fresh SimReady replay, disabled proposed weights/budget and approval gates to tab 41. The start-tab mirror link follows `codex/highway-smooth-control-v01`.

Revision-guarded native editing and full readback preserved all 42 tabs, the historical target prefix and 40 unaffected tab bodies. Regeneration and check passed from `outputs/doc-sync/highway-smooth-control-v01/verified-source.json`: 2,621 paragraphs, 14 tables and 182 code blocks. Native page layout was not inspected. Both copies clearly retain the 8.52% failure and no-training status. [Evidence](highway-smooth-control-v01.md); [proposal](highway-smooth-training-plan-v01.md).


## November 6, 2026 — Smooth-control preparation v01

Appended implemented reference/reward/trainer behavior, 44 preserved full native runs, retained v4 results, rejected v5, four comfort failures, disabled final configuration, test outcomes and SimReady replay limitations to tab `t.k82ts3ro3qz9`. Updated only the start-tab mirror URL to branch `codex/highway-smooth-preparation-v01`.

Revision-guarded native editing and a full readback preserved all 42 tabs, all 40 unaffected tab bodies and the exact historical target-tab elements. The intended target addition and start-tab URL replacement matched exactly. Regenerated and checked the mirror from `outputs/doc-sync/highway-smooth-preparation-v01/verified-source.json`: 2,630 paragraphs, 14 tables and 182 code blocks. Native page layout was not inspected. Raw source reads remain local. [Results and screenshots](highway-smooth-preparation-v01.md).


## November 6, 2026 — Highway training rethink v01

Appended the read-only replay/control/reward/weight audit, measured regressions, new proposed sequence/budgets and no-training status to tab `t.k82ts3ro3qz9`. Updated only the start-tab mirror URL to `codex/highway-training-rethink-v01`.

The fresh pre-edit full read matched the previous verified source. Revision-guarded scoped editing and full readback preserved all 42 tabs, the 40 unaffected tab bodies and exact historical target-tab elements. Target addition and start-tab replacement matched exactly. The regenerated mirror passed check from `outputs/doc-sync/highway-training-rethink-v01/verified-source.json`: 2,640 paragraphs, 14 tables, 182 code blocks. Native pagination was not inspected. [New plan and actual saved-sample plot](highway-training-rethink-v01.md).
