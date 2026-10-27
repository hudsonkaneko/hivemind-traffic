---
name: organize-av-evidence
description: Organize this AV research project's questions, decisions, runs, graphics and results into a traceable evidence catalog. Use when planning or closing project research/development work, creating figures, explaining results, or reconciling historical artifacts.
---

# Organize AV evidence

Sam wants every graphic and result connected to why it exists and what produced
it. Make the project understandable from one catalog without moving immutable
run packages or confusing a polished picture with validation.

## Locate and apply

Resolve the research project from `PROJECT_REFERENCE.md` and the active Git
checkout from its `AGENTS.md`. The maintained registry is
`documentation/evidence-catalog/activities.json` in that checkout. This skill's
installed location is not a project path. Read [the record contract](references/record-contract.md)
when registering or backfilling artifacts. Use the existing research custodian
for approved decisions and session continuity; this catalog supplies artifact
lineage and explanations, not a competing research roadmap.

Apply this workflow to all substantive project work: literature, geographic
qualification, geometry, visual assets, physics, ordinary driving, policy
interfaces, learning, traffic events, analysis, figures and presentations.
Vendor runtimes, caches and unrelated machine files are outside the research
catalog. Raw/private source assets may be indexed locally without publishing
their bytes or inferring rights.

## Before producing evidence

Record an activity with a stable ID, question, reason for doing it now, trigger
and source, intended comparison/check, inputs, code/configuration, acceptance
criteria and claim limits. A trigger may be Sam's request, a previous failed
check, a literature gap or a planned dependency. Cite the actual source; an
assistant suggestion is not a team decision. The activity can be brief for a
small edit, but its reason should not be an empty boilerplate sentence.

Keep one authoritative home per item. Use existing repository conventions:
source in code directories, compact records in `documentation/`, raw runs in
fresh `outputs/<study>/<run-id>/` directories, and the existing research editions
at the research root. Use meaningful names and stable IDs. Do not relocate,
rename, overwrite or delete old evidence merely to make a tidier folder tree.

## For each run and derived artifact

Retain actual inputs and hashes, source commit plus dirty source snapshot, exact
configuration/seed/command, runtime versions, result status and failure logs.
For every derived graphic, image, chart, video or result table, register:

- the activity/question and trigger;
- the exact upstream runs/files and generator or authoring procedure;
- a plain-language explanation of what is shown and why it was made;
- the numerical/result claim it supports, limits, and whether it changes a decision.

Differentiate measured outputs, scripted illustrations, source imagery,
reconstructions, predictions and AI-generated imagery. For source imagery include
publisher, acquisition epoch and rights status. For renders include scene/layer
and camera/preset inputs. An image edit is a new derived artifact with the original
as input. Recompute result tables from raw records rather than typing results.

Preserve failed attempts and corrections with `supersedes`/`follow_up_to` links.
Explain which changed factor is supported as a cause; matched intervention runs
can support a local causal explanation, while visual resemblance, correlation,
shared folder names and file hashes cannot establish one.

## Close and explain

Update the registry's result, interpretation and next action. Run this skill's
`scripts/build_catalog.py --project PROJECT --repo CHECKOUT --output FRESH_OUTPUT`
to build a searchable local HTML catalog, Markdown activity index, machine-readable
inventory and coverage report. The scanner is read-only outside that fresh output.
Inspect validation errors, missing references and provenance gaps before reporting
completion. Do not call context-only grouping complete lineage.

Present the question → reason → method/run → evidence → interpretation → next
decision in plain language. Link the specific artifacts and identify material
uncertainties. Reconcile the research custodian's current state/history and prepare
the local document-ready update. Preserve Sam's shared-document write hold.

## Historical backfill

Inventory declared research roots, then connect evidence using original manifests,
scripts, dated reports and recorded requests. Keep the historical run untouched.
Record reconstructed links as retrospective, distinguishing source evidence from
curator inference. If the generator or motivating question cannot be recovered,
mark `needs_backfill` and expose it in the catalog; never fabricate a causal story.
Read only needed content. Metadata-only inventory of a protected audit dataset
does not authorize opening its observations or consuming that holdout.

The skill authorizes no training, data publication, teammate messaging, shared
document edits or scheduled background work. Apply it during authorized sessions.
