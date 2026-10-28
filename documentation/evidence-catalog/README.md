# AV evidence catalog

Sam's organizing rule, 27 October 2026: every substantive activity and every
graphic/result should explain **what question prompted it, why we ran it,
what produced it, what it shows, and what follows from it**.

The [organization skill](../../skills/organize-av-evidence/SKILL.md) applies this
workflow across literature, site selection, geometry, visual assets, physics,
traffic, policy interfaces, learning, events and reporting. Repository and project
AGENTS.md make it a continuing session practice. It complements the research
custodian; it does not create a second scientific plan.

Current local edition: [open the searchable catalog](../../outputs/evidence-catalog/20261028-stress-final/index.html).
It covers 19 activities across the research folders and full repository, including
the cut-in/braking stress tranche. The edition's coverage.json gives exact counts and unresolved
relationships. Generated catalogs are ignored local outputs and are not included
in a Git clone. The installed skill is `organize-av-evidence`; this repository
contains its maintained source. File-reference/schema tests pass. An automated
browser preview was blocked because the app browser disallows local-file URLs,
so visual browser verification remains outstanding.

## Where things belong

| Material | Authoritative home |
|---|---|
| Questions, motivations, relationships, figure explanations | [activities.json](activities.json) |
| Approved research and source ledgers | Existing research-root editions and research index |
| Maintained code/config/tests | Existing repository code directories |
| Compact milestone claims and evidence | `documentation/` and `documentation/evidence/` |
| Raw runs, failures, source snapshots, traces | Fresh directories in `outputs/` |
| Derived plots and result tables | Named analysis edition beside the study's runs |
| Searchable index and coverage | Fresh `outputs/evidence-catalog/<edition>/` |

Keep original paths stable. A tidy index is safer than renaming historical run
packages and breaking reproducibility. Catalogs are local, contain local/private
links, and are neither public publications nor backups.

## Use at every session

Before a run, add or update the activity's question, actual trigger, method and
acceptance criteria. At close, register every new graphic and derived result with
its exact inputs, generator, explanation, supported claim and limitations. Record
failed attempts and their replacements. The [record contract](../../skills/organize-av-evidence/references/record-contract.md)
defines the small JSON schema. Do not infer a causal history from a folder name.

Build from the repository with a standard Python interpreter:

```text
python skills/organize-av-evidence/scripts/build_catalog.py --project "ABSOLUTE_RESEARCH_ROOT" --repo . --output outputs/evidence-catalog/NEW_EDITION
```

Open that edition's `index.html` in a browser. Current work appears first;
historical activity cards expand on demand. Search/filter the artifact table by
name, type, activity or provenance. `inventory.json` is machine readable,
`coverage.json` lists issues, and `index.md` is a readable fallback.

## What coverage means

- **explicit_lineage:** a curated record names the artifact's generator, exact
  inputs, explanation, claim and limits. Retrospective links are labeled.
- **manifest_linked:** a run manifest associates the output with its run; hashes
  are checked when the file was hashed. This is not a full explanation.
- **context_linked:** the activity is known, but exact derivation is uncurated.
- **needs_backfill:** the file is inventoried but its activity/lineage is unresolved.

The first edition inventories existing scoped project material and backfills key
recent images. Historical backfill is **not complete**. Coverage makes that
remaining work measurable instead of pretending every past result is explained.
The scanner excludes vendor runtimes, caches, Git internals, recovery archives and
catalog editions themselves. Protected research inputs are metadata-only; scanning
does not authorize opening observations or consuming an audit holdout.

For the current work, start with [ACC stress/event results](../i110-acc-stress-2026-10-28.md)
and [the ordered remaining tracks](../i110-transfer-and-next-tracks-2026-10-28.md).
