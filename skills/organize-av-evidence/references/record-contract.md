# Record contract

The registry has `schema_version: 1`, declared `scope`, and `activities`. Each
activity needs `id`, `title`, `domain`, `question`, `why_now`, `trigger` (text and
source refs), `method`, `status`, `result`, `limits`, `next_action`, `references`,
and `prefixes` used only for context grouping. Optional `follow_up_to` and
`supersedes` identify chronology without erasing history.

References use `repo:path/from/checkout`, `project:path/from/research-root`, or
an explicit HTTPS URL. File references must stay inside their named root. Do not
store credentials or private signed download URLs. Local catalogs can link private
assets; publishing that catalog or assets requires its own appropriate scope.

An activity's `artifacts` list contains explicit derived outputs:

```json
{
  "path": "repo:outputs/example/figure.png",
  "kind": "figure",
  "generator": "repo:outputs/example/plot.py",
  "inputs": ["repo:outputs/example/run/trace.jsonl"],
  "explanation": "Speed response after the specified leader-braking pulse.",
  "claim": "This run measures following response under the declared fixture.",
  "limits": "Synthetic demand; not a calibrated real-road result.",
  "lineage_basis": "recorded"
}
```

Use `lineage_basis: retrospective` for recovered historical links and describe
support in the activity references. `generator` can reference a procedure document
for manual annotation/authoring; identify the exact relevant section there. Every
declared input and generator must exist. Referencing a run manifest links the
artifact to that run's recorded sources/configuration without copying a huge graph
into the registry. It does not independently verify numerical correctness.

Coverage levels are intentionally distinct:

- **explicit_lineage:** declared question/trigger, generator and input references;
  reference existence validated. Inspect hashes/contents for stronger verification.
- **manifest_linked:** associated run manifest identifies this output; generator
  or exact derivation may still be unresolved.
- **context_linked:** recorded activity owns the package; exact derivation is open.
- **needs_backfill:** no recorded activity relationship yet.

Inventory of large/raw source files is metadata-only. Do not hash/read protected
observational content merely to improve catalog coverage. Hash small generated
results and graphics when eligible; otherwise report the exact verification scope.
An artifact can be discovered without its scientific meaning being established.

The HTML and Markdown views are generated editions. Keep source files stable and
retain the registry as the curated source of explanations. Never edit an HTML
view as if it were the authoritative registry. At session end report both useful
coverage and unresolved backfill, not just the number of inventoried files.
