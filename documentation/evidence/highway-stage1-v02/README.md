# Stage 1 preparation evidence v02

These are compact, derived evidence records. Their `source_path` and
`source_sha256` identify the complete immutable local reports under `outputs/`.
Per-vehicle trip lists are omitted here; cohort accounting totals/groups remain.
Full compressed action journals, sources, runtime logs and untrained export probes
remain local and are not included in Git. No trained weights exist for this milestone.

- `legacy-action-audit.json`: offline audit of the prior learned actor.
- `preflight-initial-failed.json`: first full preflight, 27/30 teacher cases passing.
- `geometry-verification.json` and `geometry-bank.json`: structural checks and
  portable identifiers/hashes for ten independently repeat-compiled layouts.
- `right-ramp-probe.json`: three formerly failing cases after geometry correction.
- `preflight-final.json`: final complete preflight on frozen runtime sources.
- `final-config.json`: reviewed proposal snapshot; training is disabled.
- `verification.json`: unit/regression results and configuration readiness check.

Preflight teacher cases use training layouts and dedicated diagnostic seeds.
Generated held-out geometry was inspected statically, but no held-out traffic
was evaluated. The teacher is a deterministic scripted controller using exact
SUMO state; these results do not establish learned or sensor-driven performance.
Changing ramp geometry prevents treating before/after metrics as a controller-only
comparison on identical scenarios. See the [technical note](../../highway-stage1-preparation-v02.md).
