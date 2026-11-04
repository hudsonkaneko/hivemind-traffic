# Versioned driving policies

Frozen inference weights and short, evidence-backed model records.

| Version | Type / inputs | Status | Intended scope |
| --- | --- | --- | --- |
| [MAPPO v04r2](mappo-v04r2/README.md) | Imitation-assisted MAPPO / 81 fields | Accepted in its original SUMO fixture | Straight three-lane following and passing |
| [Highway Stage 1 v02](highway-stage1-v02/README.md) | BC + MAPPO / 93 fields | Experimental; override gate failed | Varied highway cruise/follow/pass/merge |
| [Highway Stage 1 v03](highway-stage1-v03/README.md) | Acceleration-weighted BC + MAPPO / 93 fields | Experimental; override gate failed | Varied highway cruise/follow/pass/merge |

Each folder includes `policy.pt`, a short `README.md`, exact
`training-config.json`, `contract.json`, `evaluation.json` and a
`manifest.json` with hashes/source provenance. These are frozen actor exports;
full resume checkpoints and bulk training logs remain under local `outputs/`.

The latest version is not automatically the accepted version. Acceptance labels
apply only to the named evaluation protocol. SUMO controls motion; these
packages do not provide sensor-driven or physical vehicle control. Input width alone
does not establish compatibility: use the exact ordered contract and matching
environment. Acceleration output is normalized, not throttle or an unfiltered
physical acceleration command.

## Add a version

1. Create a new descriptively versioned folder; never overwrite existing weights.
2. Copy the selected actor and its exact configuration, contract and evaluation.
3. Record changes, intended behavior, measured results, limitations, source commit
   and SHA-256. Label unevaluated/rejected candidates explicitly.
4. Verify checksums, CPU loading, input/output shape and finite inference; record
   that a loading smoke test is not a driving evaluation.
5. Update this index and the project changelog. Keep acceptance promotion separate
   from publishing a newer version.
