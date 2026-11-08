# Versioned driving policies

Frozen inference weights and short, evidence-backed model records.

| Version | Type / inputs | Status | Intended scope |
| --- | --- | --- | --- |
| [MAPPO v04r2](mappo-v04r2/README.md) | Imitation-assisted MAPPO / 81 fields | Accepted in its original SUMO fixture | Straight three-lane following and passing |
| [Highway Stage 1 v02](highway-stage1-v02/README.md) | BC + MAPPO / 93 fields | Experimental; override gate failed | Varied highway cruise/follow/pass/merge |
| [Highway Stage 1 v03](highway-stage1-v03/README.md) | Acceleration-weighted BC + MAPPO / 93 fields | Experimental; override gate failed | Varied highway cruise/follow/pass/merge |
| [Highway population v01](highway-population-v01/README.md) | Warm-started MAPPO, masked population critic / 93 fields | Experimental; regressed, 15/16 validation cases failed | Varying AV count, finite traffic cohort and received peers |
| [Highway population v02](highway-population-v02/README.md) | Acceleration-weighted population MAPPO / 93 fields | Experimental; correction insufficient, 12/16 cases failed | Same variable-population fixture |
| [Stage 1 v03 phase 0](highway-stage1-v03-phase0/README.md) | Shared MAPPO / 93 fields | Restored analysis reference; not accepted | Lowest pooled/worst interventions among population-tested learned candidates |
| [Highway environment v05](highway-environment-v05/README.md) | MAPPO with continuous maneuvers / 99 fields | Rejected candidate; stable merges improve but interventions regress; 19/24 cases fail | 1–24 controlled cars, varied roads/demand, eight-second stable merges |
| [Highway flow RL v01](highway-flow-rl-v01/README.md) | Teacher-free MAPPO / 117 fields | Experimental; 7.61% pooled overrides, comfort still fails; one stable I-110 merge | Mixed highway tasks, native maneuvers and received V2V; four development cases |
| [Highway flow RL v02](highway-flow-rl-v02/README.md) | Prepared MAPPO / 117 fields | Not trained; frozen-parent diagnostics fail driving gates | Visibility correction, cruise-speed and route-detour rewards |

Prepared versions have no new actor weights until training completes.

Folders include frozen `policy.pt`, a short `README.md`, exact training settings and linked contract/evidence/checksums. Legacy packages use `contract.json`, `evaluation.json` and `manifest.json`; population v02 uses `policy-contract.json` and its linked evidence record. These are frozen actor exports;
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
