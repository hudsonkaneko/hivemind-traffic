# V04 merge preparation and complete-episode training

November 10, 2026. User authorized implementation and long, high-epoch training.
Preparation passed; training results are pending. This is a separate experimental
candidate, not an accepted replacement for a published policy.

## Why this version exists

The [v03 audit](highway-policy-audit-v01.md) found that the focal car received no
speed objective for 344/549 decisions, most often when no immediate merge gap
existed. Its lane-end guard caused 173/180 interventions. Training reset 103/177
rollouts before their declared physical horizon; larger AV cohorts got less
time per car. The instantaneous lane mask also allowed requests whose projected
rear gap disappeared before the three-second maneuver could finish.

## Implemented changes

1. **Future gap preparation.** Current local/map information scores the present
   gap, falling behind the observed rear vehicle, and advancing ahead of the
   observed front vehicle. Preparation horizons are 3/6/10 seconds. Candidate
   speeds use 17 evenly spaced values up to the effective legal limit, reachable
   with acceleration between -2 and +1.5 m/s2. The score includes projected
   gap deficits, remaining lane runway, speed loss and acceleration magnitude.
   Alternative slots have an unobserved outer neighbor: they guide longitudinal
   preparation only and never grant lateral permission. This is a kinematic
   estimate, not a native safety certificate or a learned prediction model.
2. **One merge-speed objective.** The selected preparation speed replaces the
   immediate front/rear interval target while preparing a required merge. The
   occupied-lane leader, legal limits, upcoming lower limits, ending-lane stop
   envelope, visibility and missing-data checks remain. No contradictory
   additional highway-speed target is applied during preparation. A blocked
   immediate gap no longer removes all longitudinal guidance. A genuinely
   unavailable rolling path can still produce a stopping target.
3. **Potential shaping.** Merging agents receive
   `2 * (0.99 * Phi(next) - Phi(current))`, where Phi is the bounded readiness
   score `1/(1+error)`. Error measures present/projected gap deficits for the
   selected slot plus speed mismatch. Target switches and missing evidence do
   not reset the potential; true terminals use zero and time-limit truncations
   retain the actual successor potential for bootstrapping. Non-merging agents
   receive no gap-shaping term. Tests verify discounted telescoping and that
   stationary readiness cannot collect a positive repeated shaping payment.
4. **Rolling quality bonus.** Replace the old +2 stability reward with +5 only
   when the existing stable-merge check passes AND the additional quality check
   passes. Whole-approach stopped time must be at most 1 s; the last 8 s before
   completion and 8 s afterward require at least 90% speed-target agreement
   within 2 m/s, at most 5% intervention decisions, peak absolute jerk at most
   5 m/s3, and no observed neighbor acceleration below -3 m/s2. This is a
   conservative rolling-quality bonus: necessary yielding still earns ordinary
   completion reward but may not earn this bonus. Neighbor braking is measured,
   not attributed causally to ego. Missing speed-target evidence cannot pass as
   a match. The ordinary +5 completed-merge reward remains.
5. **Full-maneuver mask.** Keep every inherited current-gap, connectivity,
   cooldown, committed-maneuver and runway restriction. Additionally check front
   and rear gaps at the start/end of a 3.1 s constant-speed projection with the
   reward's 0.3 s reaction / 1.5 s headway / 4.5 and 9 m/s2 braking assumptions.
   Linear projected gaps attain their minimum at an endpoint. This rejects the
   audited late closing-rear request. Unexpected acceleration/cut-ins remain
   SUMO's responsibility; the mask is not permission to bypass native guards.
   Mask flags change in the existing actor inputs; observation dimensions do not.
6. **Complete episode rollouts.** Reserve `horizon_steps * controlled_count`
   decisions before beginning an episode, collect to natural completion or its
   declared time limit, then update PPO. Refund unused reservations after
   arrivals. If the remaining decision budget cannot cover an entire episode,
   end learning rather than reset a partial rollout. The explicit wall deadline
   can still interrupt work; that failure/budget event is retained and charged.
   No simulator-state continuation is claimed after a process crash.
7. **More merge experience.** Retain all 12 original mixed task/population rows
   and interleave eight one-AV merge rows, yielding 20 rows, 12 of them merging.
   The environment retains 2-5 through lanes, 8/16/24 total vehicles, variable
   controlled populations and message-loss/no-message cases. This changes the
   curriculum as well as the objective; the run is not a single-factor ablation.

Implementation: [rewards](../traffic/highway_flow_rl_v04_rewards.py),
[environment](../environments/highway_flow_rl_v04_env.py),
[runner](../experiments/run_highway_flow_rl_v04.py),
[launch contract](../traffic/highway_flow_rl_v04_contract.py),
[tests](../tests/test_highway_flow_rl_v04.py).

## Resolved training configuration

Authoritative file: [highway-flow-rl-v04-long.json](../configs/training/highway-flow-rl-v04-long.json).

| Parameter | Value |
| --- | --- |
| Model | Shared feedforward MAPPO, existing flow117 architecture |
| Inputs / critic state | 117 local/message inputs / 2,832 training-only population state |
| Actions | Acceleration [-4.5,+2] m/s2 and right/stay/left intent, 10 Hz |
| Speed and native safety | 60 mph global ceiling plus local limits; speed mode 31, lane-change mode 512; longitudinal guards unchanged |
| Initialization | Frozen long-v03 actor and critic; fresh optimizer for changed reward/mask |
| Training seed | 9710; per-case seed schedule saved in config, validation seeds retained |
| Actor / critic learning rates | 1e-4 / 3e-4 |
| Epochs / minibatch | Up to 20 / 256 active-agent samples |
| PPO clipping / value clipping | 0.2 / 0.2 |
| Target KL / entropy / global gradient norm | 0.02 / 0.001 / 0.5 |
| Gamma / GAE lambda | 0.99 / 0.95, explicit and unchanged for attribution |
| Imitation / teacher loss | None |
| Reward coefficients | Progress 1; merge 5; gap shaping 2; speed error .30; Huber jerk .09; rolling quality 5; route detour 2; other native event/safety coefficients unchanged |
| Per-episode allocation | Full 80-160 s declared horizons, independent of AV count; naturally completed cars stop contributing |
| Learning limit | 4,000,000 additional decisions, eight-hour run wall cap, whichever ends first |
| Evaluation reserve | Last 15 minutes of the run reserved for final development evaluation |
| Automatic validation | Same six development cases initially, every 500,000 charged decisions, and finally |
| Candidate retention | Initial actor, periodic actor/full checkpoints, latest full checkpoint, best actor/full checkpoint and final actor |
| Best-candidate ranking | Safety events, failed case gates, completed routes, rolling-quality merges, pre-merge stop time, then worst per-car intervention fraction; lexicographic, lower is better |
| Held-out / promotion | Disabled; no automatic acceptance or deployment |
| Shared historical accounting | Existing v02 ledger preserved; round 3 has separate allocation; cumulative study ceiling 36,000 s and 4,330,000 decisions |

Twenty is a maximum epoch count. The KL safeguard can stop an update earlier.
All rollouts remain on-policy; no demonstration transitions are mixed into PPO.
The new budget does not erase previous failed work. Exact v04 resume preserves
optimizer/RNG and requires matching source/config hashes; interrupted native
episodes are discarded and charged, not reconstructed as successful data.

## Validation before launch

- **116 focused tests pass**, covering blocked-gap guidance, occupied-lane
  constraints, missing evidence, projected rear/front gaps, potential terminal
  handling, quality exclusions, full-horizon reservations, source/launch pins,
  checkpoint selection and inherited reward/runner/demand regressions.
- Saved v03 focal states: speed-target coverage rises from **205/549 to 545/549**.
  Four states still abstain for missing target or unresolved lateral evidence.
  This is an offline coverage result, not a claim of improved trajectories.
- First native preflight exposed a NumPy integer in the additional-mask-count
  report. Converted that count to ordinary integers; retained the failed run.
- Final zero-update native preflight passes API/shapes, native guards, finite
  gradients, actor export and reward/mask version checks. No optimizer steps.
- Repeated seeded smoke runs have identical decompressed action-journal SHA-256.
- A full I-110 development episode with the unchanged v03 weights completes its
  primary horizon: **2,961 decisions, zero collisions/teleports, one stable merge**.
  The main car still stops for **11.7 s** and fails rolling quality. This verifies
  execution, not a qualified controller. Do not promote it.

[Compact preparation evidence](evidence/highway-merge-rewards-v04/preparation.json).
Raw preparation outputs remain under `outputs/highway-flow-rl/v04/`, including
the failed preflight, offline rows, repeatability traces and full-merge baseline.
No new sensor/Isaac run or perception training is involved.

## Unattended launch and retained results

From the checkout using the traffic Python:

```text
python scripts/run_highway_flow_rl_v04_job.py --preflight outputs/highway-flow-rl/v04/preflight-05/preflight.json --output outputs/highway-flow-rl/v04/long-round-03
```

The [job launcher](../scripts/run_highway_flow_rl_v04_job.py) checks that the
published branch's runtime sources/config match the launch workspace, writes
PID/status/commit/command provenance, and waits for process exit without polling
metrics or invoking an assistant. The runner saves metrics and performs the
scheduled evaluations itself. Windows launches use a hidden process window.

Expected artifacts under `outputs/highway-flow-rl/v04/long-round-03/`:
`last.pt`, `best.pt`, `best-policy.pt`, final `policy.pt`, periodic `candidates/`,
per-update JSON, action journals, matched initial/final validation, configuration,
source snapshot, manifest, summary or failure JSON. Best may remain the initial
actor if learning degrades; it does not mean accepted. Logs and terminal status:
`logs/highway-flow-rl-v04-long-round-03.*`.

Branch: `highway-merge-rewards-v04`, based on the v03 audit checkpoint. Historical
code/checkpoints remain untouched. Google engineering-guide synchronization is
pending; its generated Markdown mirror was not edited. No improved policy
performance is claimed until terminal evaluation results are reviewed.
