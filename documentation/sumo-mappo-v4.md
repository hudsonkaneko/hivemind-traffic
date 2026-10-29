# MAPPO v04: lane changes, passing and cooperative following

October 29, 2026. Setup validated and the approved v04 pilot completed. The pilot
failed learned-policy acceptance; see the results below. The accepted v02 actor
remains available and unchanged. v04 has a different observation/action contract.

Follow-up: the [v04r1 refinement](sumo-mappo-v4-refinement.md) completed after user
authorization. Its neural actor passed slower cars in all 50 new test episodes
with 4.9583% aggregate overrides, but one episode exceeded the lane-change
frequency limit. Full v04 acceptance remains open. The original pilot below
is preserved as historical evidence.

## Scope

Four shared-policy cars drive a straight, three-lane SUMO highway with ordinary
traffic and a slower ordinary leader. SUMO keeps cars centered within their
selected lanes. The policy chooses acceleration and a lane-change request; this
does not teach continuous steering or validate physical Isaac lane tracking.
The first pilot uses seeded mixed traffic, not a claimed staged-curriculum
scheduler. Physical transfer, sensor-driven control, intersections and highway
on-ramp merging are outside this setup.

## Approved pilot configuration

The machine-readable files are
[`sumo_mappo_v4.json`](../experiments/configs/sumo_mappo_v4.json) and the matched
[`sumo_mappo_v4_no_comm.json`](../experiments/configs/sumo_mappo_v4_no_comm.json).
Both retain identical traffic, architecture, rewards, budget and scenario seeds;
the latter disables message delivery and remains untrained. The communication
configuration completed one pilot; no communication-benefit comparison exists.

| Setting | Value |
| --- | --- |
| Algorithm | Shared actor MAPPO; centralized critic during training only |
| Controlled fleet / road | 4 cars / 3 lanes / 5,000 m |
| Control interval / horizon | 0.1 s / 600 steps (60 s) |
| Speed ceiling / cruise | 26.8224 m/s (60 mph) / 26 m/s (58.16 mph) |
| Acceleration limits | −4.5 to +2.0 m/s² |
| Actions | Continuous normalized acceleration plus categorical right/stay/left |
| Actor observations | 81: 33 local fields + six eight-field delivered-message slots |
| Critic input | 328 joint observations/alive fields + 4 agent identity fields |
| Actor / critic hidden layers | 128×128 / 256×256 |
| Cooperative / ordinary target headway | 0.8 s / 1.5 s, subject to braking bound |
| Minimum standstill gap | 2.5 m |
| Lane-change cooldown / minimum TTC | 5 s / 3 s |
| Communication | 10 Hz, 150 m, six slots, 0–300 ms latency, 10% loss |
| Message expiration / short-gap freshness | 500 ms / 200 ms |
| Workers / device / threads per worker | 4 / CPU / 1 |
| Rollout steps / minibatch / epochs | 256 per worker / 256 / 4 |
| Actor and critic learning rate | 0.0003 |
| Discount / GAE | 0.99 / 0.95 |
| PPO and value clip / gradient norm | 0.2 / 0.5 |
| Entropy / value coefficient / KL target | 0.01 / 0.5 / 0.02 |
| Budget | 1,000,000 agent transitions or 7,200 wall seconds, including evaluation |
| Training RNG / episode seeds | 6104 / 1000000–1099999 |
| Validation / held-out / preflight seeds | 1110001–1110010 / 1210001–1210050 / 1310001–1310002 |

## Safety and interpretation of short following

Short-headway eligibility requires a fresh delivered message matching the actual
sensed leader. Ordinary, missing or stale peers use the ordinary target. Gap
selection also accounts for a 0.3 s response delay, follower braking of 4.5 m/s²
and leader braking of 9 m/s². For follower speed v and leader speed u:

`required_gap = max(2.5 + target_headway*v, 2.5 + 0.3*v + v²/(2*4.5) − u²/(2*9))`.

At equal speeds of 60 mph the braking term is approximately 50.5 m. Therefore
0.8 s is a desired cooperative target, not a promise of a 0.8 s achieved gap at
highway speed. No braking assumption is weakened merely to obtain closer cars.
The assumptions are simulator constraints, not a real-vehicle safety certificate.

Lane changes require front and rear clearances, TTC, lane existence and cooldown
checks. A joint-action shield must also resolve simultaneous competing requests.
SUMO's native speed and lane-change safety remain enabled. Requested, locally
filtered and simulator-applied maneuvers are distinguished in telemetry.
The documented native mode 512 retains collision and safety-gap enforcement:
[SUMO TraCI vehicle-state API](https://sumo.dlr.de/docs/TraCI/Change_Vehicle_State.html).

## Observations and rewards

The existing 27 local state features describe speed, acceleration, current lane,
route progress, previous acceleration, nearest front/rear traffic in own and
adjacent lanes, lane existence and elapsed time. Six new local fields describe
leader cooperation, required gap, cooldown and the three lane-action masks.
Each delivered-message slot includes position/lane/speed differences, acceleration,
previous selected acceleration, age, previous lane intent and validity. Intent
is a historical request, not a guaranteed commitment or future simulator state.
These are structured sensing abstractions; cameras and LiDAR are not actor inputs.

| Reward component | Weight |
| --- | ---: |
| progress | +1.0 |
| headway deficit | −0.5 |
| jerk | −0.02 |
| collision | −10.0 |
| safety intervention | −1.0 |
| cruise excess | −2.0 |
| verified slower-vehicle pass | +1.0 |
| lane change | −0.05 |
| safe cooperative tracking | +0.05 |
| rejected lane request | −0.25 |

A pass reward must require actual traversal past a previously encountered slower
leader and must not be paid repeatedly for weaving around the same car. Reward
and return alone do not establish driving or cooperation competence.

## Verification and launch

145 targeted tests passed, including v01-v03 regressions, PettingZoo parallel API
and seeded reset tests, native SUMO behavior, hybrid PPO likelihood replay,
masked/permutation-invariant messages, actor export and independent safety review.
Native tests read back SUMO tau changing from 0.8 to 1.5 seconds on outage;
this measures the applied parameter, not the resulting time gap.

Final preflight **preflight-20261029T221040Z-a36f0ca0** completed in 39.453 seconds
with zero optimizer updates, unchanged sources/weights, exact scripted repeat
and matching actor-only export. Its four-worker rollout collected 512 agent
transitions; maximum stored/recomputed likelihood error was 9.54e−7.
The two 60-second cooperative-scripted episodes achieved:

- 3 actual slower-car passes, 7 applied lane changes, 1 rejected lane request;
- zero collisions, overspeed or required-gap deficit steps;
- 1 safety override in 4,800 agent decisions (0.0208%);
- mean per-car progress 1,501.02 m and minimum observed headway 1.728 s;
- 1,694 applied cooperative-headway commands and 3,106 applied ordinary commands.

Mean headway while the post-step leader was eligible for cooperation was 2.033 s.
The no-message, ideal full-message, nominal delay and outage references all met
the driving checks; no-message runs never applied cooperative headway. Their
near-identical progress does **not** prove a communication benefit. A separately
trained matched no-communication policy remains required for that claim.
Random and untrained policies failed driving acceptance as expected; the latter
made no passes and had 73.23% interventions. These are pre-training results.

Review fixed an immediate cooldown-mask error, destination-lane acceleration and
headway filtering, and confusion between the applied headway command and next-step
eligibility. An early scripted probe with 56.4% overrides is retained locally;
the reference now anticipates the unchanged braking bound with a 5 m buffer.
This buffer is a scripted-reference choice, not a weakened safety rule. TorchScript
deprecation warnings are retained; export/readback still passed.

Acceptance requires zero collisions and overspeed, all nominal episode endings,
mean progress at least 600 m, interventions at most 5%, at least one actual pass
in **every** evaluation episode, and no individual car exceeding six lane changes
per minute. These gates do not by themselves establish short-gap or coordination
benefits. Validation selects checkpoints; the separate test seeds are held out.

Evidence: [compact result/provenance](evidence/sumo-mappo-v4-setup.json). Full
per-episode results, action journals, resolved settings, source snapshots and
interface-test exports remain in the unique local run directory under
`outputs/sumo_mappo/v04/`; logs are under `logs/sumo_mappo/`.

```powershell
.\.venv-traffic\Scripts\python.exe experiments/run_sumo_mappo.py --config experiments/configs/sumo_mappo_v4.json --mode check
.\.venv-traffic\Scripts\python.exe experiments/run_sumo_mappo.py --config experiments/configs/sumo_mappo_v4.json --mode preflight
```

Final configuration SHA256:
`cc1800e164b7eef4d75958d718826d656ab129a9d87d51952283ad8f68720107`.
After review and approval, the training command is:

```powershell
.\.venv-traffic\Scripts\python.exe experiments/run_sumo_mappo.py --config experiments/configs/sumo_mappo_v4.json --mode train --approved-config-sha256 cc1800e164b7eef4d75958d718826d656ab129a9d87d51952283ad8f68720107
```

The approved command completed on October 29. Policies save to
`outputs/sumo_mappo/v04/train-<timestamp>-<id>/policy.pt`, with full resumable
`checkpoints/latest.pt` and `checkpoints/best.pt`. Existing v02/v03 checkpoints
are incompatible. A matching v04 checkpoint can resume into a fresh run package
using `--resume`; agent-transition count carries forward. The runner's wall-time
cap applies per invocation, so any recovery must also account for elapsed time
against the approved study budget instead of silently granting another two hours.
Recoverable errors may be corrected and resumed within approved scope; changed
observations, actions, safety limits or budget require a new reviewed configuration.
Execution remains headless and file-logged, with results reviewed on completion
or failure rather than continuous agent monitoring.

## Completed pilot: not accepted

Run **train-20261029T221921Z-3810ae99** completed without runtime errors in
1,284.421 seconds (21 min 24 s): 1,000,000 agent transitions, 245 updates and
15,510 optimizer steps. No recovery or resume was needed. Training ran without
agent monitoring; review began after process completion. The approved transition
budget is exhausted. Configuration and safety constraints were not changed.

| Held-out result, 50 seeds each | Learned exported actor | Scripted reference |
| --- | ---: | ---: |
| Collisions / overspeed steps | 0 / 0 | 0 / 0 |
| Nominal episode endings | 50 / 50 | 50 / 50 |
| Safety intervention fraction | 7.7275% | 0.2992% |
| Applied lane changes / actual passes | 0 / 0 | 167 / 75 |
| Mean per-car progress | 1,099.03 m | 1,468.69 m |
| Minimum measured headway | 1.601 s | 1.629 s |
| Mean headway while cooperative-eligible | 1.886 s | 2.105 s |
| Accepted | **No** | Yes |

The exported actor exceeds the 5% intervention limit and never passes. It also
has 377 measured required-gap-deficit steps; zero collisions with active safety
filters does not establish independent safety or achieved 0.8 s following.

The configured validation ranking selected **update 175 (716,800 transitions)**
for `policy.pt` and `checkpoints/best.pt`. Later checkpoints learned passing but
scored worse overall. These are ten-seed validation results, not additional
held-out tests:

| Update | Applied lane changes | Passes | Overrides | Remaining failure |
| --- | ---: | ---: | ---: | --- |
| 175, exported | 0 | 0 | 7.77% | No passes; overrides |
| 200 | 11 | 11 | 30.90% | Some episodes without a pass; overrides |
| 225 | 50 | 22 | 26.63% | Overrides; driving targets otherwise met |
| 245, latest | 106 | 12 | 21.40% | Overrides; up to 10 changes/car/min, limit 6 |

All evaluated checkpoints failed full acceptance. Independent read-only review
found finite update metrics and no evident runtime or action-path bug requiring
automatic recovery. PPO includes categorical lane likelihood/entropy and exports
deterministic lane selection. The observed limitation is optimization and
checkpoint-selection tradeoffs. Revised reward shaping, curriculum or selection
would be a new configuration; additional training needs a reviewed budget.
The completed run is preserved rather than silently extended or relabeled.

Retained locally under `outputs/sumo_mappo/v04/train-20261029T221921Z-3810ae99/`:
`policy.pt`, full `checkpoints/best.pt` and `checkpoints/latest.pt`, per-seed
evaluations, update/episode logs, action journals, source snapshots, exact config,
runtime manifest and artifact hashes. Exported actor SHA256:
`96af779e9e93f0d6c7df643d753ac6f63516ac737e2cf5997ac6cb9cb0dd5f91`.
Source commit: `b0b63f7` (full identity and source hashes in evidence).

GitHub review evidence: [summary and provenance](evidence/sumo-mappo-v4-training/summary.json),
[all controller per-seed metrics](evidence/sumo-mappo-v4-training/held-out-metrics.csv),
and [exact learned evaluation](evidence/sumo-mappo-v4-training/held-out-learned-evaluation.json).
Bulk journals and model weights remain local. The runner's `passed: true` records
successful execution, while `learned_acceptance: false` records the failed policy
gate. No v04 checkpoint supersedes accepted v02; no 3D/physical validation or
communication-benefit claim is made.
