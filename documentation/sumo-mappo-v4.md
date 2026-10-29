# MAPPO v04: lane changes, passing and cooperative following

October 29, 2026. Setup and validation milestone; no v04 training authorized or
started by this implementation. The accepted v02 actor remains available and
unchanged. v04 has a different observation/action contract and starts fresh.

## Scope

Four shared-policy cars drive a straight, three-lane SUMO highway with ordinary
traffic and a slower ordinary leader. SUMO keeps cars centered within their
selected lanes. The policy chooses acceleration and a lane-change request; this
does not teach continuous steering or validate physical Isaac lane tracking.
The first pilot uses seeded mixed traffic, not a claimed staged-curriculum
scheduler. Physical transfer, sensor-driven control, intersections and highway
on-ramp merging are outside this setup.

## Proposed pilot configuration

The machine-readable files are
[`sumo_mappo_v4.json`](../experiments/configs/sumo_mappo_v4.json) and the matched
[`sumo_mappo_v4_no_comm.json`](../experiments/configs/sumo_mappo_v4_no_comm.json).
Both retain identical traffic, architecture, rewards, budget and scenario seeds;
the latter disables message delivery. Neither is a trained baseline yet.

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
made no passes and had 73.23% interventions. No trained v04 checkpoint exists.

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

This command has **not** been run. Policies will save to
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
