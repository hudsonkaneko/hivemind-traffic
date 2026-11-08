# MAPPO v03: explicit communication and traffic-wave setup

October 26, 2026. Implemented and no-learning preflight verified. **Training has not started.** This milestone covers a four-car SUMO acceleration policy and a single braking-pulse fixture. Merging, closer headways, real radio transport, physical integration and learned communication benefits remain unverified.

## Communication and policy contract

Four controlled cars broadcast immutable JSON state snapshots through a seeded delivery queue. The ordinary background leader does not transmit. Payloads carry sender, episode, sequence, timestamp, longitudinal position, lane, speed, measured acceleration and the preceding selected acceleration. Intent is historical selected intent, not future simulator state or a maneuver commitment. Range is evaluated at send time; receiver ranking uses transmitted positions. Local, no-message and full-fleet modes retain the same observation shape.

The reviewed pilot uses 10 Hz broadcasts, 150 m range, six receiver slots, uniform 0–300 ms packet delay, 10% loss and 500 ms expiration. Full mode bypasses range only; ideal preflight also sets zero delay/loss. Expired, duplicate, out-of-order and wrong-episode messages are rejected. Missing slots are masked. Outage injection is supported and preflight includes a 12–15 s outage with20% loss. JSON payload-byte counters exclude radio headers; there is no radio congestion/bandwidth model. At four cars, at most three peers are received despite six available slots.

Actor input is69: the existing27 local features plus6x7 message fields: relative position/150m, lane difference/2, relative speed/26.8224m/s, measured acceleration/4.5, previous intended acceleration/4.5, age/0.5s, and validity. Features are clipped to their declared bounds. Simulator-derived local observations remain a sensing abstraction, not a trained camera/LiDAR stack.

A shared6→16→16 Tanh encoder processes each valid message, mean-pools embeddings, and appends neighbor count/6. The resulting44 features feed the128×128 shared actor. The centralized critic consumes280 joint observation/alive features plus four identity features, through256×256 hidden layers. Deployment exports only the local actor. Six-slot order does not affect the actor; masked message values have no effect.

Action remains one normalized acceleration mapped to[-4.5,+2.0]m/s². Speed cap26.8224m/s (60mph), cruise25m/s (~55.9mph), minimum gap2.5m and time headway1.5s are unchanged. SUMO and local safety filters remain active and all overrides remain counted.

## Wave fixture and reward

Four controlled cars start in lane0, at positions65/120/175/230m and20m/s. The ordinary leader starts at300m. A seeded pulse begins uniformly between10 and15s and requests−2m/s² for4s; SUMO retains authority to apply its safety constraints. Episodes last60s at0.1s steps. Background demand continues in three lanes, but wave rewards and detector throughput use only the affected lane0.

Team reward is the mean of per-car weighted components:

| Component | Weight | Meaning |
| --- | ---: | --- |
| progress | +1.0 | Ego progress normalized by cruise speed and step, capped at1 |
| traffic_progress | +1.0 | Mean lane0 speed/cruise, capped at1 |
| headway | −0.5 | Existing desired-gap deficit |
| jerk | −0.02 | Existing acceleration-change penalty |
| collision | −10.0 | Controlled-car collision indicator; any collision terminates episode |
| intervention | −1.0 | Local or SUMO action override |
| cruise_excess | −2.0 | Normalized excess above25m/s |
| wave_fluctuation | −0.5 | Mean lane0 absolute acceleration/4.5, capped at1 |

The fluctuation reward is a smoothness proxy, not measured wave amplification. Evaluation records pooled follower speed variance over the full episode (all vehicle-time samples in lane0 behind the pulse leader), lane0 crossings at1000m, aggregate lane0 travel, safety and interventions. Pooled variance includes initial transients and changing membership; this is not a string-stability certificate. Throughput from a60s interval is reported as an hourly equivalent, not a sustained capacity claim.

## Final pilot configuration

`experiments/configs/sumo_mappo_v3.json` is the communication pilot. `sumo_mappo_v3_no_comm.json` is a separately runnable matched baseline: identical scenario, reward, architecture, budget and seeds, but no messages. Neither has been trained. The old v02 policy is preserved and is not a valid same-scenario no-communication baseline.

MAPPO: CPU,4 workers,256 rollout steps/worker,128×128 actor,256×256 critic, Adam learning rates0.0003, gamma0.99, GAE0.95, PPO/value clipping0.2, entropy0.001, value coefficient0.5, gradient norm0.5,4 epochs, minibatch256, KL target0.02. Budget is1,000,000 agent decisions or7,200s total including evaluation, whichever comes first. Runtime seed6103; training episode seeds600000–699999; validation710001–710010; held-out810001–810050; preflight920001–920002. No resume from the incompatible27-input v02 actor.

Basic driving acceptance remains zero collisions/overspeed, all nominal completions, mean progress≥600m and overrides≤5%. A separate paired helper requires matching episode seeds/durations and nonzero baseline variance/flow. Wave targets are≥15% lower pooled variance with≤2% throughput loss versus a **separately trained** no-communication policy. Basic driving acceptance alone never sets `wave_benefit_verified`; current runner explicitly reports false until that comparison is performed. A single pilot is not a multi-seed generalization study. Larger studies and extra baseline training require their own reviewed launch.

```powershell
.\.venv-traffic\Scripts\python.exe experiments/run_sumo_mappo.py --config experiments/configs/sumo_mappo_v3.json --mode check
.\.venv-traffic\Scripts\python.exe experiments/run_sumo_mappo.py --config experiments/configs/sumo_mappo_v3.json --mode preflight
```

The check command displays the exact approval digest. Training requires explicit train mode plus that digest; this setup did not invoke it. After approval, execution remains headless and file-logged; wait for completion/failure without intermediate agent monitoring. New policies save to `outputs/sumo_mappo/v03/train-<timestamp>-<id>/policy.pt`, with best/latest full checkpoints alongside.

## Verification and limitations

124 targeted tests passed, including original regressions, channel delay/loss/expiration, same-shaped no-message inputs, PettingZoo parallel API/seed tests, masked/permutation-invariant actor behavior, PPO likelihood replay, actor-only export, dimensional validation and paired metrics. TorchScript emits deprecation warnings but export readback matches.

Final native preflight `preflight-20261026T230647Z-60ff04c7` passed in35.843s, with0 optimizer steps, unchanged weights/source,512 collected agent decisions, likelihood error4.77e−7 and exact scripted repeat. Five controller types ran two episodes each, plus repeat and four channel-condition checks. Random and untrained actors intentionally fail driving acceptance; the shield contains them. Scripted/cooperative references both achieved1489.065m average progress,3.4375% overrides, zero collisions/overspeed, pooled variance4.6960 and990 vehicles/hour equivalent. Their results are identical: **this preflight does not demonstrate a communication benefit**. Native traffic variance was5.2400 with the same throughput, but that is not the required learned matched comparison.

The earlier preflight `preflight-20261026T230536Z-d277cd48` is preserved: it exposed excessive interventions in the old ceiling-seeking scripted reference. The v03 reference now targets25m/s and uses peer intent only to anticipate braking. v01/v02 reference behavior remains unchanged.

Three subagents implemented/reviewed the channel, scenario and metrics. Review corrected affected-lane measurement dilution and rejected configuration/model width mismatches. Evidence: [compact measured package](evidence/sumo-mappo-v3-setup.json). Source hashes and full journals are in the ignored local run package. No learned policy, GUI recording, real-world safety or Omniverse result is claimed by this setup.
