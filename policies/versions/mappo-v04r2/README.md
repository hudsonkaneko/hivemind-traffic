# MAPPO v04r2

**Accepted within its original straight three-lane SUMO fixture.** October 29, 2026.

[Download policy.pt](policy.pt?raw=true) · [Contract](contract.json) ·
[Training parameters](training-config.json) · [Held-out results](evaluation.json) ·
[Hashes and provenance](manifest.json)

| Detail | Value |
| --- | --- |
| Policy type | Imitation-assisted shared MAPPO; deterministic TorchScript actor |
| Inputs | 81 floats: 33 local fields + six 8-field delivered V2V messages; exact ordered fields in contract |
| Outputs | Normalized acceleration [−1,1], lane intent −1/right, 0/stay, +1/left |
| Acceleration conversion | `-4.5 + (action[0] + 1) * 3.25` m/s²; apply the matching environment's guards |
| Network | Message encoder + 128/128 actor; 256/256 centralized critic used only in training |
| Control | 10 Hz; 60 mph ceiling; SUMO owns lane execution |
| Training | Four CPU workers; 1M total transitions (48k demonstrations + 952k PPO); selected update 175 at 764,800 transitions |
| Warm start | 60 BC epochs, batch 512, LR 0.0003 |
| PPO | Actor LR 0.00003; critic LR 0.0003; four epochs, batch 256, clip 0.2; gamma 0.99, GAE 0.95 |
| Communications | 150 m range, 10 Hz, 0–0.3 s latency, 10% packet loss, 0.5 s expiry |

**Intended behavior:** follow traffic, pass a slow leader and maintain lane stability
on the original straight three-lane scenario.

**Measured:** passed the original acceptance checks over 50 held-out episodes;
74 total passes, at least one per episode; 3.175% aggregate overrides; maximum
two lane changes/car/minute; zero collisions, overspeed or required-gap deficits.
Minimum observed headway was 1.638 s. Its configured 0.8 s cooperative target is
not an achieved short-following-distance claim.

**Changes from v04r1:** stronger lane-change penalty (−0.5), reduced lane-label
weight during PPO (2), and revised imitation-assisted training for lane stability.
Use this frozen selected actor; the run's final update 233 regressed and is not
the accepted checkpoint. No merging, variable-road adaptation, camera/LiDAR,
physical steering or causal communication-benefit claim is established.

The old aggregate acceptance gate differs from Stage 1's per-case gate; results
are not interchangeable or a direct benchmark comparison. Its observation
contract is incompatible with the 93-field highway actor and other 81-field
actors with different feature meanings.

Source commit: `bd59d2f008c4c59615276c759377562fc339e826`.
Policy SHA-256: `4cf4898df997a444624552515ea987e7e994b1ca40bb002cf91f0647be8a7245`.

Load float32 tensors shaped `[batch,81]`; output is `[batch,2]`. The safety
filter, critic and optimizer are not in this inference file. Full details:
[original training record](../../../documentation/sumo-mappo-v4-stability.md).
