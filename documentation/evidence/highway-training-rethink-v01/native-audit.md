# Read-only motion/control audit — November 6, 2026

The scripted v4 is smoother in the displayed I-110 case, but it is not a fleet-progress improvement. It deliberately approaches the first junction slowly, holds a slow merge slot afterward, and sends another mainline car into the ending auxiliary lane to pass. The latter car then nearly stops while trying to return. Earlier focal merge completion alone concealed this tradeoff.

Scope: existing files, saved 0.1 s trajectories and command journals, plus official SUMO sources. **No simulator runs, source edits, optimizer steps or training.** Read repository instructions, project-skills inventory, SUMO/reproducibility skills, scene conventions and living roadmaps. Runtime versions below come from retained manifests, not a new runtime launch.

## Exact compared evidence

- `outputs/highway-smooth-control/v02/matrix-02/i110-merge/smooth`: active scripted reference v4, SHA256 `ed3780f990e0751bb5e8f23b491c918e8a84162402a0fb352a5b71b966b8fc6c`.
- `outputs/highway-smooth-control/v02/matrix-01/i110-merge/{frozen-v05,teacher,native}`: fixed comparison runs. Frozen actor SHA256 `fd639c39f0c514256a458b2b85d35baba242125f44dcc3d9542d767eb2dd46ec`, originating in `outputs/highway-environment/v05/train-20261106T024828Z-90b25880/policy.pt`.
- All share seed **6800019**, eight controlled slots, 16 initial vehicles, difficulty 2, sustained native arrivals, local communication without packet loss, 80 s measurement horizon. SUMO/TraCI **1.27.1**, Python 3.11.9. Fifty-five total requested vehicles; original ramp merge target is only `agent_0`.
- Corrected network SHA256 `ae829282ad98cfaeeba806fba30e44536361ccabf4d0861708c5638fff512836`. Existing false-foe network is not used here.
- [CSV](merge-speed-traces.csv): 3,453 actual samples for `agent_0` and `agent_4`, native/teacher/frozen-v05/v4. SHA256 `b46731a3ae9588366026b5ce7af1b2c998000e43db9a04a7012c838585d490a6`. No fabricated arrival samples or interpolation.
- [Four peak fits](native-acc-peak-fits.json): previous-frame inputs, command/native outputs, raw source paths/checksums and offline equations. SHA256 `8be0353e7f78eb809558455404e764eb55230f7b3eb01e0cf230e2566d3f66e8`.

## What the displayed main car actually does

Times are native timestamps from reset's first observed state at 0.1 s. Distances are SUMO traveled distance since departure, not distance since merge. Speeds are m/s.

| Main-car measurement | Frozen MAPPO | Scripted v4 |
| --- | ---: | ---: |
| Initial speed | 8.6121 | 8.6121 |
| Ramp maximum | 12.1556 | 8.6121 |
| Ramp minimum | 6.3056 at 3.6 s | 6.0153 at 5.0 s |
| Enter first internal junction / speed | 4.3 s / 7.4996 | 5.6 s / 6.0265 |
| Enter auxiliary lane / speed | 4.5 s / 7.8372 | 5.9 s / 6.1516 |
| Start commanded lateral merge / speed | 32.7 s / 0 | 15.0 s / 11.5099 |
| Settled lateral merge / speed | 35.9 s / 2.7502 | 20.0 s / 15.6497 |
| Stable completion | 43.9 s | 28.0 s |
| Distance at 10 / 20 / 30 s | 103.560 / 275.089 / 407.304 m | 69.865 / 181.348 / 352.482 m |
| Observed speed <9 m/s, first 30 s | 4.6 sample-seconds | 13.0 sample-seconds |
| Observed speed <9 m/s, full observed trip | 14.1 sample-seconds | 13.0 sample-seconds |
| Longest observed speed <0.1 m/s span | 4.2 s, 30.1–34.3 | 0 |
| Actual arrival | 49.2 s | 42.9 s |

Sample-seconds mean qualifying 0.1 s samples multiplied by 0.1, not reconstructed dwell durations. The v4 car merges earlier in a different spatial location: traveled distance 181.348 m at settled merge, versus 409.652 m for the frozen actor. It remains behind the frozen car in road progress through 30 s, then benefits from avoiding the frozen car's later stop.

## Causal chain, with evidence boundaries

**1. This test is not a 60 mph driving test.** `scripts/prepare_highway_smooth_control.py:40` selects an 18 m/s ceiling (40.26 mph). `environments/highway_population_env.py:189` applies `min(original lane limit, episode ceiling)` to every native lane before the first departure. The ramp remains **13.41 m/s** (30 mph); mainline and the ramp internal connector become **18 m/s**. The internal connector's source limit is 20.12 m/s, so no lower connector speed limit explains the approximately 6 m/s junction approach.

**2. The first entry is modeled as a minor-priority junction, distinct from the later lateral merge.** In `network-v02/network.net.xml:157`, `mlk_on_0` enters `sb2_0` via `:n2_5_0`, state `m`. Junction `n2` at line 126 has six links: only auxiliary continuation `sb1_0 -> sb2_0` is a real foe. Ramp request index 5 is `response="000001" foes="000001"`; the four through links have no ramp foe. The previous false-conflict correction retained this true auxiliary yield. The 296.89 m ramp feeds a 2 m connector, then 173.15 m `sb2_0`, then 193 m `sb3_0`, which has no continuation to `sb4`. No vehicle-class permission or authored signal/stop is blocking these links.

No connection visibility is authored. SUMO's default **4.5 m** is the ego approach distance at which conflicting approaches are assumed fully visible. It is not measured camera/LiDAR range or a verified physical occluder. Minor links prepare to brake until that boundary even when conflict traffic is absent. See [official approach rules](https://sumo.dlr.de/docs/Simulation/Intersections.html#speed_when_approaching_the_intersection), [connection visibility definition](https://sumo.dlr.de/docs/Networks/PlainXML.html#lane-to-lane_connectivity), and [pinned 1.27.1 loader](https://github.com/eclipse-sumo/sumo/blob/v1_27_1/src/netload/NLHandler.cpp#L1382).

The v4 focal has safe projected front/rear gaps for **all 55 recorded active-entry decisions**, and no car occupies `sb1_0` during its first six seconds. Its recorded terminal target is nevertheless **6.06714 m/s**. `traffic/highway_smooth_reference.py:177` explicitly anticipates the inherited minor-entry stop envelope; the finite-distance term at line 191 commands roughly −0.55 m/s² from the start. Thus this slowdown is a real programmed/model response, not a rendering artifact or a currently necessary yield to an occupied foe. Whether the short visibility and the upstream auxiliary connection represent the intended highway is **unverified**. The source builder explicitly labels its ports, stripe endpoint and widths provisional (`scripts/build_i110_sumo.py:139`, `:215`). Raising visibility or assigning right of way is not a justified fix without choosing and validating that scenario assumption.

**3. After entry, the reference intentionally remains slow to let a closing car pass.** At 8–11 s the focal travels at **7.1846 m/s**, requests effectively zero acceleration, and logs active `merge_slot_yield`. At 10 s the target-lane-following speed is **17.564 m/s**, but `traffic/highway_smooth_reference.py:154` sets the slot target to `min(ego_speed, rear_speed−1)`, so it stays at 7.1846. This is a heuristic yield choice without a retained chosen gap, future gap prediction or progress objective. Some yielding can be necessary; the current record does not prove that this particular speed hold is the best safe plan. It directly explains the slower-looking first half of the clip.

**4. A second car chooses a self-defeating auxiliary-lane pass.** V4 `agent_4` starts on through lane `sb1_1`, requests right at **17.6 s**, and settles in the auxiliary lane at **20.7 s**. At the decision, its own leader `main_9` is only 0.228 m/s slower, gap31.536 m; the teacher's local target is16.399 m/s. The empty adjacent-front lane scores17.9, barely exceeding the **+1.5 m/s** passing threshold. The left alternative is rear-gap blocked. `traffic/highway_stage1_teacher.py:137` permits this right pass because it tests local speed advantage, not destination-lane route usefulness. `highway_environment_v05.py:330` checks enough runway to finish one lane change, not enough runway to pass and return. The five-second cooldown after completion (`i110_merge_env.py:223`) adds a return delay. The return starts only33.6 s and completes36.7 s; speed bottoms at **1.456 m/s** and arrival is50.0 s, versus frozen42.3 s. The frozen actor also enters the auxiliary lane, but much earlier (0.1–3.2 s) and returns8.3–12.7 s. This is not a new lane choice uniquely introduced by v4; changed timing makes its consequence worse.

**5. The ending-lane profile can brake before its speed bound is active.** Every `requires_merge` state contributes the profile derivative plus tracking error to an unconditional minimum (`traffic/highway_smooth_reference.py:158`, `:206`). For `agent_4` at26.0 s, the previous observed end distance is97.317 m and stop-profile target **19.466 m/s**, above the17.9 legal target. Its derivative is−1.642 m/s², and the total profile term is already **−0.249 m/s²**, becoming the chosen command. This accelerates the decline toward the eventual lane end. Such anticipatory braking may be needed if no merge slot exists, but the implementation applies it before a route-aware choice of whether to remain, merge, or pass. It should be tested as a planning/activation issue, not blindly removed. For the focal's first approach, this term is **not** the cause: its remaining-distance field is clipped at240 m, and its ending-profile acceleration stays positive; the entry term and slot-yield term dominate instead.

## Whole-fleet check

| Measured over full 80 s | Frozen | v4 |
| --- | ---: | ---: |
| Completed/requested | 33/55 | 33/55 |
| All-request boundary-time mean | 34.2138 s | 34.2374 s |
| Controlled-trip boundary-time mean | 39.8875 s | 39.8875 s |
| Controlled sampled mean speed | 16.3582 m/s | 16.3661 m/s |
| Background sampled mean speed | 16.1949 m/s | 16.1785 m/s |
| Controlled minimum speed | 0 | 1.4560 m/s |
| Controlled samples below5 m/s | 82/3,183 | 47/3,183 |
| Intervention fraction | 557/3,183 =17.4992% | 0/3,183 |

The focal gain is canceled by other-car delay. V4 does remove sustained stops in this fixed case, but zero overrides and low jerk do not establish more effective driving. Original teacher/native focal arrival times are37.2/37.3 s, faster than both. Native is context, not a learned-policy result.

Background ramp minima are identical across **all four** controllers: `inflow_8`2.943, `inflow_17`2.575, `inflow_26`3.226 m/s. None stops. These shared late-ramp slowdowns are separate from focal policy quality. Pooled lane samples: frozen/v4 ramp means11.089/10.926, auxiliary12.320/11.112, through16.605/16.606 m/s. These are occupancy-weighted descriptions, not paired causal estimates; trajectories spend different times in each phase. Original-ramp-only merge acceptance misses mainline `agent_4`'s induced remerge; full-route/fleet measures must stay primary.

## Four remaining jerk failures: new offline explanation

The saved previous-frame gap/speed inputs plus the existing `acc_response_ceiling` reproduce **all four** measured peak accelerations to maximum absolute error **1.49e−14 m/s²**. This is an analytical fit to retained measurements, not native-emitted causal labels; the journal remains `native_realization_unattributed`.

| Fixture / next time | Trigger in previous measured state | Predicted = measured acceleration |
| --- | --- | ---: |
| Cruise31.6 s | Native `main_12` starts left occupancy; ego's leader switches to it, gap44.127 m, speed14.416 vs ego17.900 | −2.195869 m/s² |
| Following55.0 s | Controlled `agent_2` starts left occupancy, gap69.101 m, speed7.178 vs ego17.900 | −4.500000 |
| Passing44.8 s | Ego begins right motion; farther destination `slow_leader` gap84.698 m, speed10 vs ego16.445 | −2.854787 |
| Generic merge51.9 s | Ego begins left motion; destination `main_7` gap39.566 m, speed15.810 vs ego17.472 | −0.895512 |

The first two expose lack of anticipatory response to changing occupancy. The fourth exposes late adaptation to the chosen destination leader. The passing case additionally exposes an information-reduction problem: `ContinuousMotionLaneGraph.neighbor` (`highway_environment_v05.py:89`) selects the **nearest** leader across both occupied corridors. It keeps `main_8` at32.478 m and15.443 m/s, producing only−0.589049, while the farther, slower destination leader requires−2.854787 under the same model. Adjacent fields still expose the latter; the reference's ordinary response uses only own-front. A stronger jerk penalty cannot fix this omitted constraint. [Official ACC mode description](https://sumo.dlr.de/docs/Car-Following-Models/ACC.html).

## Complete active path inventory

| Layer | Active file/line and relevant behavior |
| --- | --- |
| Controller selection | `scripts/prepare_highway_smooth_control.py:109`: loads exact frozen TorchScript or explicit scripted adapter. No trained weights are replaced by v4. |
| Inference/recording | `scripts/record_highway_environment_v05.py:140`, `:176`: batched deterministic actor, then env.step; one native motion source. |
| Objective wrapper | `environments/highway_smooth_control.py:38`: changes labels/rewards/metrics only; same action execution when actor is fixed. |
| Execution | `highway_environment_v05.py:449`: normalized acceleration→[−4.5,2], discrete lane request; applies analytical ceiling, issues0.1 s acceleration and one native step. |
| Native modes | `i110_merge_env.py:182`: speedMode31, laneChangeMode512; native baseline adapter uses1621 and setSpeed(−1). 512 suppresses autonomous lane choice while retaining gap enforcement and speed adaptations; [SUMO mode documentation](https://sumo.dlr.de/docs/TraCI/Change_Vehicle_State.html#lane_change_mode_0xb6). |
| Physical lateral execution | `highway_environment_v05.py:257`: one target command with3.1 s lifetime; native lanechange.duration3 s; measured completions may be later, with no pose interpolation. |
| Guards | `highway_stage1_env.py:34`: speed, own-following and stopping-at-lane-end ceilings; `i110_merge_env.py:223`:1.5 s/braking-gap mask, internal-link restriction,5 s cooldown; v05 adds committed-maneuver and runway masks. |
| Observation | `highway_environment_v05.py:402`:99 fields, own/adjacent gap and relative speeds, map/merge context, ego lateral state, six fresh V2V slots. Exact SUMO truth; no cameras/LiDAR. |
| Context/topology | `traffic/i110_policy_interface.py:23`, `traffic/highway_merge_context.py:137`, `highway_environment_v05.py:55`: connected routes, first-entry vs later-merge semantics, dual-corridor occupancy. V3 explicitly rejects minor visibility other than4.5 m. |
| Demand | `traffic/highway_v05_demand.py:158`, `:268`: controlled ACC(tau1.5, accel2,decel4.5); three native IDM profiles(tau1.5–2, factor.94–1); deterministic matched positions, native sustained inflow, no warmup in this fixture. |
| Speed setting | `highway_population_env.py:189`: native lane limits read back; no observation-only speed spoof. |
| SUMO lifecycle | `i110_merge_env.py:147`, `:191`: direct TraCI start/advance, dt.1, teleportdisabled, collisionswarn/checkjunctions. `traffic/sumo_backend.py:37` supplies binary discovery; its generic `SumoBackend` stepping class is **not** used by this path. |
| Full-horizon accounting | `experiments/run_highway_stage1.py:150`: continues native tail through80 s after controlled trips finish; smooth wrapper records it. |
| Visualization | Retained renderer reports confirm recorded-position replay, not another driving controller, physical actuator or sensor loop. |

## Discriminating tests to propose, not execute

1. **Validate the intended roadway semantics first.** Keep current network immutable. Establish whether an upstream auxiliary stream really shares this ramp's first successor, whether the intended port is a priority entry or added-lane continuation, and what sight distance the synthetic/map model assumes. Compare empty and occupied true-foe cases on a separately versioned fixture. Preserve right-of-way, collision and stopping protections; do not grant75 m visibility as an undocumented performance tweak.
2. **Fix route-useful lane decisions before more learning.** Offline-replay the17.6 s pass decision with an auxiliary/end-distance-aware destination score and return feasibility. Test passing ordinary through lanes, legitimate exit use, and mandatory remerge separately. Add all controlled vehicles' induced remerge/delay to acceptance, not only original ramp identities.
3. **Audit all occupied/destination leaders and plan transitions.** Reproduce these four fits in focused unit fixtures. Compare most restrictive feasible response across relevant corridors versus nearest-only reduction; inspect measured lateral history and delivered intents, without future simulator state. Evaluate speed matching before lane-change initiation, not only jerk-limiting after a leader suddenly becomes own-lane.
4. **Ablate the reference mechanisms independently.** Entry finite-distance term, chosen-slot hold and ending-profile activation should each have explicit phase/feasibility conditions. A guarded profile derivative should be used when tracking that profile, not automatically because the route currently requires a merge. Preserve hard emergency authority. Compare paired full-trip time, progress at fixed timestamps, speed deficit, stops, lane changes and total delay before selecting labels for imitation.
5. **Only then propose training.** Revalidate a teacher on fixed train-only cases and distinct development seeds; retain old frozen actor/native baselines. Do not train merely to reproduce the current slow heuristic or lower override/jerk alone. Keep independent safety, throughput/progress, task and comfort metrics and unused held-out cases. Source changes require a reviewed config and fresh bounded preflight before a learning launch.

No future-test outcome or physical site accuracy is claimed. This audit leaves the prior failed readiness and all raw experiment evidence unchanged.
