# 35 mph physical-car demonstration

Status: implemented development preview; final isolated runtime validation
pending another chat's GPU batch, October 8, 2026. Not a validated highway fleet.

## Scope and design

User request: increase the physical obstacle-bypass car's cruise speed to 35 mph
(15.6464 m/s), not accelerate playback time. Preserve the original 3 m/s profile
under `--profile low-speed`. PhysX remains the sole movement authority; there is
no teleportation, SUMO movement, training or dependency installation.

The named `35mph` configuration uses a 550 m road, a nominal stop at X=500 m and
the same 2 x 1.6 x 1.5 m obstacle at X=300 m. Lane shifts extend from 20 to 70 m,
with 60 m longitudinal clearance and 160 m detection range. The reference radius
is (70²+3.8²)/(4*3.8)=323.32 m, versus 27.27 m previously. At 35 mph, ideal lateral
acceleration drops from about 8.98 to 0.76 m/s². These calculations motivate the
fixture; they do not validate its actual tire/suspension behavior.

Planning deceleration remains 1.5 m/s². At target speed, the conservative stopping
corridor is about 85.73–88.86 m beyond the front bumper for 0–0.2 s scan age,
compared with 4.6–5.2 m at 3 m/s. It is a planning buffer, not a stopping-distance
claim. The maneuver starts sufficiently early to avoid the straight emergency
brake's irreversible obstacle latch before the car begins turning.

The higher-speed profile increases rotary pattern firing from 7,200 to 72,000 Hz
while retaining 32 emitters and 20 Hz scans: nominal 0.1° horizontal spacing and
115,200 emitted samples per revolution. The old ~1° spacing can skip a 1.6 m
obstacle 160 m away. This is a custom synthetic sensor profile, not a certified
sensor model or guaranteed real-world detection range. Raw XYZ returns, not
obstacle labels/ground-truth coordinates, still drive the planner.

Physics/control/planning/render rates remain 120/60/10/30 Hz. All freshness,
identity, coordinate-frame, expiry, contact and asset-immutability checks remain.
Explicit speed-profile opt-in permits up to 16 m/s; default controller/brake
instances still reject speeds above 3 m/s. The target is 15.6464, not 16 m/s.

## Predeclared validation

1. CPU checks: preserve low-speed limits and historical resolved configurations;
   validate high-speed profile consistency, detection/turn geometry, longer
   stopping horizon, and unchanged stale-input behavior.
2. Run the `dropout` case first as a straight high-speed braking probe. It must
   cruise within 0.5 m/s of target for >=3 continuous seconds, be near target in
   the half-second before braking, and have initial yaw <=0.01 rad. Frozen data
   at 15 s must trigger full braking by the original 0.2-second-age deadline.
   Measured distance to <0.05 m/s must stay within the conservative stop envelope.
3. Run full pass and blocked-road variants, retaining every failed attempt.
   Require sustained target speed for >=3 seconds; pass mode also holds target
   within 0.5 m/s over the 20 m window centered on the barrier. Fault-mode stops
   begin near target, not after a slow approach that hides high-speed braking.
4. Keep zero contacts/departures, all four wheel supports, upright Z>0.99,
   clearance >=0.35 m, tracking error <=0.5 m, sensor-pose error <=0.05 m,
   final pass lane offset <0.25 m, endpoint error <=0.5 m, and five seconds
   of stopped hold. Road containment includes the authored 5 m end padding;
   braking distance sums traveled segments, not the shorter endpoint chord.
5. Repeat a pass. Before testing, declare aligned-position/speed tolerances of
   0.1 m / 0.05 m/s for this longer-range noisy-sensor profile. This does not
   rewrite the earlier low-speed migration's tighter tolerance or evidence.
6. Re-run the preserved slow profile. Report actual timing/memory separately;
   higher physical speed and more sensor rays do not establish real-time GUI
   playback, highway readiness or fleet capacity.

The episode is 60 simulated seconds, including two seconds settling and a final
five-second hold. The process retains a 540-second deadline and 90% GPU-memory
guard. Raw evidence is local under `outputs/vehicle_obstacle_bypass/`.

## Validation record

Two startup/partial attempts were deliberately interrupted when an unrelated
vehicle-qualification chat started a second Kit process on the same GPU:
`20261008T233201Z-44ea4337` and `20261008T233720Z-17011e2b`. Only this task's
verified child processes were stopped. Their logs, checkpoint chunks and failed
supervisor summaries remain intact, with separate interruption notes. Neither
run is a complete validation pass or an isolated performance measurement.

The second attempt's 39 seconds of checkpointed telemetry reached 15.6009 m/s
(34.90 mph), sustained target tolerance for 5.81 seconds and applied timely full
braking on frozen LiDAR. Recorded braking travel was 18.04 m from 15.6009 m/s,
versus an 88.37 m conservative planning envelope. This partial result cannot
establish the final hold, clean shutdown or uninterrupted obstacle-pass gates.

The third attempt, `20261008T234354Z-3590eae5`, completed all 7,200 ticks of
the `dropout` case with every physics, sensing, source-integrity and supervisor
gate passing, including the final hold and clean shutdown. However, the other
chat started another render job during the episode and a soak job afterward.
A separate concurrency note explicitly excludes this run from isolated timing,
GPU-capacity and final serial-runtime acceptance claims. No other task's
processes were stopped. The pass, blocked, repeated-pass and fresh low-speed
runtime checks remain pending; do not infer obstacle-pass success from braking.

The CPU suite initially caught a historical lane-fixture compatibility problem:
adding the named profile made its exact configuration comparison reject omitted
legacy defaults. The validator now explicitly interprets omission as low-speed,
while still rejecting a high-speed profile in that old experiment. Historical
configuration/result files were not rewritten. Independent review found the
same omission issue in the older visual LiDAR fixture and missing source-capture
entries for the new shared speed-profile module. Both legacy launchers now retain
low-speed-only validation and capture that dependency. Regression tests include
their historical configurations, not just newly constructed defaults.

Automated verification: **82 focused profile/compatibility tests pass**;
the final full traffic-environment suite reports **965 passed, 40 skipped**.
Both the original visual-LiDAR launcher's `--check` and the new profile's
`--check` pass. `git diff --check` passes. Skipped Isaac/USD tests are not new
physical validation. The feature branch is a development checkpoint, not a
promotion of the pending runtime gates.

## Commands

From the repository root with the traffic environment active:

```text
python scripts/demo_obstacle_bypass.py --profile 35mph
python scripts/demo_obstacle_bypass.py --profile low-speed
python scripts/demo_obstacle_bypass.py --profile 35mph --headless --mode dropout --unpaced
python scripts/demo_obstacle_bypass.py --profile 35mph --headless --capture --unpaced
python scripts/demo_obstacle_bypass.py --profile 35mph --headless --mode blocked --unpaced
```

The usual preview now selects the 35 mph profile by default. The HUD reports both
mph and m/s. Existing historical commands can explicitly select `low-speed` to
retain the prior geometry, scan pattern and driving target.

## Sources and limits

- [NVIDIA RTX LiDAR](https://docs.isaacsim.omniverse.nvidia.com/6.0.1/sensors/isaacsim_sensors_rtx_lidar.html)
- [Custom sensor profiles](https://docs.isaacsim.omniverse.nvidia.com/6.0.1/sensors/isaacsim_sensors_rtx_custom.html)
- Installed Example_Rotary asset and API readback are retained with each run.
- Known straight road and privileged ego odometry remain controller inputs.
  Static obstacle size bounds and an empty adjacent lane remain fixture assumptions.
- Google learning-guide sync remains pending the Windows trusted-reader issue;
  this note is the repository record, not a claim of Google Doc synchronization.
