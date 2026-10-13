# Driving and Isaac Lab task readiness

Read this only when the requested scope includes physical driving or an RL starter.
Completing visual/prop preparation does not by itself require training a policy.

## Choose a physical representation

Inspect the installed simulator's supported vehicle implementation before choosing
between a native PhysX vehicle and an articulated chassis/wheel model. In the
Americano prototype, the native vehicle used raycast suspension/tire forces, a
rigid-body chassis and wheel actuator APIs. Lab accessed the chassis as a
`RigidObject`; it was not an `Articulation` with ordinary wheel-joint actuators.

Use PhysX as the rollout pose owner. Commands affect steering and wheel drive/brake
actuators; chassis pose/velocity writes belong to explicit reset operations. Keep
SUMO-driven visuals separate. An environment that teleports the chassis along a
path does not demonstrate learned steering/traction dynamics.

### Fit the vehicle to the asset

- Measure four wheel centers, axle spacing, track, wheel radius and width from the
  normalized geometry. Confirm front/rear/left/right ordering in the chosen API.
- Recenter body and wheel meshes into their respective physical frames. Connect
  each wheel visual to its own simulated attachment so it steers and rotates.
  Hide factory demonstration visuals while retaining required collision behavior.
- Fit wheel contact shapes and the body collision envelope. Preserve the intended
  suspension-frame offset/compression convention while moving the attachments.
- Inspect wheel-component relationships before authoring attributes. NVIDIA's
  Factory example can put wheel radius/width on shared component prims referenced
  by attachments. Writing a nonexistent attribute on the attachment caused an
  empty-typeName error in the pilot.
- Preserve existing USD transform-op types/precision when editing values, or
  deliberately rebuild the transform stack. Avoid duplicate operations.
- Rebind materials included outside mesh reference subtrees. Verify the assembled
  vehicle in an actual rendered frame and compare geometry before/after baking.
- Record mass, inertia, center of mass, tire/road friction, spring/damping/travel,
  wheel dimensions, steering limits/rate and drive/brake torque limits with units.
  Classify each as measured, inferred, calibrated or provisional. Do not copy the
  pilot's 1,800 kg mass or torque limits as real specifications for another car.

## Prove physical control first

Use a small flat-ground fixture before road or fleet integration. Check settling
and wheel contacts, forward acceleration, controlled braking, left/right steering,
turn radius and repeated resets. Record actual speed, displacement, yaw, upright
direction and contact state. Validate stale/invalid command handling and record
requested versus applied controls. A scripted controller should finish the simple
task before learner performance is considered.

Reset native vehicle rest state and wheel state as well as chassis pose, linear
and angular velocity. Clear command sequences, steering history and held commands.
Exercise explicit resets and automatic termination/time-limit resets. Verify
pose, velocity and wheel rotation speed after each and repeat a fixed seeded
trajectory within a declared tolerance.

The pilot used one CPU physics environment with fabric disabled. Native vehicle
cloning, GPU replication and large batches were not established by that result.
Start with the smallest known-working configuration and separately test any
scaling change. Do not borrow multi-environment claims from articulation examples.

## Define the RL task explicitly

For one car, a `DirectRLEnv` is sufficient; multi-agent tasks need a separate
decision about API and centralized/decentralized observations. Import simulator
extensions and environment classes only after the app launcher initializes the
runtime. Inspect the installed Lab version for reset signatures and quaternion
ordering; the tested Lab 3.0 beta path used `xyzw`.

Record physics interval, policy interval/decimation, action space and units,
observation fields/scales, reward terms, success, failure, timeout, seed, horizon
and reset distribution. Check that each control interval advances exactly the
intended number of physics ticks. Reject nonfinite actions before stepping.

A useful starter is a car reaching a nearby target from rest. The pilot used two
normalized actions: steering and signed longitudinal input, with positive input
mapped to throttle and negative to braking. That mapping does not implement
reverse. Keep steering-rate limiting and brake priority explicit. Log component
rewards separately and test their signs and dependence on the post-step state.

State observations can include goal-relative position, vehicle-frame velocity,
applied steering and previous actions. Label them **simulator state**. Enabling
RTX rendering does not make a state-based policy a perception policy. Introduce
camera/LiDAR observations only when the task consumes and validates them.

### Preserve the terminal transition

Read the installed environment's step order and the learner wrapper. The pilot's
Lab step calculated termination/reward, automatically reset the car, and then
returned observations. Its SB3 wrapper used those returned observations as the
terminal observation. That would bootstrap a timed-out transition from the next
episode's state.

Capture the observation before automatic reset. Supply that captured state as
`terminal_observation`, and set `TimeLimit.truncated` only for a truncation that is
not also a termination. Test a timeout and a successful/failed terminal episode,
including the returned next-episode observation. Recheck this behavior after
runtime upgrades; do not apply a historical wrapper workaround blindly.

## Training boundary and evidence

Check existing user authorization and repository instructions before launching
learning. If task design or budget has not been agreed where required, finish
the runnable environment, configuration, scripted baseline and bounded checks,
then present the concrete training setup. Do not launch unattended training just
because the asset is described as RL-ready. Highway Sim has an explicit user
preference for this, recorded in its current `AGENTS.md`.

For an authorized bounded trial:

1. Fix training seeds, step budget, evaluation seeds, algorithm/hyperparameters
   and checkpoint selection before execution. Save all effective settings,
   including settings inherited when resuming a checkpoint.
2. Run headlessly unless sensor observations require rendering. Record actual
   step count; PPO rollout sizes can round the requested budget.
3. Check observations, rewards, policy parameters and learning statistics for
   finite values. Preserve interrupted/failed run evidence.
4. Save checkpoints atomically with actor, critic, optimizer, normalization state
   if used, task metadata, step count and practical RNG snapshots. With SB3, use
   an explicit temporary `.zip` filename before replacement; assuming `.partial`
   would become `.partial.zip` failed in the pilot.
5. Load the saved policy, compare deterministic actions on the same observation,
   and evaluate it in a new process on the fixed held-out seeds. Compare against
   the scripted controller on the same targets. Record failures and timeouts.
6. Check continued training step accounting if resume is offered. State whether
   resumption restores exact simulator/RNG state or starts new seeded episodes.
7. Check visual replay separately and compare its metrics with headless replay.

A completed PPO run establishes working learning infrastructure. It does not
establish a competent driver. Report actual success rates, sample counts and
variability; avoid policy claims based on reward growth or one attractive video.

## Useful primary references

Inspect local source for the installed versions before adapting these interfaces:

- [Isaac Lab direct-environment tutorial](https://isaac-sim.github.io/IsaacLab/v3.0.0-beta/source/tutorials/03_envs/create_direct_rl_env.html).
- [SB3 PPO documentation](https://stable-baselines3.readthedocs.io/en/master/modules/ppo.html).
- [SB3 vector-environment terminal observations](https://stable-baselines3.readthedocs.io/en/master/guide/vec_envs.html).

These are navigation references, not a promise that the current release matches
the historical prototype.
