# Vehicle provenance and configuration

Documented: 2026-09-11. Implementation: [waypoint_env.py](../scripts/waypoint_env.py).

## Where the car came from

The car is NVIDIA's **Leatherback**, a supplied, articulated robotics vehicle
asset. It was not modeled for this project. The scene loads this OpenUSD asset
from NVIDIA's Isaac Sim 6.0 asset collection:

[Leatherback USD asset](https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/6.0/Isaac/Robots/NVIDIA/Leatherback/leatherback.usd)

The starting reference was NVIDIA's [mobile robot controller example](https://docs.isaacsim.omniverse.nvidia.com/6.0.1/robot_simulation/mobile_robot_controllers.html).
That example provided the asset path and steering/wheel joint names. Our task
uses its own vectorized Ackermann calculation, built around measured dimensions.

The **vehicle asset** provides geometry, joints, collision shapes, and physical
properties. The **policy** is a separate neural network trained locally from
scratch with PPO. Downloading the car did not download a trained driving policy.

This is an approximately RC-scale car, not a validated full-size highway vehicle.

## Dimensions used

| Quantity | Value | Basis |
|---|---:|---|
| Overall length | Approximately 0.424 m | World-space bounds inspected during setup |
| Wheelbase | 0.32 m | Front wheel centers at x ≈ +0.15 m; rear at x ≈ −0.17 m |
| Track width | Approximately 0.2416 m | Wheel centers at y ≈ ±0.1208 m |
| Tire radius | 0.052 m | Wheel geometry bounds |
| USD root scale | 0.01 on each axis | Inspected asset transform |
| USD meters per unit | 1.0 | Inspected stage metadata |

The documentation example used a 1.65 m wheelbase, 1.25 m track, and 0.25 m tire
radius. Those dimensions did not match this downloaded asset. Using them produced
only about 1.64 m of forward travel in the original test. Correcting the dimensions
produced about 2.36 m over the four-second 0.6 m/s test.

## What is controlled

- Front steering joints: `Knuckle__Upright__Front_Left` and
  `Knuckle__Upright__Front_Right`, using position targets.
- Four wheel joints: `Wheel__Knuckle__Front_Left`,
  `Wheel__Knuckle__Front_Right`, `Wheel__Upright__Rear_Left`, and
  `Wheel__Upright__Rear_Right`, using angular-velocity targets.
- Suspension and remaining joints retain the asset's implicit actuator gains.

The asset exposed 26 degrees of freedom in the initial test. The task does not
ask the neural network to control all 26 independently; its two outputs become
coordinated wheel and steering commands.

## Local changes

The remote asset is referenced into the task stage; its server copy is not edited.

1. Articulation self-collisions are disabled for this isolated-car task.
2. `/Car/CollisionGroup` is deactivated in the source environment because PhysX
   reported unsupported collision-group replication. Road contact still exists.
3. Physics replication and Fabric cloning are disabled in the scene configuration.
4. Solver iterations are set to 8 position iterations and 2 velocity iterations.
5. Rear shock reset positions are −0.03 m and front shock positions are +0.03 m.
   Resetting all joints to zero violated the shock limits.
6. No camera or lidar data is used by the policy, even if sensor geometry or
   references are present in the supplied asset.

## Asset reproducibility

The simulator caches downloaded resources, but an uncached launch needs network
access. The source URL is versioned by asset collection, not pinned by a recorded
content hash. The initial download's exact dependency tree was not archived.
Consequently the original asset cannot yet be reproduced byte-for-byte solely
from this repository. A future reproducibility milestone should archive the USD
and dependencies with their hashes and applicable NVIDIA license information.
This record makes no claim that the asset is freely redistributable.
