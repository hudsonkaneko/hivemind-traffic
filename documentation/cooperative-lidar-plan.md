# Two car lidar cooperation plan

Implementation started September 30, 2026. Extend the real-time single-car baseline to two separately sensed and controlled cars, bounded moving-object tracking, and explicit vehicle-to-vehicle messages. Keep the existing demo unchanged. No PPO, learned communication, low-level steering, or ovrtx migration in this milestone.

## Experiment contract

SUMO is the sole motion authority, with one batched 0.1-second traffic advance for both cars. Isaac supplies two independently bound world-space lidar streams. Each controller receives only its own odometry, its own filtered lidar and tracks, and fresh permitted messages. Ground-truth obstacle and peer poses are evaluation-only. Vehicle IDs and episode IDs are explicit.

The first fixture places the passing car in the right lane and another controlled car just behind in the left lane. A stationary box blocks the right lane. With communication enabled, the passing car broadcasts a lane-change intention; the other car yields while continuing to use its own lidar safety checks. Without communication, both remain sensor-driven and must find a safe gap without that coordination signal. Delayed/dropped messages must not authorize an unsafe merge.

## Validation gates

- Preserve the prior unit tests and add tracking, message expiry/order/isolation, controller safety, and batched TraCI tests.
- Run CPU synthetic smoke tests before GPU tests. Save unique source/config/raw evidence and retain failed runs.
- Validate separate sensor identities and timestamps; reject stale or mismatched data. Track storage and message queues must be bounded; unseen moving tracks expire.
- For short real-time runs, target RTF 0.95–1.05 and zero control deadline lateness >100 ms after initialization; report GUI separately.
- Require no vehicle/obstacle overlaps, body footprint inside the road, and >0.3 m conservative obstacle/intervehicle separation. Log safety overrides instead of disabling SUMO protections.
- Compare matched no-communication, ideal communication, and degraded communication (seeded delay/drop); report outcomes and yielding/merge timing without assuming communication helps.
- Repeat a fixed configuration; add a second seed. Sensor dropout must trigger braking, no new lane requests, and retained fault evidence.
- Measure RSS periodically rather than only at endpoints. Compare GUI point visualization enabled/disabled and repeated runs to narrow the earlier memory-growth risk. Bounded demo stability is not proof of an indefinitely running service.

## Parallel ownership

GPT-6.1-sol agents implement the standalone tracker and message protocol and review runtime risks. The primary agent integrates controllers, sensor/TraCI wiring, serial GPU experiments, results, and documentation. GPU work is not delegated concurrently.
