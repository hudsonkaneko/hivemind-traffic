# Cooperative lidar results

Two-car scripted static-obstacle fixture with local lidar tracking and optional V2V intents; SUMO motion authority

Communication ablation is a fixture comparison, not statistical proof of cooperative-policy performance or general safety.

Every supplied attempt is retained below, including failures and unfinished cases. Missing measurements are N/A.

| Suite / case | Full pass | Audit integrity | Complete time (s) | Min car gap (m) | Min obstacle gap (m) | RTF | >100 ms late |
|---|---|---|---|---|---|---|---|
| suite-20260930T092530Z-6b36ad40ff3e / headless-none | True | True | 28.200 | 1.160 | 0.589 | 1.000 | 0 |
| suite-20260930T092530Z-6b36ad40ff3e / headless-ideal | True | True | 19.700 | 1.215 | 1.071 | 1.000 | 0 |
| suite-20260930T092530Z-6b36ad40ff3e / headless-degraded | True | True | 20.000 | 1.215 | 1.064 | 1.000 | 0 |
| suite-20260930T092530Z-6b36ad40ff3e / repeat-ideal | True | True | 19.700 | 1.215 | 1.088 | 1.000 | 0 |
| suite-20260930T092530Z-6b36ad40ff3e / seed43-ideal | True | True | 19.500 | 1.215 | 1.089 | 1.000 | 0 |
| suite-20260930T092530Z-6b36ad40ff3e / dropout-ego | True | True | N/A | 1.160 | 1.180 | 1.000 | 0 |
| suite-20260930T092530Z-6b36ad40ff3e / gui-points-off | True | True | 20.000 | 1.215 | 1.087 | 1.000 | 0 |
| suite-20260930T092530Z-6b36ad40ff3e / gui-points-on | True | True | 19.200 | 1.215 | 1.118 | 1.000 | 0 |
| suite-20260930T092530Z-6b36ad40ff3e / extended-ideal | False | True | 19.500 | 1.215 | 1.091 | 1.000 | 28 |

## Memory and communication

| Suite / case | RSS delta (MiB) | Private growth (MiB) | Last-half RSS slope (MiB/s) | Sent | Delivered | Dropped | Expired |
|---|---|---|---|---|---|---|---|
| suite-20260930T092530Z-6b36ad40ff3e / headless-none | 12.590 | -64.637 | 0.275 | 0 | 0 | 0 | 0 |
| suite-20260930T092530Z-6b36ad40ff3e / headless-ideal | 14.238 | -65.098 | 0.272 | 900 | 899 | 0 | 0 |
| suite-20260930T092530Z-6b36ad40ff3e / headless-degraded | -35.734 | -63.461 | 0.269 | 900 | 699 | 197 | 0 |
| suite-20260930T092530Z-6b36ad40ff3e / repeat-ideal | 12.668 | -58.996 | 0.311 | 900 | 899 | 0 | 0 |
| suite-20260930T092530Z-6b36ad40ff3e / seed43-ideal | 12.508 | -67.078 | 0.275 | 900 | 899 | 0 | 0 |
| suite-20260930T092530Z-6b36ad40ff3e / dropout-ego | 8.562 | -68.102 | 0.324 | 500 | 499 | 0 | 0 |
| suite-20260930T092530Z-6b36ad40ff3e / gui-points-off | 400.203 | 345.266 | 10.940 | 1200 | 1199 | 0 | 0 |
| suite-20260930T092530Z-6b36ad40ff3e / gui-points-on | 451.414 | 351.312 | 0.941 | 1200 | 1199 | 0 | 0 |
| suite-20260930T092530Z-6b36ad40ff3e / extended-ideal | 43.340 | -40.453 | 0.334 | 2400 | 2399 | 0 | 0 |

RSS measures the whole runtime process. Private growth is unavailable when the runtime did not record it. Bounded memory samples do not establish leak-free indefinite operation.

## Repeat and second seed

- outputs/cooperative_lidar/suite-20260930T092530Z-6b36ad40ff3e.json: exact simulation repeat=False, audits=True; phase/request equality=False; maximum differences={'ego.x': 0.5543026319425337, 'ego.y': 0.12800000000000056, 'ego.angle': 1.4758113174379872, 'ego.speed': 0.2999999999999998, 'peer.x': 0.8400000000000318, 'peer.y': 0.0, 'peer.angle': 0.0, 'peer.speed': 0.40000000000000036}; error=N/A.

The seed-43 case is listed separately. One additional seed does not provide statistical confidence; communication modes share a scripted fixture rather than a learned or randomized traffic policy.

## Retained failures and evidence

- headless-none: output=outputs/cooperative_lidar/20260930T092531Z-none-b27999; log=logs\cooperative-suite-20260930T092530Z-6b36ad40ff3e-headless-none.log; result=True; errors=none.
- headless-ideal: output=outputs/cooperative_lidar/20260930T092628Z-ideal-970fce; log=logs\cooperative-suite-20260930T092530Z-6b36ad40ff3e-headless-ideal.log; result=True; errors=none.
- headless-degraded: output=outputs/cooperative_lidar/20260930T092726Z-degraded-763086; log=logs\cooperative-suite-20260930T092530Z-6b36ad40ff3e-headless-degraded.log; result=True; errors=none.
- repeat-ideal: output=outputs/cooperative_lidar/20260930T092823Z-ideal-a10cfd; log=logs\cooperative-suite-20260930T092530Z-6b36ad40ff3e-repeat-ideal.log; result=True; errors=none.
- seed43-ideal: output=outputs/cooperative_lidar/20260930T092921Z-ideal-d6e8fa; log=logs\cooperative-suite-20260930T092530Z-6b36ad40ff3e-seed43-ideal.log; result=True; errors=none.
- dropout-ego: output=outputs/cooperative_lidar/20260930T093018Z-ideal-b8a7eb; log=logs\cooperative-suite-20260930T092530Z-6b36ad40ff3e-dropout-ego.log; result=True; errors=none.
- gui-points-off: output=outputs/cooperative_lidar/20260930T093055Z-ideal-fe30bb; log=logs\cooperative-suite-20260930T092530Z-6b36ad40ff3e-gui-points-off.log; result=True; errors=none.
- gui-points-on: output=outputs/cooperative_lidar/20260930T093209Z-ideal-72100d; log=logs\cooperative-suite-20260930T092530Z-6b36ad40ff3e-gui-points-on.log; result=True; errors=none.
- extended-ideal: output=outputs/cooperative_lidar/20260930T093323Z-ideal-8be9c5; log=logs\cooperative-suite-20260930T092530Z-6b36ad40ff3e-extended-ideal.log; result=False; errors=Runner summary did not pass.

## Provenance and limits

The JSON report retains each supplied suite snapshot, launch command, exit status, logs, manifest hash, archived source hashes, Git commit and working-tree state, Python/SUMO versions, GPU/driver information, and any recorded runtime dependency versions.

Audits reconstruct per-car raw sensor filtering, tracking, controller decisions and seeded message delivery. Safety gaps and peer visibility are evaluated after decisions against fixture geometry. Receipt age and wall deadline measurements are hashed runtime observations; they are not independently timed by this report.

Loop real-time factor excludes startup, warm-up, final hashing and shutdown. GUI and headless cases remain separate. Results do not certify hard real-time deadlines, vehicle physics, long-running service stability, or city-scale coordination.

All supplied attempts and available repeat checks passed: False.
