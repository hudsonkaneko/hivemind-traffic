# Real-time lidar results

One car, stationary obstacles, known straight two-lane map; no PPO, communication, or physics steering

Generated from saved run summaries and raw-evidence audits. See [implementation guide](realtime-lidar.md).

## Fixed validation suite

| Case | Behavior | Wall / simulated seconds | RTF | >100 ms late cycles | Alignment p95 max | Full gate |
|---|---|---|---|---|---|---|
| headless-a | True | 60.001 / 60.0 | 1.0000 | 0 | 0.256 m | True |
| headless-b | True | 60.001 / 60.0 | 1.0000 | 0 | 0.256 m | True |
| gui | True | 60.001 / 60.0 | 1.0000 | 0 | 0.256 m | True |
| blocked | True | 25.001 / 25.0 | 1.0000 | 0 | 0.201 m | True |
| dropout | True | 25.001 / 25.0 | 1.0000 | 0 | 0.222 m | True |

All suite gates passed: **True**. This is not a guarantee of hard-real-time behavior or crash-free future runs.

Repeat check (1 cm / 0.01 m/s tolerance and identical phases/lane requests): True; max differences: {'x': 0.0, 'y': 0.0, 'speed': 0.0}.

- headless-a: `20260929T001006Z-avoid-a33acb`; log `logs\realtime-suite-20260929T001006Z-headless-a.log`; audit error: none.
- headless-b: `20260929T001118Z-avoid-7189a8`; log `logs\realtime-suite-20260929T001006Z-headless-b.log`; audit error: none.
- gui: `20260929T001230Z-avoid-1b28c7`; log `logs\realtime-suite-20260929T001006Z-gui.log`; audit error: none.
- blocked: `20260929T001344Z-avoid-235207`; log `logs\realtime-suite-20260929T001006Z-blocked.log`; audit error: none.
- dropout: `20260929T001421Z-avoid-695266`; log `logs\realtime-suite-20260929T001006Z-dropout.log`; audit error: none.

### Process memory during the bounded loop

| Case | RSS at start | RSS at end |
|---|---|---|
| headless-a | 5535.2 MiB | 5496.3 MiB |
| headless-b | 5511.5 MiB | 5526.3 MiB |
| gui | 5786.9 MiB | 6267.2 MiB |
| blocked | 5527.1 MiB | 5501.8 MiB |
| dropout | 5520.1 MiB | 5526.4 MiB |

RSS includes the whole Isaac process, not just controller allocations; these bounded measurements do not prove leak-free indefinite operation. The GUI run grew by about 480 MiB, so continuous-service memory behavior remains an explicit follow-up, not a completed claim.

## Retained exploratory runs

| Run | Status | Error / result |
|---|---|---|
| 20260928T235226Z-avoid-aa3dce | failed | RuntimeError('Sensor/traffic clock drift 0.04999999999999985') |
| 20260928T235312Z-avoid-9e0321 | failed | RuntimeError('Sensor/traffic clock drift 0.04999999999999985') |
| 20260928T235346Z-avoid-5acc46 | failed | RuntimeError('Sensor/traffic clock drift 0.04999999999999985') |
| 20260928T235436Z-avoid-28c59a | failed | RuntimeError('No RTX packet after warm-up') |
| 20260928T235525Z-avoid-4bdcdb | failed | behavior=False; RTF=1.256440956095222; deadline misses=0; alignment=0.15355366468429565 |
| 20260928T235628Z-avoid-995e83 | failed | behavior=True; RTF=0.9999890434533821; deadline misses=2; alignment=0.3823035955429077 |
| 20260928T235802Z-avoid-1e3b7a | running | Native GUI startup access violation; log realtime-60-gui-01.log; no completed simulation |
| 20260928T235906Z-avoid-89586e | failed | behavior=False; RTF=1.1535361720565336; deadline misses=0; alignment=0.24485628306865692 |
| 20260929T000019Z-avoid-151ce8 | running | Startup stalled; only the matching child PID 9024 was terminated; original manifest intentionally unchanged |
| 20260929T000208Z-avoid-7df7a1 | failed | behavior=True; RTF=0.9999647172448753; deadline misses=1; alignment=0.2563714385032654 |
| 20260929T000411Z-avoid-18e431 | failed | behavior=True; RTF=0.9999892601153; deadline misses=1; alignment=0.25637054443359375 |
| 20260929T000523Z-avoid-07f86c | failed | behavior=True; RTF=0.9999853685474466; deadline misses=1; alignment=0.25604248046875 |
| 20260929T000635Z-avoid-e8b12f | failed | RuntimeError('Sensor/traffic clock drift 53.7') |
| 20260929T000748Z-avoid-0619c8 | failed | behavior=True; RTF=0.99996556518581; deadline misses=1; alignment=0.1993129551410675 |
| 20260929T000825Z-avoid-388814 | failed | behavior=True; RTF=0.9999650172238975; deadline misses=1; alignment=34.067291259765625 |

A retained manifest marked `running` belongs to an interrupted native process that could not finalize it; it is not a claim that the process is still active.

## Interpretation

Earlier exploratory runs overlapped a separate user-started baseline viewer. The fixed suite began after it exited. Exploratory failures are retained, not replaced.

The initial suite showed a repeatable full Python garbage-collection pause. A measured generation-2 collection lasted 176 ms and overlapped the late cycle. Deferred GC is scoped to the bounded demo and restored in a finally block; process memory measurements are saved in each run summary. Long-running service memory management is not validated.

Contract v2 corrects the dropout alignment audit: only fresh newly admitted packets are scored against obstacle geometry; rejected scans remain saved. Frozen old ego-body returns must not be treated as newly perceived obstacles when evaluating a stopped sensor.

The native COMPENSATED world-output probe completed avoidance at real-time average speed but failed alignment and deadline gates. NONCOMPENSATED world output and the lower-density demo profile were evaluated explicitly, not treated as identical sensors. Exact output transformation internals are not proven by this fixture.

The GPU runtime had an exploratory startup access violation and a separate startup stall. These are retained limitations, not claimed fixed. No external runtime binaries or driver settings were patched.

Warm-up, application initialization, final evidence hashing, and shutdown are excluded from loop real-time factor. GUI performance is evaluated separately from headless performance. Whole scans are still roughly 100–200 ms old at control time; real-time speed does not mean zero sensor latency.

Reproduce the fixed suite: `python experiments/run_realtime_suite.py`. JSON includes every recorded run configuration and manifest hash. Raw scans remain in ignored `outputs/`, runtime logs in `logs/`; large evidence is not pushed to GitHub.
