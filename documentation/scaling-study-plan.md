# Capacity and roadmap foundations — October 3, 2026

Exploratory study, not evidence that coordination improves traffic.

Question: how much additional cost comes from (a) SUMO vehicles/commands,
(b) rendered cars and (c) independent RTX LiDAR streams on this workstation?
Hypothesis: additional sensor streams dominate additional box-body traffic.
Primary outcomes: steady-step/frame p50/p95/max duration, sustained real-time
factor, CPU-seconds, process RAM and sampled global GPU memory/utilization.

Predeclared SUMO population set: 2, 20, 50, 100, 200. Seeds: 42, 43, 44.
Variants: native SUMO; 50% scripted-speed AVs at 2 Hz; same at 10 Hz.
Each: 2 simulated seconds warmup, 20 measured seconds, 0.1-second steps.
Matched role selection and demand, safety modes retained, batch commands then
one simulationStep. No communication or perception in these CPU probes.
Native mode uses the same role assignment but delegates ALL driving to SUMO;
AV tags alone must not be interpreted as coordinated autonomous behavior.

Predeclared RTX probes: (cars, sensors) = (2,0), (2,1), (2,2), (20,2),
(50,2), (20,4). One exploratory run each, followed by a repeated (2,2) probe.
Simple moving boxes, 30 render steps/s, 32 emitters, 7200 firings/s, default
rotary profile, acquisition/callback counts checked, no perception/controller
or full scan recording. Repeatable geometry, not a vehicle dynamics test.
Measure 240 updates after warmup. Bound each process wall time and monitor
global GPU memory. Stop expansion at >=90% VRAM or failure; retain failures.
These stripped-down frame timings are NOT end-to-end controller capacity.

Reliability ablation: refresh stationary sensor frames after GC preparation
before establishing the control-loop sensor epoch. Run two fixed-seed ideal
communication 25-second checks; independently audit evidence and inspect initial
freshness. No claim that this fixes all native RTX nondeterminism or long-run
timing/memory growth. Do not hide failures or choose new seeds after results.
Also run one separately labeled 120-second diagnostic after the short checks,
retaining failure if the previously observed long-run deadline issue recurs.

System measurements include other desktop applications where stated. A fast
average does not guarantee deadlines: retain worst-case and p95 measurements.
Project calculations must separate measured data, arithmetic workload estimates,
and extrapolation. Keep at least 20% resource headroom as an engineering target,
not as a proven safety margin. Do not lower sensor/control quality silently in
an effectiveness comparison. Low-cost traffic still contributes scene geometry
and must remain visible to nearby sensors.

Sources: SUMO TraCI Python guide (subscriptions and optional libsumo),
https://sumo.dlr.de/docs/TraCI/Interfacing_TraCI_from_Python.html ; NVIDIA performance
handbook (viewport, pixel count, textures, profiling),
https://docs.isaacsim.omniverse.nvidia.com/latest/reference_material/sim_performance_optimization_handbook.html .
