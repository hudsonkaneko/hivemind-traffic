# GPUDrive I-110 v04 — merge diagnostic completed, not qualified

Same frozen actor as v03: 2,984 observation values; 91 acceleration/steering
combinations. No training or new weights. SUMO route association now follows the
actual continuous pose and restores a connected route to the unchanged goal.
Steering and the 60 mph/lower legal speed cap remain intact.

One seed completed the merge criterion at 8.3 s and reached the goal at 28.5 s,
with no stops, contact or overspeed. Acceleration interventions were 89.79%; peak
jerk was 60 m/s³, and background traffic logged emergency braking. Not accepted
under the 5% intervention gate and not a promoted replacement for MAPPO.

[Config](../../../configs/evaluation/gpudrive-i110-v04.json) ·
[Changes and results](../../../documentation/gpudrive-i110-v04.md)
