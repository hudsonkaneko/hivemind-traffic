# GPUDrive I-110 v05 — comfort filter, not a newly trained policy

Same frozen actor and 2,984-value / 91-action interface as v04. Adds a route
connection audit and an explicit longitudinal comfort filter: −4.5/+2 m/s²,
1.8 s desired headway, 3 m/s³ routine jerk. Original guard runs last; steering
and 60 mph/lower legal limits remain unchanged.

One seed reached the goal in 28.4 s with no stop/contact/overspeed. Peak scalar
longitudinal jerk fell 60→3 m/s³. Raw commands are still modified 99.29%, and the
car crosses onto the mainline 33.41 m before the ramp ends without a network
connection. **Not a verified proper merge, accepted policy or training result.**

[Configuration](../../../configs/evaluation/gpudrive-i110-v05.json) ·
[Changes, limitations and metrics](../../../documentation/gpudrive-i110-v05.md)
