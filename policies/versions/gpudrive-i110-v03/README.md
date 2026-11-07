# GPUDrive I-110 v03 — diagnostic, not accepted

Frozen pretrained GPUDrive actor; same weights and 2,984-value observation contract
as v01, 91 discrete acceleration/continuous-steering combinations. No training.
Restores the actual steering output instead of v02's discrete lane translation.
Adds explicit longitudinal limits, 60 mph/lower legal cap, and bounded footprint
checks. SUMO runs traffic; external bicycle motion runs the main car.

The 1.1-second test ends on an unsupported route association before completing a
merge. **Not a working merge policy or replacement for accepted MAPPO weights.**
See [changes and metrics](../../../documentation/gpudrive-i110-v03.md) and
[configuration](../../../configs/evaluation/gpudrive-i110-v03.json).
