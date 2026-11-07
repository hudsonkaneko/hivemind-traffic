# GPUDrive guarded I-110 v02

**Frozen external PPO weights; changed execution adapter; rejected transfer.**

The same 51,228-parameter actor and 2,984-input/91-action contract as v01 now feed
our native SUMO safety stack through a steering-to-lane-intent adapter.
Ceiling: 60 mph, respecting lower road limits. Checkpoint SHA256 remains
`f3f26475def35f375c6c72d8f8f20b2b091f77175010345dc3fa968a860521b7`.

One 80-second I-110 test recorded zero collisions/overspeed, but no stable merge,
71.9 s stopped exposure and 92.25% combined interventions. No training or promotion;
existing accepted policies remain unchanged.

[Configuration, metrics and replay](../../../documentation/gpudrive-i110-v02.md).
