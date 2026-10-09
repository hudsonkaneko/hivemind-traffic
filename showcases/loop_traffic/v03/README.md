# v03 — presentation package

Status: physical safety and repeatability verified; **visual qualification failed**.
Add follow and overview views, cyan planned-path overlay,
speed/traffic/pass/status display and default real-time pacing. Verify a repeat,
blocked-lane following, command-loss braking and a longer circulation run. Keep
earlier versions and failed attempts. Isaac Lab and policy training are deferred.

The 90-second independent pair `20261009T174736Z-e4f828b5` and
`20261009T174856Z-d6a48d83` had exactly equal physical trajectories, seven passes
and two changes. Comparison `20261009T175035Z-e187a731` verified all recorded
hashes. This did not detect the rendering problem: native kinematic poses did
not automatically publish to the visible USD transforms. At t=20, native peer
000 was 3.79 m from ego while its visible geometry remained at spawn. The image
audit rejected this as a finished showcase. See v04 for the correction.
