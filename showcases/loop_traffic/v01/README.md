# v01 — vehicle and traffic integration

Status: verified within the bounded integration scope. Preserve the supplied prepared vehicle and V02 highway.
Assemble one physics-driven main car and twelve deterministic block cars through
references. Initial check: 30 s driving at 6 m/s, then an explicit brake/hold.
No passing claim is required at this stage.

Run `20261009T173729Z-dce58c1e`: all physics/composition gates passed, 12 moving
backgrounds, maximum main-car speed 5.9704 m/s, braking distance 2.6553 m,
9.125 s stopped hold, no contacts. Vehicle, road and static package hashes stayed
unchanged. Follow screenshot verified the supplied detailed vehicle.

Earlier `20261009T173505Z-e3db9ad7` stopped before driving: the scene manager
missed a typed-scene discovery event. Defining the scene before bulk USD copying
fixed it and now has a regression test. Both attempts are preserved locally.

Unpaced playback measured 0.878x including first-frame renderer warm-up; it is
**not** a real-time qualification. Separate renderer readiness from the live
window, then verify v03 pacing. Source and captures are under
`outputs/loop_showcase/<run-id>/`; these ignored artifacts are not on GitHub.

```text
python scripts/demo_loop_showcase.py --version v01
```
