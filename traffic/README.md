# Traffic

TraCI runners and vehicle-control code belong here.

`sumo_backend.py` owns one labeled TraCI connection and exposes deterministic
`reset`, `step`, and idempotent `close` operations. `scripted_baseline.py` runs
the first non-learning controller and creates an immutable telemetry package.

