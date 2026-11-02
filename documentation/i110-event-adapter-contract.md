# I-110 event adapter contract

28 October 2026 · Codex for Sam · bounded synthetic implementation.

Sam's continuation request authorizes scripted engineering while the frozen
checkpoint is unavailable. `traffic/i110_events.py` is the first traffic adapter
for the separate disruption layer. It supports one scripted leader brake or
safe cut-in request per run. Construction, disabled-car occupancy and police
sweeps remain unsupported and are rejected, rather than mapped to a visual toggle.

The permanent network is copied unchanged and hashed. Event JSON owns a stable
ID, source classification, actor identity, route/initial position, physical lane,
onset, duration or brake/hold/recovery schedule, release, extent and capabilities.
These v1 fixtures use synthetic mainline actors; `initial_physical_lane` 0–3
follows the persistent through lane, while the adapter accounts for SUMO's
auxiliary-lane index shift on sb1–sb3. Internal junction indices are not physical
lane IDs. The initial position is an sb0 lane position, not a global corridor station.

The actor is an explicit cohort request. Event-off adds no actor and produces no
commands. Controller-independent event files and demand bytes are matched across
native and filtered ACC. Event truth and future schedules are evaluator data;
they are not added to the 30 actor features. The interface still observes exact
current SUMO state, not sensor perception.

Each simulation step observes the previous request's result, then applies event
and controller requests, advances SUMO once and records the resulting state.
The speed schedule targets the end of that next step. The event journal contains
requested/effective times, speed, road/lane request, actual speed/acceleration and
road/lane/position, plus signed speed error. A command with an absent actor is
recorded and fails realization. Release returns speed authority to native IDM;
the event actor retains mode 512 for its through route. No source geometry is edited.

`speedMode=31` and `laneChangeMode=512` remain active. The latter preserves
collision and safety-gap checks; a lane-change request may be delayed or denied.
See the [official TraCI control contract](https://sumo.dlr.de/docs/TraCI/Change_Vehicle_State.html#lane_change_mode_0xb6).
This is SUMO's configured logic, not a guarantee of collision-free operation.
The emergency fixture changes only its synthetic leader's declared deceleration
to 9 m/s²; the four controlled cars keep their existing capabilities.

Realization is a separate gate: the cut-in must transition to its requested
physical lane and appear as agent_1's leader; a braking actor must stop and
realize at least 90% of requested braking while agent_1 follows. Missing actors,
unreleased events and unrealized requests fail the fixture gate. Comfort is a
separate diagnostic: the 2.05 m/s³ bound is never enforced by weakening safety.
Request/realization mismatches do not identify which SUMO internal check intervened.
Native ACC internal overrides remain unobservable and are reported as null.

`lanechanges.xml` records realized ramp/event changes. It does not enumerate all
rejected ramp opportunities. The existing ledger retains every mainline/ramp
request, external wait, active/removed/completed status, fixed-horizon burden and
separate bounded drain. No incident actor may be added outside that ledger.

Next adapters need explicit lane/footprint and permission lifecycle semantics:
occupied stopped vehicle, then closure/reopening with overlap ownership, then
moving multi-lane obstruction. Each needs event-off equivalence, lifecycle/reset,
realization, full accounting and paired input tests. A future USD consumer should
read the same event ID and realized trajectory with a declared time/coordinate
mapping. USD visibility alone never satisfies the traffic effect gate. SUMO or
PhysX owns a vehicle's motion at a time; integration is still pending.
