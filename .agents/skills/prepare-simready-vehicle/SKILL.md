---
name: prepare-simready-vehicle
description: Prepare and validate a USD or CAD vehicle for OpenUSD traffic playback, an explicitly named SimReady physics profile, or a drivable Isaac Sim and Isaac Lab prototype. Use for asset inspection, normalization, dependency packaging, wheel and collider setup, sensor checks, and RL task readiness. Fleet policy training and traffic experiments remain separate tasks.
---

# Prepare a SimReady Vehicle

## Highway Sim repository scope

Read the repository's `AGENTS.md` and `documentation/project-skills.md` before
applying this workflow. The active roadmap determines the selected backend:
SUMO authority statements below apply to SUMO-backed experiments, not to native
PhysX-controlled vehicles. Preserve one motion owner per vehicle. Repo-local
skills do not authorize training, dependency installation, or publication of
local-only vehicle assets; follow the current task and project authorization.

Produce a versioned vehicle with evidence for its intended simulation use. State
the achieved capability precisely: a rendered model, a profile-validated prop,
a physically driven vehicle, and an RL task have different acceptance criteria.

## Establish the requested result

Inspect the workspace instructions, existing asset versions, runtime configuration,
and previous evidence before creating another pipeline. Resolve the pilot asset,
its source prim or CAD assembly, supplied dependencies, and intended use from the
request and files. Start with one representative vehicle when preparing a collection.
Do not expand to the fleet before the pilot is checked.

| Requested use | Motion owner | Required evidence |
| --- | --- | --- |
| Traffic playback and synthetic sensing | SUMO or the declared upstream simulator | Correct placement and heading, stable IDs, reusable geometry, camera and sensor checks |
| Rigid-body prop | Physics engine | Mass/inertia and collider checks, contact behavior, and the exact requested profile result |
| Physically driven vehicle | Physics engine through steering and wheel actuators | Wheel/contact alignment, propulsion, braking, steering, suspension and repeatable resets |
| Isaac Lab RL starter | Physics engine; policy provides actuator commands | Driving evidence plus observations, actions, rewards, episode boundaries, scripted baseline and authorized trainer checks |

Maintain separate entries or explicit variants when different modes need different
physics. Never let two components write the same chassis pose during a rollout.
A prop-profile pass does not establish vehicle dynamics or policy competence.

## Prepare the asset

Read [preparation-and-validation.md](references/preparation-and-validation.md)
for the detailed inspection, packaging, profile and sensor checks.

1. **Preserve source and provenance.** Record source paths, hashes, author/license
   information or unknowns, and permitted use. Keep originals unchanged and create
   a new numbered version for meaningful asset changes. Honor existing local-only
   instructions; validation metadata does not establish redistribution rights.
2. **Inspect composed geometry.** Resolve layers, references, materials, textures,
   instance transforms, units and axes. Measure world-space bounds and wheel
   locations. Record missing dependencies and existing physics before converting.
   Existing usable USD may need no CAD conversion.
3. **Normalize deliberately.** Document the source-to-target transform, metres,
   up/forward/left axes and reference origin. Account for both stage metadata and
   authored scales. A SUMO front-bumper origin and a physical chassis origin need
   an explicit conversion. Validate known positions and headings numerically.
4. **Make a focused package.** Extract the selected vehicle and required materials
   without carrying the whole collection's unused dependencies. Use resolvable
   references, a valid default prim and the selected profile's directory boundary.
   When textures are unavailable, document diagnostic material substitutions and
   their limitations; do not describe them as restored appearance.
5. **Add properties for the selected use.** Keep visual geometry, physics and
   sensor attachments distinguishable. Record measured, inferred and provisional
   values separately. Use provisional dynamics only within the authorized technical
   testing scope. Add semantics and provenance using the target runtime/profile's
   supported schemas, then test their actual downstream behavior.
6. **Validate the delivered version.** Reopen it in the target runtime, exercise
   the requested mode, check geometry preservation and source hashes, and copy
   the package to another directory to test dependency resolution there. Run a
   named official profile unchanged when formal compliance is requested and the
   validator is available. Preserve failures and optional gaps in the report.

For a drivable vehicle or RL starter, read
[isaac-lab-readiness.md](references/isaac-lab-readiness.md). Establish physical
actuation and scripted control before learning. A native PhysX vehicle is not
automatically an Isaac Lab articulation, and one working vehicle does not prove
that simulator cloning or GPU batches work.

## Runtime and authorization boundaries

- Prefer the configured runtime and existing tested adapters. Keep USD inspection,
  standalone validation, traffic Python and Isaac Python dependencies isolated.
  Resolve API differences from installed code and official versioned documentation.
- Identify the official workflow actually used. Standalone profile validation is
  distinct from NVIDIA's complete CAD/Content Agents pipeline. If a required
  service is unavailable, record that limit and continue independent local work
  within scope. Do not claim skipped conversion, assignment or packaging stages.
- Use bounded probes with readiness conditions, explicit cleanup and unique output
  paths. Preserve the user's running applications and installed runtime source.
  Retain runtime warnings instead of changing vendor code to hide them.
- Preparing an RL environment does not by itself authorize training. Check the
  current request and project rules for an agreed task, algorithm, budget and
  evaluation. Reuse existing authorization when it covers the planned run.

## Evidence and handoff

Deliver explicit asset entry paths and versions, dependency/provenance records,
units/origin/mount definitions, physics assumptions, actual previews, scoped
validation reports, and rerunnable commands. Record exact runtime/profile versions,
configuration, seed, input/script hashes, raw measurements, exit status and cleanup
results. A successful process exit alone is insufficient if the report failed.

Report each relevant claim as passed, failed, blocked or untested. Separate
schema/profile conformance, rendered sensing, physical behavior, RL execution,
policy performance and long-run scalability. Do not transfer a result across
asset versions without rerunning the checks affected by the change.

The installed skill's private vehicle case study is deliberately not distributed
with this repository. Use only evidence available and authorized in the current
checkout. Historical numeric examples in the reusable references are not defaults
for other vehicles and are not independently reproducible from this skill bundle.

When available, `stream-traffic-openusd` and `build-omniverse-traffic-viewer` handle
the downstream live scene and viewer; `run-reproducible-av-experiments` handles
experiment evidence. The existing `validate-av-isaac-lab` skill addresses fleet
policy validation. This skill can prepare a standalone car without first training
a SUMO policy.
