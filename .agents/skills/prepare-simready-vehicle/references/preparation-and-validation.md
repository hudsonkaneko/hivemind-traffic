# Asset inspection, packaging and validation

Use this reference for visual assets, profile-validated props and the shared
preparation work that precedes driving. These checks came from the Americano
pilot; choose numeric tolerances and profile requirements for the actual task.

## Inspect before authoring

Capture source hashes and inventory the selected prim or assembly. Check:

- Root/default prim, layer stack, payloads, variants and external references.
- Mesh counts, finite points, face counts/index validity, normals and bounds.
- Units metadata, all ancestor scales and transforms, up axis, forward axis,
  lateral axis and collection placement. Use named wheel positions and visual
  inspection together to resolve orientation.
- Active material bindings, shader networks, texture paths, UDIM patterns and
  unavailable network assets. Check dependency closure beyond what appears in a
  viewport; unused referenced collection content can still fail packaging.
- Existing rigid bodies, colliders, joints, mass properties, semantics and
  attachment points. Do not stack a new physics rig over an unnoticed old one.
- Available input formats. Convert CAD only when needed, preserving an immutable
  input and recording tessellation settings and the resulting geometry change.

Missing explicit units are an uncertainty to resolve. USD's fallback and an
already-authored 0.01 instance scale must not both be applied as independent
centimetre-to-metre conversions. Check the composed dimensions before accepting a
scale assumption; plausible dimensions are evidence for an assumption, not a
measurement of the real vehicle.

## Normalize and compose

Record a transform ledger: original reference frame, removal of collection
placement, scale conversion, orientation correction and new origin. Recompute
bounds after reopening the saved stage. Test cardinal headings and at least two
known placement anchors for a live simulator bridge.

For a metre-scale Z-up, +X-forward SUMO presentation asset, the pilot used
`yaw_degrees = 90 - sumo_heading_degrees` and a front-bumper ground origin. This
formula applies only after confirming those conventions. Geometry already behind
the front-bumper origin must not be shifted backward by half a vehicle length a
second time. Treat road elevation and pitch/roll as separate tested capabilities.

Use stable controllable roots with original vehicle IDs stored as metadata. A
reversible encoding such as UTF-8 bytes represented in hexadecimal avoids prim-name
collisions between IDs containing punctuation or Unicode. Instance a reusable
geometry/material model below each root when appropriate. Keep sensors as siblings
of the instanceable model; do not author into an instance proxy.

Define named attachments such as a roof LiDAR mount, front bumper, body center and
camera follow target. Recompute mounts when the body reference frame changes.
Keep physical mount extrinsics separate from a camera's display offset.

## Dependencies and presentation

For a collection, extract only the selected composed vehicle. Preserve transforms,
points, normals and topology when no geometry change is intended. If baking and
recentering parts for articulation or native wheel attachments, compare reconstructed
world-space points and topology with the source under a declared tolerance.

Reference subtrees carefully: a mesh can bind a material outside its referenced
subtree. Move or rebind the material to an included scope and check the resolved
binding in the final scene. Do not assume a file copying successfully means its
composition is portable.

When original textures are missing, keep that failure visible. A separate version
may use portable diagnostic `UsdPreviewSurface` materials for technical tests.
Opaque diagnostic glass and coarse body material labels do not establish optical,
LiDAR intensity or material segmentation fidelity.

Keep all dependencies within the boundary enforced by the chosen profile. In the
pilot, geometry in a sibling directory outside the main USD directory failed the
official check despite passing a broader project-local dependency check. Place
required geometry below the correct boundary and rerun the official checker.

Copy the completed package to a different directory and reopen/validate there.
Check unresolved assets, composition errors and resolved file locations, so an
absolute reference to the original tree cannot produce a false portability pass.
Record intentional external runtime dependencies separately from asset files.

## Profile and physics checks

Select the profile from the requested use, not from a desired green result. Record
its exact name/version, validator/core-tier versions and dependency lock. Do not
edit requirements or substitute project checks while calling the result an
official pass. Interpret blocking requirements and optional feature gaps according
to the validator's report; preserve the original report and exit code.

For the pilot's Robotics-Prop 4.0.0 package, authoring included mass/inertia, a
collider, class labels, provenance, and a real 256 by 256 rendered thumbnail at
`.thumbs/256x256/<main-usd-filename>.png` beside the USD. This is a recorded profile
layout, not a universal requirement for every future profile. Recheck the actual
profile before relying on it.

For a prop, test gravity, settling, ground contact, lateral drift and finite
velocities. Check collider shape/extent, center of mass and inertia against the
metadata. Collision geometry can use guide purpose while presentation geometry
remains visible to RTX. For traffic playback, inspect the composed entry and ensure
it has no active dynamics competing with the declared motion owner.

Author supported semantic schemas and verify them with an actual annotator when
claiming rendered labels. Modern `UsdSemantics.LabelsAPI` was used in the pilot;
inspect schema availability in the chosen runtime. A `car` class label or a
Wikidata identifier supplies classification, not verified material properties.

## Render and sensor probes

Warm-up duration varies with stage loading and shader caches. Wait for observable
readiness within a bounded deadline: correctly sized nonempty RGB and valid,
complete scans for each sensor. Fixed update counts alone failed in the pilot.
Log startup, readiness and measured rollout times separately.

For a paused timeline, ordinary app updates may not produce a capture. Use the
supported explicit capture API for that runtime. The pilot used Replicator's
`orchestrator.step(rt_subframes=2, pause_timeline=True)` for still images, followed
by stopping/detaching capture before a separately controlled physics probe. Do not
accidentally advance a live experiment with an extra rendering/physics step.

Use two instances to test mutual sensor visibility and independent streams. In a
controlled fixture, count valid hits in the peer's known bounds, then hide its
presentation geometry and repeat. Disappearing peer returns support a geometry
visibility claim. They do not establish semantic object-ID correctness. A rigid
collider is not the source of RTX mesh returns.

Test the roof mount for self-occlusion; moving a sensor inside the body should
produce an interpretable change. Check camera targeting, heading, grounding and
moving scan timestamps. Save raw sensor packets and actual frames, with frame
conventions, return validity and scan-completion metadata.

Detach annotators/writers, destroy render products, close simulator connections
and reap owned processes. Run GPU probes serially when needed to stay within the
machine's resource budget. Record whole-system memory as whole-system memory;
a short successful run with another GUI open is not a fleet or memory-leak test.

## Completion evidence

Use a new run directory for each attempt. Hash source assets, prepared inputs and
scripts; save configuration, runtime versions, reports, logs and previews. Check
input hashes again afterward. Keep failed preparation versions and failed runs so
the final report explains which change resolved each failure. Review the actual
delivered thumbnail after packaging and include its final hash.

Report formal profile results separately from contact, sensing and integration
checks. If the official service-based CAD workflow was not run, say which local
steps and standalone validators were run instead.
