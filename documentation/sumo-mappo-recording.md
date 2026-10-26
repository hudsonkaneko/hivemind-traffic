# SUMO MAPPO 2D recording

## October 26: native SUMO GUI capture repair

Native capture is now supported with `--native-gui`. This launches `sumo-gui`,
keeps the trained actor and SUMO safety controls, colors the four controlled
vehicles and follows them with the viewport. The frames come directly from
SUMO's screenshot API, not the schematic renderer described below.

An isolated unpaced native-size probe reproduced a blocked `simulationStep`
after three screenshot requests. Both native-size and resized probes completed
when given 0.15 seconds between screenshot requests and simulation steps.
The production recorder uses native-size frames, a conservative 0.2-second
GUI refresh pause and a frame-file completion check. A 15-second TraCI socket
timeout plus bounded owned-process cleanup prevents indefinite GUI hangs.
This identifies a capture pacing dependency on this workstation; it does not
claim to diagnose the underlying SUMO/OpenGL internals. Headless training's
timing and cleanup remain unchanged.

The trained-policy smoke captured 50 native frames. All 26 targeted backend and
environment tests passed, including forced GUI cleanup after a socket failure.
The failed unpaced probe and successful paced probes remain in ignored
`outputs/sumo_recordings/native-probe-20261026/`; the prior two-frame failed
attempt is preserved. No policy learning or safety changes were made.

Capture and encode from the checkout root using traffic Python:

```text
python scripts/record_sumo_mappo.py --native-gui --policy outputs/sumo_mappo/v01/train-20261026T042040Z-ad06049e/policy.pt --output outputs/sumo_recordings/<new-native-run> --seconds 60
python scripts/encode_sumo_native.py outputs/sumo_recordings/<new-native-run> --ffmpeg <local-ffmpeg-executable>
```

The encoder requires a complete native PNG sequence and verifies the decoded
MP4 frame count. It adds no graphics or overlays; H.264 requires at most one
pixel of padding for even dimensions. The isolated FFmpeg executable for this
run was extracted from the pinned `imageio-ffmpeg==0.6.0` Windows wheel into
ignored outputs, with wheel/binary hashes saved. No traffic or Isaac dependency
installation was changed. The manifest records policy and encoded video hashes.

The completed run `native-60s-20261026` produced 600 viewport PNGs and
`sumo-native-replay.mp4`, verified by decoding all 600 frames at 10 fps. Its
complete vehicle trajectories exactly match the earlier headless seed-400001
replay. The decoded midpoint was visually inspected. The clip retains the
actual native viewport, with no reconstructed vehicles or overlays. It had zero
collisions/overspeed and 73.583% interventions; the prior policy acceptance
failure is unchanged. See [compact evidence](evidence/sumo-native-capture-20261026.json).
Raw frames, the MP4 and encoder remain local under ignored outputs.

Reference: [SUMO screenshot API and next-step scheduling](https://eclipse.dev/sumo/pydoc/traci/_gui.html).

## October 25: historical schematic fallback

Recorded October 25, 2026, from the frozen actor in
`outputs/sumo_mappo/v01/train-20261026T042040Z-ad06049e/policy.pt`.
The local deliverable is
`outputs/sumo_recordings/mappo-20261025/sumo-mappo-replay.gif`:
600 frames, 10 fps, 60 simulated seconds, seed 400001. The recorded trajectories
and policy hash are retained alongside it. No optimizer or training ran.

This is a schematic visualization of actual SUMO positions, not a native GUI
screen recording. The native screenshot attempt stalled after two frames;
that attempt remains in `outputs/sumo_recordings/preview-20261025`, and only its
owned processes were terminated. The completed animation uses a following
camera and expanded lateral lane spacing for readability. Colored cars use the
trained actor; gray cars are ordinary traffic; orange outlines indicate actual
safety overrides. SUMO remains the motion owner throughout.

The replay completed 600 steps without collision or overspeed, with 1,572.795 m
mean per-car progress. Overrides affected 1,766 of 2,400 decisions (73.583%).
This remains a policy failing the intervention acceptance gate, not a successful
autonomous-driving demonstration. First/middle/final PNG previews are retained;
the middle frame was visually inspected and all 600 GIF frames were verified.

Reproduce from the checkout root using traffic Python for collection:

```text
python scripts/record_sumo_mappo.py --policy outputs/sumo_mappo/v01/train-20261026T042040Z-ad06049e/policy.pt --output outputs/sumo_recordings/<new-run> --seconds 60
```

Render with a separate Python containing Pillow (the bundled document/image
runtime was used without changing traffic or Isaac dependencies):

```text
python scripts/render_sumo_recording.py outputs/sumo_recordings/<new-run>
```

The animation and bulk traces are local ignored artifacts, not shipped in Git.
