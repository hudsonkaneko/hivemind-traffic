# SUMO MAPPO 2D recording

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
