# Highway scene v1

`highway_v1.usda` is a simple, drivable, one-way circular highway for Isaac Sim.
It has three 3.7 m lanes centered on a 30 m radius, solid shoulder lines, dashed
lane dividers, and clockwise direction arrows. A flat ground collider sits just
beneath the rendered road to provide stable tire contact.

Open `highway_v1.usda` directly in Isaac Sim. To regenerate it after changing
the constants at the top of `generate_scene.py`, run the generator with Python.

Future highway revisions should be placed next to this folder as
`scenes/highway/v2`, `scenes/highway/v3`, and so on, leaving prior iterations
intact. Other scene families can use their own folders beneath `scenes/`.
