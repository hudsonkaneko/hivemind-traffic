# Scene v1: circular highway

Created the first versioned highway scene at
`scenes/highway/v1/highway_v1.usda`.

## Geometry and behavior

- One-way clockwise loop with a 30 m centerline radius.
- Three lanes, each 3.7 m wide (11.1 m total paved width).
- Closed annular visual road mesh with 0.12 m thickness.
- Solid inner/outer edge lines, dashed lane separators, and repeated clockwise
  arrows to make the intended travel direction unambiguous.
- 120 m square ground slab immediately beneath the road provides stable tire
  collision; the scene also includes a basic sun light.

The scene uses meter units and Z-up coordinates.
`scenes/highway/v1/generate_scene.py` is the source generator for the checked-in
USD file and exposes the main road dimensions as constants for future iterations.
