# Semantic fusion CI dependency repair

November 8, 2026. GitHub run 42528883654 failed during collection because the
semantic fusion evaluation tests import PIL but the branch CI profile omitted
Pillow. Add Pillow==11.3.0, matching the existing later-branch CI profile.

The missing dependency was reproduced locally before installing the declared
version in the isolated traffic environment. All seven semantic evaluation
tests then passed, and the entire branch collected 3,169 tests without errors.
The old failure log is retained. The new head requires its own full remote CI
result; no runtime policy, simulator parameters or training changed.
