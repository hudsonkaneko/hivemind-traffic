# Clean-clone checkpoint fixture repair

November 8, 2026. Backport of the test-only repair validated on e9b7186.
The smooth initialization test previously required an ignored local training
checkpoint. It now creates a deterministic untrained checkpoint/export pair,
keeps real checksum/loading/parity checks, verifies a fresh critic and rejects
corrupted artifacts. Optimizer steps remain prohibited.

The affected test, runner, contract and model files were byte-identical across
all ten failing branches. The revised module passed all 67 tests on the oldest
affected branch; the same repair also passed the full GitHub CPU suite on the
current development branch. Each backported head receives its own remote check;
this note does not claim its CI has already passed. No policy/training changes.
