# Provenance of this copy

- **Source.** The user's Google Drive folder of the LA I-10 / I-110 interchange
  model, `la_10_110.zip`, downloaded on November 18, 2026.
  - Zip SHA-256: `bfa125780337a7d6659d5509da44daca6bfdfcab714bc9a45e8d0ac1cb1ba2fd`
- **Contents.** The extracted `la_10_110/` folder, unchanged, plus this file. A byte-for-byte
  comparison with the extraction matched. The repository's `.gitattributes` keeps
  `highway_usd/**` byte-exact (`-text`).
- **Model documentation.** See [`README.md`](README.md): what the model contains,
  its frame, the simulation loops, how it was built, and its checks.
- **Licence.** The map data is © OpenStreetMap contributors, used under the Open
  Database License (ODbL). Keep this attribution in any copy or derived network.
- **Role.** From v12 on, this is the primary road model for training and
  simulation:
  - [`scripts/build_la_10_110_corridors.py`](../../scripts/build_la_10_110_corridors.py)
    cuts SUMO merge corridors from `lanegraph.json`, in this model's own ENU
    coordinates;
  - [the v12 plan](../../documentation/highway-merge-training-plan-v12.md) lists
    what is real and what is modeled.
- **Not run.** The bundled `isaac/*.py` helpers come from the model package. This
  project has not executed them.
