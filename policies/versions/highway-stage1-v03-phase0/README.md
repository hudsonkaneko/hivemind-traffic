# Stage 1 v03 phase-0 baseline

Restored as the reference for population-policy analysis, not promoted for deployment. Exact previously trained weights; no new training or simulation.

Shared MAPPO actor: 93 inputs (45 local/merge/mask fields plus six 8-field messages), masked message pooling, 128/128 MLP, normalized acceleration and lane intent. Local SUMO/map state, not camera/LiDAR. SUMO executes motion with the existing guards.

Matched population validation: 20.6393% pooled / 44.875% worst interventions; 15/16 cases fail, 13 passes and four completed controlled merges. v02 has fewer failing cases but higher pooled and worst interventions. This is the strongest tested reference for reducing intervention rates, not an accepted or universally best controller. The original four-car results use a different protocol.

See manifest.json for checksums/full checkpoint location, evaluation.json for population evidence and contract.json for the original actor contract. Future population training imports the actor only and initializes a compatible population critic; ordinary resume cannot use this selected checkpoint. The older accepted 81-field MAPPO v04r2 remains separate.
