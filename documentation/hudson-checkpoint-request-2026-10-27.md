# What Sam should request from Hudson

Prepared 27 October 2026. This is a draft for Sam; no message has been sent.

> Could you share the frozen TorchScript `policy.pt` for your reported SUMO MAPPO
> v02 candidate, run `train-20261026T190003Z-9d133a8c`, preferably as a GitHub
> release asset? We recovered the matching source/config from the repository,
> but the weights are absent. Please confirm the file checksum, training source
> commit, resolved configuration and inference/evaluation command with runtime
> versions. If available, include per-seed held-out results and action journals.
> Also let us know if a newer accepted checkpoint supersedes it, and its supported
> lane layout and observation/action contract. We can reproduce the original
> three-lane case first, then evaluate an explicit I-110 adaptation. No new
> training is needed just to send the existing actor.

Minimum missing artifact:

- Recorded path: `outputs/sumo_mappo/v01/train-20261026T190003Z-9d133a8c/policy.pt`
  (the report says v02 although the stored run path contains v01).
- Expected SHA256: `6ec2167aa2c769ec41c9221b77382fe74658430ce852703ce32568d089e591fd`.
- Report commit: `d12c168c72c3a7441fb274af3f483de1609dac19`.
- Training source: `c06e45eaa79698eda85d01a65fa6ae5a6811b6d7`.

We already recovered all 14 recorded source hashes and the normalized config
digest. Asking him to confirm these avoids requesting files he must recreate.
Optimizer checkpoints (`best.pt`/`latest.pt`) are optional and needed for resuming
training, not for frozen inference. A different checkpoint needs its own manifest,
hashes and evaluation evidence; do not substitute it under the old identity.

The known actor takes 27 observations and emits a scalar longitudinal action in
[-1, 1], mapped to [-4.5, 2.0] m/s². It was built for three lanes and ideal simulator
state. The I-110 graph interface has 30 features and additional topology/masks;
the old weights cannot be assumed compatible. There is no existing exported-actor
standalone evaluation CLI in the recovered training runner, so the exact command
or original evaluation script is particularly useful.

See the [artifact audit](i110-sumo-checkpoint-2026-10-27.md) and
[current interface/ACC evidence](i110-acc-interface-2026-10-27.md).
