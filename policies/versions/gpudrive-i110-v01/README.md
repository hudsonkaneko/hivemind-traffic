# GPUDrive pretrained actor — I-110 test v01

**External frozen PPO actor; transfer rejected; not an accepted Highway Sim policy.**

- Source: `daphne-cornelisse/policy_S10_000_02_27`, revision
  `1532950cad84dafc6e9d976a2bcc524ee481a1a1`.
- 51,228 parameters; 2,984 structured inputs; 91 acceleration/steering actions.
  No camera/LiDAR or V2V inputs, no MAPPO actor compatibility.
- SHA256: `f3f26475def35f375c6c72d8f8f20b2b091f77175010345dc3fa968a860521b7`.
- Intended upstream capability: multi-car driving learned by self-play.
  Actual local result: unshielded I-110 main-car transfer oversped and crossed
  the mapped road boundary at 13.4 s; no completed stable merge or route goal.
- Existing weights unchanged; zero training updates. No existing accepted
  checkpoint is superseded. Local-only checkpoint retrieval details and exact
  adapter limits are in the [experiment note](../../../documentation/gpudrive-i110-v01.md).
