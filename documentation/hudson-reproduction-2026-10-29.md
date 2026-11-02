# Delivered Hudson actor: original-environment reproduction

29 October 2026 · Codex for Sam · frozen inference, no training.

Sam supplied the release's `policy.pt`, `evaluation.json` and `SHA256SUMS.txt`
during the ramp/cut-in work. The exact actor is now present and hash verified.
The original smoke/repeat and 20 fixed held-out episodes pass the original
acceptance criteria. **Strict numerical reproduction against the delivered
per-seed evaluation fails the frozen tolerance.** These are separate findings.

## Identity and execution

Release: `mappo-v02-train-20261026T190003Z-9d133a8c` at
[Hudson's repository](https://github.com/hudsonkaneko/hivemind-traffic/releases/tag/mappo-v02-train-20261026T190003Z-9d133a8c).
The user-supplied direct asset URLs and acquisition timestamps are preserved in
`outputs/hudson-delivery-20261029/received-v2/manifest.json`.

| Asset | SHA256 |
|---|---|
| policy.pt, 85,048 bytes | `6ec2167aa2c769ec41c9221b77382fe74658430ce852703ce32568d089e591fd` |
| evaluation.json, 17,374 bytes | `6cb6a70ca72c7bc7796406d178d2b6e85a0c1fa4aafd578c54caadc28d712b12` |
| SHA256SUMS.txt | `135265d1bed6db703fd3a654f3fa232f705db0e791ab5175d4652a6bb709f13b` |

Both asset hashes match the supplied checksum file; the policy also matches the
previously pinned training identity. Hash verification precedes model loading.
The first sandbox download attempt failed with WinError 10013 and is retained
in `received-v1`; the approved network retry produced `received-v2`.

The evaluator reconstructed all 14 original source files from commit
`c06e45eaa79698eda85d01a65fa6ae5a6811b6d7`, matching recorded hashes, and verified
the normalized v02 configuration. It uses the original evaluation routine with
a frozen TorchScript actor, CPU inference, one Torch thread and deterministic
execution. Source hashes stayed unchanged. The actor contract is 27 float32
features and one normalized acceleration action, mapped to [−4.5,+2] m/s².
There is no steering or communication action. No optimizer or training ran.

The smoke episode at seed 520001 was repeated before the held-out evaluation;
episode metrics matched exactly. Then seeds 410001–410020 ran once without
tuning. Actions, runtime, logs, source snapshots and evaluation files remain
in the immutable `smoke-v1` and `held-out-v1` packages.

| Original held-out metric | Hudson reference | Local result |
|---|---:|---:|
| Nominal episodes | 20 / 20 | 20 / 20 |
| Collision vehicles | 0 | 0 |
| Overspeed steps | 0 | 0 |
| External intervention fraction | 0 | 0 |
| Mean progress (m) | 1445.453448644255 | 1445.453448614148 |
| Mean team reward | 500.247340353284 | 500.247340726716 |
| Peak speed (m/s) | 26.066446626654 | 26.066446597597 |
| Original acceptance | Pass | Pass |

These are 600-step, 60-second episodes on the original three-lane, 5 km fixture.
Nominal termination is not full-route completion. Zero external interventions
does not establish zero internal SUMO safety intervention. This tranche does
not compare the actor with Sam's smooth ACC or establish traffic benefit.

## Numerical comparison and unresolved cause

The comparison plan `experiments/configs/hudson_delivered_reproduction_v1.json`
was frozen before evaluation. It checks all supplied fields, including types,
ordered seeds, every per-seed metric and the summary. Exact equality is reported
separately from absolute/relative floating tolerance of 1e−9. No tolerance was
relaxed after observing results.

`comparison-v1/comparison.json` records 153 unequal floating values, 34 outside
tolerance. All non-floating fields match. The largest absolute difference is
4.0065331745609e−6 in seed 410015's reward. Mean-progress difference is
3.010654836543836e−8 m. The comparison exits nonzero and is retained as a failed
strict-reproduction result, despite both original acceptance flags being true.

Local Python is 3.11.16 versus reported 3.11.9; local Torch is 2.14.0+cu132 versus
reported 2.14.0+cpu. CPU execution does not make those builds identical. NumPy
2.4.6, Gymnasium 1.3.0, PettingZoo 1.27.0 and SUMO/TraCI 1.27.1 match the reported
versions. Runtime differences are plausible contributors, not a proven cause.
No delivered action/state journal permits locating the first trajectory divergence.
Do not install another runtime into the working traffic or Isaac environment
merely to erase the comparison failure.

The existing evaluator's immutable result has a generic interpretation about an
unprovided reference. That sentence describes its standalone scope; the new
comparison package and this report supersede it now that the reference exists.
The evaluator itself and its original-source execution were not changed.

## Reproduce and continue

Run from the active checkout using the research root's traffic Python:

```text
python -m scripts.evaluate_hudson_actor --mode evaluate --suite smoke --checkpoint outputs/hudson-delivery-20261029/received-v2/policy.pt --output outputs/hudson-smoke-new
python -m scripts.evaluate_hudson_actor --mode evaluate --suite held-out --wall-seconds 1800 --checkpoint outputs/hudson-delivery-20261029/received-v2/policy.pt --output outputs/hudson-held-out-new
python scripts/compare_hudson_evaluation.py --reference outputs/hudson-delivery-20261029/received-v2/evaluation.json --actual outputs/hudson-held-out-new/source/outputs/evaluation/evaluation.json --output outputs/hudson-comparison-new
```

Never reuse an output directory, change held-out seeds to seek a pass, substitute
weights or bypass the checksum gate. `comparison-v1` links exact reference,
actual result, frozen plan and copied comparator hashes. The compact versioned
record is `documentation/evidence/hudson-delivery-20261029.json`.

Next, keep the strict reproduction discrepancy open while reviewing the explicit
[27-versus-30-feature semantic transfer contract](i110-transfer-and-next-tracks-2026-10-28.md).
Any closer runtime replication belongs in an isolated environment with a recorded
protocol; original action journals would support first-divergence analysis if
later supplied. The current actor has not run on I-110. Padding/truncating features
is prohibited; original-fixture success does not authorize transfer or training.
Use the [current handoff](continuation-2026-10-29.md) rather than the older
checkpoint-unavailable instructions.
