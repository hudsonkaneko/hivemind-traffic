---
name: tune-highway-training
description: Diagnose a failing Highway Sim training gate and choose the one change most likely to fix it (a reward term or weight, a loss weight, a PPO or MAPPO setting, a mask rule, or a supervised fit), sized to the shared budget and judged without tuning on held-out data. Use before proposing or revising a highway flow RL round, when a round regressed or stalled, or when asked to fine-tune rewards, weights or training parameters. Not for building environments or packaging evidence.
---

# Tune Highway Training

## Highway Sim repository scope

Read the repository's `AGENTS.md` and `documentation/project-skills.md` before
applying this workflow. Before any run, read the coordination master
`TRAINING_COORDINATION.md` (in the parent of Git's common directory) and follow
its branch, worktree and run ownership. This skill describes how to decide what
to change. It does not authorize training, evaluation, held-out runs, budget
increases or dependency installation; follow the current task's authorization
and the shared cost ledger.

Tuning here means choosing **one** change that can move a **named failing
condition**, testing it at the lowest cost that can show an effect, and
reporting the result honestly. In this project, most rounds that changed a
reward or setting before finding where the failure came from moved nothing, or
regressed. The [tuning history](references/tuning-history.md) records each
round, what it changed and what it showed.

## 1. Name the failure precisely

Run the summarizer that matches the environment version (for v11,
`scripts/summarize_highway_flow_rl_v11.py`) on the validation directory. Write
the failure down as gate, condition, car, time and number:

> I-110 merge fails `rolling_merge_quality`: `agent_0` speed match 0.689
> (needs 0.90); 9 window samples have a neighbor braking at -4.5 m/s².

Not "merge quality is low". Gate thresholds are in the config's `acceptance`
and `target_settings` blocks. Also list the cases that pass: every change must
keep them passing.

## 2. Read the saved journals before simulating anything

Validation outputs keep per-decision action journals. Reading them costs no
budget. For the failing car, answer:

- **When** does the condition fail, and what did the car request compared with
  what was applied? Which filter changed it, and why?
- **Is it inside the car's control?** Check the fixed 5-decision insertion lag,
  braking by cars the policy does not control, and gate targets that drop or
  jump near a lane end.
- **Is it a timing decision or a continuous one?** When a lane change starts
  is decided by the lane outputs and the mask. How fast the car goes is decided
  by the acceleration request. They respond to different levers.

Write small read-only analysis scripts for this. Rounds 5, 9 and the v11 merge
timing finding all came from journals, not from new runs.

## 3. Classify the layer, then pick a lever that can reach it

| Symptom in the journals | Layer | Levers that can move it | Levers that cannot |
| --- | --- | --- | --- |
| Fails only in the first decisions after insertion, or on another car's braking, or on a target that jumps | Measurement or gate | A rule decision by the user; report old and new scores side by side | Any reward or training change |
| A discrete action happens at the wrong moment (merging on the first allowed decision) | Action timing or mask | A mask rule that only removes permissions; evaluate it with no training first | Reward on speed requests |
| Requests the filters keep trimming (speed-ceiling clamps, rate-limit changes) | Policy against the filter band | A supervised fit of the actor onto the band; anticipate the limit in the band | A bigger per-step penalty |
| A term is flat over the states that matter, a bonus is never paid, or a charge lands where the action cannot act | Reward shape | Reshape that term | Changing its weight |
| Updates stop early on KL, few optimizer steps, small weight change, `log_std` unchanged | Optimizer | Episodes per update, `target_kl`, epochs, learning rate; or a supervised step instead of PPO | Reward changes |
| The state that matters is rare in the training rows | Data coverage | Extra rows, using training seeds only | Weights |
| The right action depends on a quantity the actor does not observe | Observation | Add the input (shared layout: coordinate first) or keep a scripted law | Rewards or supervised labels |

Pick the cheapest lever that can reach the layer. The usual order of cost is:
journal analysis, zero-training evaluation (a rule or a scripted probe), a
short supervised fit, then a PPO round.

## 4. Check that the target is reachable

Before spending training budget:

1. **Bound it kinematically** with the real geometry: actual lane lengths,
   speed limits and full acceleration, not a short or transient target.
2. **Probe it with a script.** Replace only the failing car's acceleration with
   a simple control law and keep everything else on the policy (the pattern of
   `scripts/probe_highway_flow_rl_v11_tracking.py`). This costs diagnostic
   episodes, not training decisions.
3. **If no simple controller passes, stop tuning.** The gate or the environment
   is the problem; take it to the user.
4. **If one passes, write down what it did differently** (for example: wait
   0.7 m/s under the target, then match it). That behavior is what any training
   change must teach, and it tells you whether the actor can observe what it
   needs.

## 5. Tune reward terms and weights

- **Build a per-term budget** on the failing case from the journals: each
  term's episode total and per-decision size. If one term dominates (progress
  was about 97% of the per-decision signal in round 9), changing a small
  term's weight is noise.
- **Give every weight units.** For example, "one intervention costs as much
  as N decisions of progress". Compare a new weight with the largest term it
  competes with.
- **Fix the shape before the weight.** Look for:
  - saturation: `-0.25 * min(1, err / 2)` gives no gradient beyond 2 m/s of
    error;
  - all-or-nothing bonuses that are never paid, so they add no signal;
  - charges inside the insertion lag, where the request cannot reach the car.
- **Tie terms to the gate definition** with explicit units, and grade them
  rather than paying only on a pass.
- **Check for farming.** Make sure a policy cannot earn the new term without
  passing the gate; test with a scripted bad policy.
- **Change one term per round** and record the old value, the new value and the
  reason in the config's provenance.
- **Set expectations for the scale.** At about 8 PPO updates per 100,000
  decisions with `log_std` near -3.2, a reward change moved merge matching by
  +0.007 and 0.000 in rounds 11 and 12. A reward change alone is unlikely to
  show within one round.

## 6. Tune training parameters

Read the PPO update records before changing a hyperparameter. Look at the share
of updates stopped on KL and after how many steps, epochs completed, optimizer
steps, the value-loss trend, gradient norms against `max_grad_norm`, the
relative change in actor weights, and `log_std`.

| What the update records show | Reading | What to try (one per round) |
| --- | --- | --- |
| Most updates stop on KL inside the first epoch | Updates are too small and noisy | More complete episodes per update (1 to 4 fixed it between rounds 5 and 9); a moderately looser `target_kl` |
| Updates stop on KL after dozens of steps with KL far above target, and passing cases break | The policy moves far beyond the targeted term | Lower the actor learning rate, or replace PPO with a supervised step |
| All epochs run, small steady gains, no gate changes | Healthy but too slow for the budget | Change the lever (layer), not the hyperparameter |
| `log_std` near -3.2 (about ±0.13 m/s²) | Exploration too narrow to find a different timing | Do not rely on PPO to discover it; use a rule, a probe or a supervised target |

**Supervised projection** (`experiments/run_highway_flow_rl_v11_projection.py`)
fits the actor to a band or label you can compute, holding it near a reference
actor elsewhere. It suits "stay inside what the filters would pass".

- Run at least 2 collect-and-fit passes: the fit is made on the states the
  current actor visits, and pass 1 alone was worse in round 16.
- Round 16 used weights of 50 for the band, 1 for the preserve term and 1 for
  the lane term. The preserve term is what keeps passing cases passing.

**Teacher labels** only transfer when the actor's inputs determine the label.
In round 17 the label depended on the gate's target speed, which the actor does
not observe, and the fit stalled at 0.12 to 0.16 m/s² RMS.

Change one hyperparameter or loss weight per round, and state whether the
optimizer state is fresh or carried over.

## 7. Size the round to the budget

- **Read what is left** from the shared ledger `outputs/highway-flow-rl/v02/cost-ledger.json`
  in the canonical checkout: decisions, wall seconds and diagnostic episodes.
  Respect `active.lock`; runs cannot overlap.
- **Know the unit cost.** One curriculum cycle of 20 training rows is roughly
  100,000 decisions. 100,000 decisions is about 8 PPO updates at 4 episodes per
  update.
- **Predeclare stop rules** before launch:
  - any collision;
  - pooled interventions above 5%;
  - a merge floor taken from the starting policy's own validation in the same
    environment, minus a small margin. A floor from an older environment is
    either too lenient or trips immediately.
- **Avoid the known traps.** A round index already charged cannot be reused.
  Source files hashed by the runner must not change while a run is live.
- **Ask before raising budgets.** Raising any study budget needs a note to the
  user.

## 8. Judge the result honestly

- **Compare with the start** on the same development cases: gates passed,
  per-car conditions, pooled merge match, stable merges, interventions and peak
  jerk. Losing a passing case is a regression even if the count holds.
- **Keep the best validation**, which may be the starting policy. Record failed
  rounds; do not delete or rerun them quietly.
- **Treat one seed per case as noise near limits.** Events within about 25% of
  a limit (raw jerk 5.6 or 6.1 against 5) flipped between nearly identical
  runs. Treat a change of one gate as unconfirmed until a multi-seed check.
- **Admit that selection is tuning.** Choosing on validation cases is tuning on
  them. Held-out rows run once, after development gates pass, with user
  approval, and are never used to choose.
- **Keep seeds apart.** Collection and training seeds must never equal a
  validation or held-out seed; test it.

## Proposal template

Every tuning proposal or round declaration states:

1. **Failing condition:** gate, condition, car and number, with the evidence file.
2. **Layer and evidence:** what the journals showed and why this layer.
3. **The one change:** old value, new value, and the file and config key.
4. **Expected effect:** which number should move, by roughly how much, and how
   it will be measured.
5. **What it will not fix,** and which passing cases could break.
6. **Budget and stop rules:** decisions, wall seconds and diagnostic episodes,
   with the floors and their sources.
7. **Result:** recorded afterwards beside the declaration, including failure.
