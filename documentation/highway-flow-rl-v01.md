# Flow117 teacher-free MAPPO pilot — November 8, 2026

User authorized revising the plan, training, reviewing only terminal results,
then adjusting/restarting within a bounded study. This separate experimental
pilot supersedes the imitation launch for this task; it does not declare the
failed flow reference qualified or modify its original disabled configuration.

## Revised learning contract

- Shared MAPPO actor:117 inputs; centralized training critic:2,832 inputs.
  Explicit checksum-pinned migration from the earlier v05 initializer, selected
  in advance for this exploratory study; no new comparative selection claim.
- Native SUMO motion and three-second lane changes; 10 Hz decisions; normalized
  acceleration maps to −4.5…+2 m/s²; masked right/stay/left; 60 mph/lower limit.
- Remove the teacher-alignment reward, imitation loss and near-limit teacher
  auxiliary. Keep progress, native merge/pass/arrival, collision, intervention,
  analytical unsafe-request, measured jerk, lane and cooperation terms.
- No GPUDrive weights, continuous-pose adapter, comfort filter or steering port.
  Native route/link execution and existing stable-merge metrics remain enforced.
- Round1 jerk coefficient .01, actor/critic LR1e-4/3e-4; PPOclip .2, four epochs,
  batch256, entropy .001, global gradient clip .5, target KL .02. No teacher labels.
- Mixed predeclared cruise/follow/pass/merge cases, including I-110, with varied
  speeds, 8/16/24 total cars, controlled counts, messages and packet loss. Critic
  warmup has an independent cursor so PPO starts with the I-110 merge case.

## Cost, review and restart

At most two sequential rounds,30,000 active decisions each,1,500 learning-wall
seconds each. Round1 includes up to2,048 critic-warmup decisions. The existing
5,400-second shared ceiling retains1,018.6849924 seconds already spent and900
seconds for final evaluation; all new preflight/evaluation/failure costs persist
in `outputs/highway-flow-rl/v01/cost-ledger.json`. Unit tests/development are
separate from simulation/training wall accounting, as in the parent study.

Four fixed development validation rows (cruise, passing, I-110 following/merge)
run before/after each round. These are not held-out qualification. No live result
inspection while learning runs. After terminal review, round2 may raise the
jerk coefficient to .03 if measured peak jerk exceeds5 m/s³ and lower actor LR
to5e-5 if KL or intervention regression warrants it. A changed objective uses a
pinned model warm start and fresh optimizer, explicitly not an exact resume.
Unchanged interrupted jobs require matching config/source and charged resume.

The runner preserves per-episode reports, raw native journals, gradient/update
metrics, coverage, initial/final evaluation, actor export, checkpoint/RNG state,
source snapshots and failed attempts. An interrupted update is rolled back to
the pre-update model/optimizer. No automatic budget extension or policy promotion.

## Preparation evidence

Environment/native tests passed; zero-update migration/gradient and live guard
checks are recorded separately. First runtime preflight caught unsupported total
population12/20 before learning; corrected to the existing8/16/24 contract and
added validation. Failed evidence remains charged. Preflight02 passed; the final
source revision requires a refreshed source-bound proof before launch.

[Round1 config](../configs/training/highway-flow-rl-v01.json).
Initial state of this note: implementation prepared; results appended only after
completion/failure. Google engineering-guide incremental sync pending.
