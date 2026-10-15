# SUMO demo readiness results

Updated: October 6, 2026.

## Question and protocol

This bounded check asks whether the existing scripted and frozen shared-PPO
controllers can complete the current two-car curved SUMO scenario in headless
mode. It evaluates both policies on the preselected matched seeds 2001, 2002,
and 2003, then repeats seed 2001 once per policy. Every episode uses the same
400-step horizon, scenario, deterministic policy mode, and safety shield. The
checkpoint is the trusted local 16-observation checkpoint selected by
`hivemind.local.json`; no training is run.

The readiness gates were fixed before execution: both cars must complete every
episode, collisions must remain at zero, and all recorded numeric metrics must
be finite. Shield overrides are recorded as behavior to inspect; zero overrides
is not a gate. This small run is a readiness check, not a performance or
superiority claim.

## Result

Evidence run: `demo-sumo-readiness-20261006T085028Z-9e077dff` under
`outputs/demo_sumo_readiness/`. SUMO 1.27.1, Python 3.11.9, and the configured
TraCI package were used on Windows. The scenario's recorded simulation step is
0.2 seconds; the horizon is 400 steps. Source and scenario snapshots,
checkpoint hashes, dirty-tree metadata, per-episode logs, raw traces, and
derived tables are in the run package. Each child rollout has a 120-second
wall-time limit and captures stdout and stderr. The checkpoint itself and
private `hivemind.local.json` are not copied.

| Policy | Episodes (including repeat) | Both cars completed | Collisions | Shield overrides on seeds 2001/2002/2003 | Mean wall time |
| --- | ---: | ---: | ---: | ---: | ---: |
| Scripted | 4 | 4/4 | 0 | 0 / 0 / 0 | 2.158 s |
| Trained | 4 | 4/4 | 0 | 29 / 39 / 43 | 2.296 s |

All declared gates passed for both policies. The repeated seed-2001 metric
fingerprint matched exactly for each policy, and source/checkpoint hashes were
unchanged during the run. The scripted episodes took 269
environment steps each; the trained episodes took 243, 249, and 244 steps on
the three distinct seeds. The trained controller triggered safety overrides,
especially for `agent_1`, so its shield activity remains relevant when judging
the behavior. These results do not establish that either policy is better.

The paired per-seed rows are in `comparison.csv`; `episode-metrics.csv` and
`metrics.csv` retain episode summaries and raw step traces. Each episode has a
stdout/stderr log in `logs/`. The package snapshots source and scenario inputs
and records their start and end hashes alongside the checkpoint hash.

## Reproduce the readiness run

From the repository root, using the configured traffic environment and local
checkpoint:

```text
.venv-traffic/Scripts/python.exe -m experiments.demo_sumo_readiness --headless
```

On a POSIX machine with the traffic environment at `.venv-traffic/bin/python`:

```text
.venv-traffic/bin/python -m experiments.demo_sumo_readiness --headless
```

Each invocation creates a new directory under `outputs/demo_sumo_readiness/`.
For another trusted checkpoint, add `--checkpoint PATH`; the local configuration
remains the default. To change the bounded horizon, use `--horizon N`.

## GUI launch commands

The readiness runner deliberately uses headless SUMO. The existing portable
launcher is the supported GUI entry point: `python -m hivemind demo sumo` opens
the scripted rollout in SUMO GUI, and `python -m hivemind demo trained` opens
the configured checkpoint in SUMO GUI. Add `--seed 2001 --horizon 400` to either
command to match one readiness episode; the trained command uses the trusted
checkpoint in `hivemind.local.json`, or accepts `--checkpoint PATH`. Add
`--headless` to either launcher command to suppress the SUMO window. See
`documentation/portable-demos.md` for environment discovery and cross-platform
setup. GUI timing is for visual inspection and is not comparable to the
headless wall-time measurements above.

## Limits

The separate GUI rehearsal also completed successfully for both policies on
seed 2001, horizon 400 and 20-ms GUI delay. Evidence is retained in
`results/rollouts/scripted/seed-2001-20261006T085101Z` and
`results/rollouts/trained/seed-2001-20261006T085109Z`. This checks window startup
and completion; it is not a GUI throughput benchmark. Earlier readiness
packages are preserved, including `20261006T083745Z-b41440b8`; the final run
additionally snapshots the portable launcher source. Local configuration
contents remain private and are not copied; resolved paths and commands are
recorded instead.

This is three distinct seeds plus one repeat per policy, on one SUMO version,
one scenario, and one local checkpoint. It does not evaluate other traffic
loads, policies, checkpoints, platforms, or GUI timing. A passing readiness
gate is not a collision-rate estimate for general traffic and does not validate
physical vehicle behavior.
