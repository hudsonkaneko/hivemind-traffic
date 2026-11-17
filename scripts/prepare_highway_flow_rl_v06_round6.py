"""Write a resolved v06 round-6/7/8 configuration (training_authorized stays false).

`--round 7` runs the round-6 profile: round index 6 was already charged in the
shared ledger by the v07 cut-in run. `--round 8` writes the proposed final round
(see contract.ROUNDS[8]): raw-request jerk scored after a fixed 5-decision
insertion lag, KL target 0.1, and the merge-speed floor reported rather than
enforced at periodic validation. `--round 9` (the default) is that same profile
under the shared study ceilings: round index 8 was charged by the v08
merge-recovery run.

Derived from the committed round-5 configuration
(configs/training/highway-flow-rl-v06.json). Rows, objective, observation,
rate limit, safety, warm start and budget are unchanged. Round 6 changes only
what the user chose after the round-5 diagnosis ("Fix scoring + updates",
November 15, 2026):
- raw-request jerk is scored from each car's second decision (SUMO inserts each
  car at 0 m/s2); the every-decision result is still reported beside it;
- 4 complete episodes per PPO update, KL target 0.03, 10 epochs;
- stop rules: merge speed matching may not fall more than 0.02 below the
  starting policy's round-5 initial validation in this same environment
  (0.4367), and periodic-validation pooled interventions may not exceed the 5%
  per-case gate value.
Round index 6 shifts every training seed; every missed-gap seed the first 10
curriculum cycles visit is rebuilt to prove it is buildable.
No simulation and no ledger charge.
"""
import argparse
import json
import sys
import tempfile
from copy import deepcopy
from pathlib import Path

HERE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HERE))
from traffic import highway_flow_rl_v06_contract as contract  # noqa: E402
from traffic.highway_merge_context import Stage1LaneGraph  # noqa: E402
from traffic.highway_v05_demand import write_demand  # noqa: E402

DEFAULT_ROUND = 9
# Curriculum cycles whose missed-gap seeds must build. Rounds 6/7 checked 10.
# One cycle is the 20 training rows (101,600 decisions), so the 300,000-decision
# round cap reaches PPO cycles 0-2 and critic cycle 0; round 8 checks 0-3.
# (Row 16, ramp240 missed-gap, seed 9177010 at round-8 cycle 7 does not build;
# no round can reach it.)
CYCLES_CHECKED = {6: 10, 7: 10, 8: 4, 9: 4}
ROUND5_INITIAL_MERGE_SPEED_MATCH = .4367  # documentation/evidence/highway-merge-training-v06/round-05-results.json
FLOOR_MARGIN = .02


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--project-root', type=Path, required=True, help='checkout holding outputs/ (main checkout)')
    parser.add_argument('--round5', type=Path, default=HERE / 'configs/training/highway-flow-rl-v06.json')
    parser.add_argument('--round', type=int, default=DEFAULT_ROUND, choices=(6, 7, 8, 9))
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    ROUND = args.round
    output = args.output or HERE / f'configs/training/highway-flow-rl-v06-round{ROUND}.json'
    cfg = json.loads(args.round5.read_text(encoding='utf-8'))
    contract.validate_config(cfg)
    profile = contract.ROUNDS[ROUND]

    seed_check = []
    for index, row in enumerate(cfg['training_rows']):
        if not row.get('missed_gap'):
            continue
        graph = Stage1LaneGraph(args.project_root / row['case']['network'], row['case']['through_lanes'])
        for cycle, critic in [(c, 0) for c in range(CYCLES_CHECKED[ROUND])] + [(0, 50000)]:
            seed = row['seed'] + 1000 * cycle + 10000 * (ROUND - 1) + critic
            with tempfile.TemporaryDirectory() as tmp:
                manifest = write_demand(Path(tmp) / 'r.xml', seed, graph, 'merge', 1, 8,
                    speed_limit_m_s=row['speed_limit_m_s'], difficulty=row['difficulty'],
                    horizon_seconds=row['horizon_steps'] * .1, controlled_car_follow_model='Krauss', missed_gap=True)
            seed_check.append(dict(row=index, seed=seed, blocker=manifest.scenario['missed_gap']['blocker_slot']))

    stop_rules = dict(any_collision=True,
        merge_speed_match_floor=round(ROUND5_INITIAL_MERGE_SPEED_MATCH - FLOOR_MARGIN, 4),
        merge_floor_source=('round-5 initial validation of the same v04 actor in the v06 environment '
                            f'({ROUND5_INITIAL_MERGE_SPEED_MATCH}) minus {FLOOR_MARGIN}'),
        max_pooled_intervention_fraction=.05,
        intervention_source='per-case development gate value (5%); round 5 reached 9.5-10.5%',
        checked_at='every periodic validation (100k, 200k decisions)')
    provenance = dict(parent_policy_sha256=contract.WARM_START['sha256'], parent_run='v04 long-round-03 best',
        previous_round='v06 round 5 (no improvement; best validation was this same warm start)',
        changes=['raw-request jerk scored from the second decision', '4 complete episodes per PPO update',
                 'KL target 0.03, 10 epochs', 'pooled-intervention stop rule',
                 'merge floor from the round-5 initial validation'])
    if ROUND in (8, 9):
        stop_rules.update(merge_floor_action='report',
            merge_floor_action_source=('final round: the remaining study budget (under 300k decisions) cannot fund '
                                       'another round, so stopping early saves nothing usable; the floor result is '
                                       'still reported, and checkpoint selection keeps the starting policy unless a '
                                       'validation ranks better'),
            checked_at='periodic validation at 100k decisions; collision and pooled-intervention rules stop the run')
        provenance.update(previous_round=('v06 round 7 (round-6 profile; stopped at 100k by the merge floor after one '
                                          'ramp180 car failed its post-merge window on speed-limit trims; best '
                                          'validation was this same warm start)'),
            changes=['raw-request jerk scored after a fixed 5-decision insertion lag (round-7 validations: lag up to 4 decisions)',
                     '4 complete episodes per PPO update, 10 epochs (as round 7)',
                     'KL target 0.1, scaled to the warm start action std 0.04 (round 7: every update KL-stopped at 0.03)',
                     'merge-speed floor reported, not enforced, at periodic validation',
                     'pooled-intervention and collision stop rules unchanged'])
    cfg.update(round_index=ROUND, training_authorized=False, learning_profile=profile['learning_profile'],
        optimizer=deepcopy(profile['optimizer']), raw_request_scope=profile['raw_request_scope'],
        acceptance=deepcopy(profile['acceptance']), stop_rules=stop_rules, provenance=provenance,
        budget=deepcopy(profile.get('budget', contract.BUDGET)),
        missed_gap_seed_check=dict(cycles=CYCLES_CHECKED[ROUND], built=len(seed_check), rows=seed_check),
        authorization='Prepared for user review; training_authorized=false until the user approves this configuration')
    contract.validate_config(cfg)
    output.write_text(json.dumps(cfg, indent=2) + '\n', encoding='utf-8')
    print(json.dumps(dict(round_index=ROUND, optimizer=cfg['optimizer'], raw_request_scope=cfg['raw_request_scope'],
        stop_rules={k: v for k, v in cfg['stop_rules'].items() if not k.endswith('source')},
        missed_gap_seeds_built=len(seed_check)), indent=1))


if __name__ == '__main__':
    main()
