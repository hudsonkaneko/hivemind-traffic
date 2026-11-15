"""Write the resolved v06 training configuration (training_authorized stays false).

Derived from the v05 pilot configuration:
- merge rows move to the ramp-started layouts (historical ramp layouts are
  reference only); a 5-lane 240 m layout is added;
- 3 of 12 merge rows (25%) become one-AV missed-gap rows; their demand is
  pre-built for the first 10 curriculum cycles to prove every seed is buildable;
- validation = the Phase 0c baseline rows (six development cases with the
  ramp-started ramp120, plus the two ramp-started validation layouts);
- the merge speed-match stop floor is the frozen-v04 value on those same rows,
  computed from the saved Phase 0c reports.
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
from traffic import highway_flow_rl_v06_rewards as v06  # noqa: E402
from traffic.highway_flow_contract import file_digest  # noqa: E402
from traffic.highway_merge_context import Stage1LaneGraph  # noqa: E402
from traffic.highway_v05_demand import write_demand  # noqa: E402

RAMP = contract.RAMP_STARTED_PREFIX
FOCUSED_TO_MISSED_GAP = {  # pilot row id -> ramp-started layout for its missed-gap replacement
    'train-2-lane-ramp120-merge-focused-6': 'train-2-lane-ramp120',
    'train-2-lane-ramp120-merge-focused-10': 'train-5-lane-ramp240',
    'train-3-lane-ramp240-merge-focused-11': 'train-3-lane-ramp240',
}
CYCLES_CHECKED = 10


def merge_speed_match(reports):
    values = []
    for report in reports:
        quality = report.get('quality_merges') or {}
        values += [(quality.get(a) or {}).get('speed_match_fraction', 0.) or 0. for a in report.get('merge_target_ids') or []]
    return sum(values) / len(values)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--project-root', type=Path, required=True, help='checkout holding outputs/ (main checkout)')
    parser.add_argument('--pilot', type=Path, default=HERE / 'configs/training/highway-flow-rl-v05-pilot.json')
    parser.add_argument('--output', type=Path, default=HERE / 'configs/training/highway-flow-rl-v06.json')
    args = parser.parse_args()
    root = args.project_root
    pilot = json.loads(args.pilot.read_text(encoding='utf-8'))
    ramp_cfg = json.loads((root / 'outputs/highway-flow-rl/v05/ramp-native-fixes-01-config.json').read_text())
    ramp_rows = {r['case']['id'].removeprefix('ramp-eval-').removesuffix('-ramp-started'): r for r in ramp_cfg['validation_rows']}

    def ramp_case(layout, case_id):
        case = deepcopy(ramp_rows[layout]['case'])
        case['id'] = case_id
        assert file_digest(root / case['network']) == case['network_sha256']
        return case

    training = []
    for row in pilot['training_rows']:
        row = deepcopy(row)
        net = row['case']['network']
        if row['case']['id'] in FOCUSED_TO_MISSED_GAP:
            layout = FOCUSED_TO_MISSED_GAP[row['case']['id']]
            base = ramp_rows[layout]
            row.update(case=ramp_case(layout, f'{layout}-missed-gap'), missed_gap=True, controlled_count=1,
                       total_vehicles=8, speed_limit_m_s=base['speed_limit_m_s'], horizon_steps=1600,
                       rollout_decisions=1600, difficulty=1, communication='local', packet_loss=.1)
        elif net.startswith('outputs/highway-environment/v05/network-bank/train-'):
            layout = net.split('/')[-2]
            row['case'] = ramp_case(layout, row['case']['id'] + '-ramp-started')
        training.append(row)

    validation = deepcopy(pilot['validation_rows'])
    swap = next(i for i, r in enumerate(validation) if r['case']['id'] == 'v02-development-train-2-lane-ramp120-merge')
    validation[swap] = deepcopy(ramp_rows['train-2-lane-ramp120'])
    validation += [deepcopy(ramp_rows['validation-2-lane-ramp180']), deepcopy(ramp_rows['validation-4-lane-ramp180'])]

    held_out = deepcopy(pilot['held_out_rows'])
    for row in held_out:
        if row['case']['network'].startswith('outputs/highway-environment/v05/network-bank/train-2-lane-ramp120'):
            row['case'] = ramp_case('train-2-lane-ramp120', row['case']['id'] + '-ramp-started')

    # Stop floor: frozen-v04 merge speed matching on exactly these validation rows (Phase 0c reports).
    reports = {}
    for run in ('dev-native-fixes-01', 'ramp-native-fixes-01'):
        for path in sorted((root / 'outputs/highway-flow-rl/v05' / run / 'evaluation').glob('[0-9][0-9].json')):
            report = json.loads(path.read_text())
            reports[(report['case_id'], report['seed'])] = report
    baseline = [reports[(r['case']['id'], r['seed'])] for r in validation]
    floor = round(merge_speed_match(baseline), 4)

    # Every missed-gap seed the curriculum will visit must be buildable.
    seed_check = []
    for index, row in enumerate(training):
        if not row.get('missed_gap'):
            continue
        graph = Stage1LaneGraph(root / row['case']['network'], row['case']['through_lanes'])
        for cycle, critic in [(c, 0) for c in range(CYCLES_CHECKED)] + [(0, 50000)]:  # PPO cycles + critic warmup
            seed = row['seed'] + 1000 * cycle + 10000 * (contract.ROUND_INDEX - 1) + critic
            with tempfile.TemporaryDirectory() as tmp:
                manifest = write_demand(Path(tmp) / 'r.xml', seed, graph, 'merge', 1, 8,
                    speed_limit_m_s=row['speed_limit_m_s'], difficulty=row['difficulty'],
                    horizon_seconds=row['horizon_steps'] * .1, controlled_car_follow_model='Krauss', missed_gap=True)
            seed_check.append(dict(row=index, seed=seed, blocker=manifest.scenario['missed_gap']['blocker_slot']))

    cfg = deepcopy(pilot)
    for key in ('variant', 'continuity_settings', 'v05_reward_weights', 'initializer_selection', 'budget_note'):
        cfg.pop(key, None)
    cfg.update(schema=contract.SCHEMA, training_authorized=False, round_index=contract.ROUND_INDEX, seed=9811,
        learning_profile='merge_v06_300k_20epochs', reward_version=v06.REWARD_VERSION,
        observation_version=v06.OBSERVATION_VERSION, v06_reward_weights=dict(v06.WEIGHTS),
        v06_settings=dict(v06.SETTINGS), rate_limit='full', controlled_car_follow_model='Krauss',
        road_end_leader_exit_rule=False, budget=dict(contract.BUDGET), acceptance=dict(contract.ACCEPTANCE),
        warm_start=dict(contract.WARM_START), critic_warmup_decisions=20000,
        stop_rules=dict(any_collision=True, merge_speed_match_floor=floor,
            merge_floor_source='frozen v04 (startup, Krauss) on these validation rows; Phase 0c reports',
            checked_at='every periodic validation (100k, 200k decisions)'),
        training_rows=training, validation_rows=validation, held_out_rows=held_out,
        held_out_authorized=False, development_gates_passed=False,
        provenance=dict(parent_policy_sha256=contract.WARM_START['sha256'], parent_run='v04 long-round-03 best',
            changes=['v06 merge objective', 'faded actor merge inputs', 'whole-episode rate limit',
                     'Krauss controlled vehicles', 'ramp-started merge layouts', 'missed-gap rows', 'critic warmup',
                     'stop rules']),
        missed_gap_seed_check=dict(cycles=CYCLES_CHECKED, built=len(seed_check), rows=seed_check),
        authorization='Prepared for user review; training_authorized=false until the user approves this configuration',
        publication_branch='highway-jerk-diagnosis-v05',
        restart_policy='Exact v06 source/config/checkpoint resume; interrupted simulator episode is discarded and charged. No automatic objective changes.')
    contract.validate_config(cfg)
    args.output.write_text(json.dumps(cfg, indent=2) + '\n', encoding='utf-8')
    merges = [r for r in training if r['case']['task'] == 'merge']
    print(json.dumps(dict(training_rows=len(training), merge_rows=len(merges),
        missed_gap_rows=sum(r.get('missed_gap', False) for r in merges),
        rollout_decisions_per_cycle=sum(r['rollout_decisions'] for r in training),
        validation_rows=len(validation), held_out_rows=len(held_out), stop_floor=floor,
        missed_gap_seeds_built=len(seed_check)), indent=1))


if __name__ == '__main__':
    main()
