"""Prepare the user-approved November 15 merge/cut-in round; no training."""
from copy import deepcopy
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def build():
    from traffic.highway_merge_cutin_v07 import MIX, SETTINGS, VERSION
    cfg = json.loads((ROOT / 'configs/training/highway-flow-rl-v06.json').read_text())
    cfg.update(schema='highway_merge_cutin_v07', round_index=6,
        learning_profile='merge_cutin_v07_300k_20epochs', training_authorized=True,
        authorization='User approved merge/cut-in implementation and one 300k/90min round on 2026-11-15',
        publication_branch='2026-11-15-codex-merge-cutin', curriculum_mix=MIX,
        lane_admission_settings=SETTINGS, lane_admission_version=VERSION,
        seed=9911, evaluation_interval_decisions=100000)
    # Add exactly the approved new round to the existing study ceiling. The
    # append-only ledger and its active lock are retained, not reset.
    cfg['budget'].update(total_decisions=4930000, total_wall_seconds=46800)
    rows = cfg['training_rows']
    for row in rows:
        task = row['case']['task']
        row['curriculum_group'] = ('missed_gap' if row.get('missed_gap') else 'normal_merge') if task == 'merge' else (
            'passing' if task == 'passing' else 'cruise_following')
        row['cutin_stress'] = False
    for row in list(rows):
        if row.get('missed_gap'):
            stress = deepcopy(row)
            stress.update(curriculum_group='cutin_recovery', cutin_stress=True)
            stress['case']['id'] += '-cutin'
            stress['seed'] += 200000
            rows.append(stress)
    cfg['coverage_note'] = 'Actual agent-decision deficit scheduling, full episodes: 30% normal merge, 20% missed gap, 10% cut-in, 25% cruise/following, 15% passing; finite-episode overshoot reported.'
    # Stress evaluation is reported separately and retains all failures. It is
    # never used to loosen nominal safety gates or the raw-jerk requirement.
    for source in [r for r in cfg['validation_rows'] if r['case']['task'] == 'merge' and 'ramp-networks' in r['case']['network']][:2]:
        row = deepcopy(source)
        row.update(cutin_stress=True)
        row['case']['id'] += '-cutin-stress'
        row['seed'] += 300000
        cfg['validation_rows'].append(row)
    # Compare merge quality to the same-environment initial baseline, rather
    # than a hard-coded value measured under different background admission.
    cfg['stop_rules']['merge_speed_match_reference'] = 'matched_initial_validation'
    cfg['recovery_preflight_row'] = deepcopy(next(r for r in rows if r['curriculum_group'] == 'normal_merge' and r['controlled_count'] == 1))
    cfg['mask_version'] += '_headway_ttc_v07'
    return cfg


if __name__ == '__main__':
    import sys
    sys.path.insert(0, str(ROOT))
    path = ROOT / 'configs/training/highway-merge-cutin-2026-11-15.json'
    path.write_text(json.dumps(build(), indent=2) + '\n', encoding='utf-8')
    print(path)
