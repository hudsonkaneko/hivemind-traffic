"""Prepare the approved November 17 bounded recovery round; never trains."""
import json
from pathlib import Path
import sys
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from traffic.highway_merge_recovery_v08 import SETTINGS, REWARD_VERSION


def build():
    cfg = json.loads((ROOT / 'configs/training/highway-merge-cutin-2026-11-15.json').read_text())
    cfg.update(schema='highway_merge_recovery_v08', round_index=8, seed=10111,
        learning_profile='merge_recovery_v08_300k_20epochs', reward_version=REWARD_VERSION,
        v08_settings=SETTINGS, initial_policy_std=.12, raw_request_scope='all',
        publication_branch='2026-11-17-codex-merge-recovery',
        authorization='User approved fixing merge regression, measured cut-ins and bounded retraining on 2026-11-17')
    cfg['optimizer'].update(actor_lr=3e-5, epochs=20, episodes_per_update=4)
    cfg['stop_rules']['max_pooled_intervention_fraction'] = .05
    for split in ('training_rows', 'validation_rows'):
        for row in cfg[split]:
            if row.get('cutin_stress'):
                row.update(missed_gap=False, total_vehicles=8, controlled_count=1)
                row['case'].update(task='cruise', id=row['case']['id'] + '-verified-onset')
                row['cutin_coverage_required'] = True
    cfg['recovery_preflight_row'] = next(r.copy() for r in cfg['training_rows'] if r.get('cutin_stress'))
    cfg['coverage_note'] += ' Cut-in recovery is now a dedicated mainline scene on the same frozen road, with measured cut-in motion required at reset; empty scenarios fail.'
    return cfg


if __name__ == '__main__':
    p = ROOT / 'configs/training/highway-merge-recovery-2026-11-17.json'
    p.write_text(json.dumps(build(), indent=2) + '\n')
    print(p)
