import copy
import json
from pathlib import Path
import unittest

from traffic.highway_flow_rl_v02_contract import validate_config, validate_preflight, PREFLIGHT_CHECKS

ROOT = Path(__file__).resolve().parents[1]


class ContractTests(unittest.TestCase):
    def setUp(self):
        self.cfg = json.loads((ROOT/'configs/training/highway-flow-rl-v02.json').read_text())

    def test_prepared_config_cannot_launch_training(self):
        validate_config(self.cfg)
        self.assertFalse(self.cfg['training_authorized'])
        with self.assertRaisesRegex(ValueError,'disabled'):
            validate_config(self.cfg,require_training=True)

    def test_old_budget_is_not_reopened(self):
        for key,value in [('total_wall_seconds',5400),('inherited_spent_seconds',1018.6849924),('rollout_decisions',30000)]:
            cfg=copy.deepcopy(self.cfg);cfg['budget'][key]=value
            with self.assertRaises(ValueError):validate_config(cfg)

    def test_safety_and_abi_cannot_silently_change(self):
        for mutation in [lambda c:c['safety'].update(speed_mode=0),
                         lambda c:c.update(observation_size=99),
                         lambda c:c.update(mask_version='new_hidden_mask'),
                         lambda c:c['warm_start'].update(sha256='0'*64)]:
            cfg=copy.deepcopy(self.cfg);mutation(cfg)
            with self.assertRaises(ValueError):validate_config(cfg)

    def test_held_out_overlap_and_unsupported_population_rejected(self):
        cfg=copy.deepcopy(self.cfg);cfg['held_out_rows'][0]['seed']=cfg['training_rows'][0]['seed']
        with self.assertRaises(ValueError):validate_config(cfg)
        cfg=copy.deepcopy(self.cfg);cfg['training_rows'][0]['total_vehicles']=20
        with self.assertRaises(ValueError):validate_config(cfg)

    def test_second_round_requires_separate_flag(self):
        self.cfg.update(training_authorized=True,round_index=2)
        with self.assertRaisesRegex(ValueError,'second round'):
            validate_config(self.cfg,require_training=True)

    def test_stale_or_updated_preflight_is_rejected(self):
        proof=dict(schema='highway_flow_rl_preflight_v02',passed=True,training=False,optimizer_steps=0,
                   config_sha256='a',source_hashes={'runner':'b'},checks=dict.fromkeys(PREFLIGHT_CHECKS,True))
        validate_preflight(proof,'a',{'runner':'b'})
        for key,value in [('optimizer_steps',1),('config_sha256','other'),('source_hashes',{})]:
            changed=copy.deepcopy(proof);changed[key]=value
            with self.assertRaises(ValueError):validate_preflight(changed,'a',{'runner':'b'})


if __name__=='__main__':unittest.main()
