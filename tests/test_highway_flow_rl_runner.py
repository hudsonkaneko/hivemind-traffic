import unittest
from experiments.run_highway_flow_rl import summary, training_row


class RunnerTests(unittest.TestCase):
    def test_real_report_denominator_and_unaccepted_status(self):
        result=summary([dict(agent_steps=100,overrides=10,peak_abs_jerk=7.,collisions=0,teleports=0,
            stable_merge=dict(stable=1,targets=2,entered=2,lateral_completed=2),primary_horizon_complete=True)])
        self.assertEqual(result['pooled_intervention_fraction'],.1)
        self.assertEqual(result['stable_merges'],1)
        self.assertFalse(result['quality_accepted'])

    def test_critic_warmup_does_not_skip_first_merge_ppo_case(self):
        cfg=dict(round_index=1,training_rows=[dict(seed=100,case={'task':'merge'}),dict(seed=101,case={'task':'cruise'})])
        state=dict(episodes=1,critic_episodes=1,ppo_episodes=0)
        self.assertEqual(training_row(cfg,state,False)['case']['task'],'merge')
        self.assertEqual(training_row(cfg,state,False)['seed'],100)
        self.assertEqual(training_row(cfg,state,True)['seed'],50101)
        self.assertEqual(cfg['training_rows'][0]['seed'],100)


if __name__=='__main__':unittest.main()
