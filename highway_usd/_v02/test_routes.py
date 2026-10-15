"""Route randomization must produce varied, continuous, bounded return trips."""
import json
from pathlib import Path
import unittest
from route_policy import generate_plans


class Routes(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.network=json.loads(Path(__file__).with_name('navigation.json').read_text())

    def test_seed_reproduces_and_changes_choices(self):
        a=generate_plans(self.network,10,128)
        self.assertEqual(a,generate_plans(self.network,10,128))
        b=generate_plans(self.network,11,128)
        self.assertNotEqual([p['edges'] for p in a['plans']],[p['edges'] for p in b['plans']])

    def test_all_four_opportunities_rejoin_continuously(self):
        edges=self.network['edges']; seen=set()
        for plan in generate_plans(self.network,123,256,1.)['plans']:
            seen.add(plan['return_opportunity'])
            for a,b in zip(plan['edges'],plan['edges'][1:]):
                self.assertIn(b,edges[a]['successors'])
                self.assertEqual(edges[a]['points'][-1],edges[b]['points'][0])
            kinds=[edges[e]['kind'] for e in plan['edges']]
            self.assertEqual(kinds.count('exit'),1); self.assertEqual(kinds.count('return'),1)
            self.assertLessEqual(kinds.count('collector'),8); self.assertEqual(kinds[-1],'auxiliary')
        self.assertEqual(seen,{1,2,3,4})

    def test_zero_probability_stays_on_auxiliary_circle(self):
        for plan in generate_plans(self.network,77,32,0.)['plans']:
            self.assertFalse(plan['take_exit']); self.assertIsNone(plan['return_edge'])
            self.assertTrue(all(self.network['edges'][e]['kind']=='auxiliary' for e in plan['edges']))

    def test_invalid_configuration_rejected(self):
        for probability in [-1,1.1,float('nan')]:
            with self.assertRaises(ValueError): generate_plans(self.network,1,1,probability)
        with self.assertRaises(ValueError): generate_plans(self.network,True,1)
        with self.assertRaises(ValueError): generate_plans(self.network,1,0)


if __name__=='__main__': unittest.main(verbosity=2)
