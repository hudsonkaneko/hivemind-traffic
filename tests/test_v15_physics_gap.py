"""The v15 physics-gap runner's rules (no simulator, no Isaac Sim, nothing charged)."""
import unittest
from dataclasses import asdict

from experiments import v15_physics_gap as G
from traffic.physical_controller import CONTROLLERS
from traffic.physics_link import server_command


def result(recorded=(), rebaselined=(), flags=0, events=None, collisions=0, failure=None):
    if failure:
        return dict(failure=failure)
    return dict(failure=None, metrics=dict(flags=flags), sides=dict(physics=dict(
        recorded=dict(passed=not recorded, failures=list(recorded)),
        rebaselined=dict(passed=not rebaselined, failures=list(rebaselined)),
        events=dict(events or {}), collisions=collisions)))


PASS = result()
FAIL_RECORDED = result(recorded=['rolling_merge_quality'])
FAIL_BOTH = result(recorded=['stable_merge'], rebaselined=['stable_merge'])
SUMO = {11: dict(recorded=True, rebaselined=True), 25: dict(recorded=True, rebaselined=True),
        39: dict(recorded=False, rebaselined=True), 53: dict(recorded=False, rebaselined=True),
        67: dict(recorded=False, rebaselined=True)}


class PlanTests(unittest.TestCase):
    def test_ten_drives_in_the_approved_order(self):
        drives = G.plan()
        self.assertEqual([(d['row'], d['controller'], d['render_hz']) for d in drives],
                         [(11, 'c1', 10), (11, 'c2', 10), (25, 'stage5', 0), (25, 'pick', 0), (39, 'stage5', 0),
                          (39, 'pick', 0), (53, 'stage5', 0), (53, 'pick', 0), (67, 'stage5', 0), (67, 'pick', 0)])
        self.assertEqual([d['controller'] for d in G.plan('c2')][3::2], ['c2'] * 4)
        self.assertNotIn(11, [d['row'] for d in drives if d['controller'] == 'stage5'])   # row 11's before: Stage 5

    def test_expected_charge_from_the_sumo_decisions(self):
        decisions = {11: 462, 25: 453, 39: 436, 53: 428, 67: 443}
        drives = G.plan()
        self.assertEqual(sum(decisions[d['row']] for d in drives), 4444)
        self.assertAlmostEqual(sum(88.4 if d['render_hz'] else 70.8 for d in drives), 743.2, places=6)
        self.assertEqual((G.ALLOWANCE, G.TIME_CAP, G.PHASE, G.DRIVER), (6000, 850., '6', 'l2:bc7c050f6d3da3ce'))

    def test_only_the_approved_rows_and_versions(self):
        self.assertEqual(G.SOUTHBOUND, (25, 39, 53, 67))
        self.assertEqual(G.VERSIONS, ('c1', 'c2'))
        self.assertEqual(G.CURRENT, 'stage5')


class PickTests(unittest.TestCase):
    def test_fewer_failed_gates_over_both_sets(self):
        self.assertEqual(G.pick({'c1': FAIL_BOTH, 'c2': FAIL_RECORDED}), 'c2')
        self.assertEqual(G.pick({'c1': FAIL_RECORDED, 'c2': FAIL_BOTH}), 'c1')

    def test_then_fewer_flags_then_c1(self):
        self.assertEqual(G.pick({'c1': result(flags=9), 'c2': result(flags=3)}), 'c2')
        self.assertEqual(G.pick({'c1': result(flags=3), 'c2': result(flags=3)}), 'c1')
        self.assertEqual(G.pick({'c1': result(flags=30), 'c2': FAIL_RECORDED}), 'c1')     # gates come first

    def test_a_drive_without_a_result_ranks_last(self):
        self.assertEqual(G.pick({'c1': result(failure='ConnectionError'), 'c2': FAIL_BOTH}), 'c2')
        with self.assertRaises(G.StopAndAsk):
            G.pick({'c1': result(failure='x'), 'c2': result(failure='y')})


class StopRuleTests(unittest.TestCase):
    def test_estimate_is_the_larger_of_the_rule_and_the_longest_drive(self):
        self.assertEqual(G.estimate_seconds(10, None), 88.4)
        self.assertEqual(G.estimate_seconds(0, None), 70.8)
        self.assertEqual(G.estimate_seconds(0, 88.0), 88.0)
        self.assertEqual(G.estimate_seconds(10, 60.), 88.4)

    def test_a_drive_that_does_not_fit_stops_and_asks(self):
        G.check_fits(700., 88.4, 850., 500, 1556)
        with self.assertRaises(G.StopAndAsk):
            G.check_fits(762., 88.4, 850., 500, 1556)
        with self.assertRaises(G.StopAndAsk):
            G.check_fits(100., 70.8, 850., 500, 499)

    def test_the_measured_drives_leave_room_for_all_ten(self):
        elapsed, longest = 0., None
        for d in G.plan('c1'):
            seconds = 88.4 if d['render_hz'] else 70.8
            G.check_fits(elapsed, G.estimate_seconds(d['render_hz'], longest), 850., 462, 6000)
            elapsed, longest = elapsed + seconds, max(longest or 0., seconds)

    def test_collision_and_server_checks(self):
        self.assertTrue(G.collided(result(collisions=1)))
        self.assertFalse(G.collided(PASS))
        self.assertFalse(G.collided(result(failure='x')))
        report = dict(report=dict(controller='c1', vehicle=dict(spec=dict(gains=asdict(CONTROLLERS['c1']))),
                                  bias_paused_decisions=7))
        self.assertEqual(G._check_server(report, 'c1'), dict(controller='c1', bias_paused_decisions=7))
        with self.assertRaises(G.StopAndAsk):
            G._check_server(report, 'c2')
        with self.assertRaises(G.StopAndAsk):
            G._check_server(dict(report=dict(report['report'], controller='c2')), 'c2')

    def test_the_server_is_started_with_the_controller_version(self):
        command = server_command('out', network='n', manifest='m', render_hz=0, extra=('--controller', 'c2'))
        self.assertEqual(command[-2:], ['--controller', 'c2'])


class ReadingTests(unittest.TestCase):
    def current(self):
        return {11: FAIL_BOTH, 25: FAIL_BOTH, 39: FAIL_BOTH, 53: FAIL_RECORDED, 67: FAIL_BOTH}

    def test_closed(self):
        picked = {11: PASS, 25: PASS, 39: FAIL_RECORDED, 53: FAIL_RECORDED, 67: FAIL_RECORDED}
        verdict = G.reading(SUMO, self.current(), picked)
        self.assertTrue(verdict['row11_closed'])
        self.assertEqual(verdict['corridor'], 'closed')

    def test_a_collision_is_not_closed(self):
        picked = {11: PASS, 25: PASS, 39: FAIL_RECORDED, 53: result(recorded=['x'], collisions=1), 67: FAIL_RECORDED}
        self.assertNotEqual(G.reading(SUMO, self.current(), picked)['corridor'], 'closed')

    def test_improved_needs_more_passes_no_loss_and_no_new_event_kind(self):
        picked = {11: FAIL_BOTH, 25: FAIL_RECORDED, 39: FAIL_RECORDED, 53: FAIL_RECORDED, 67: FAIL_BOTH}
        verdict = G.reading(SUMO, self.current(), picked)
        self.assertEqual(verdict['rebaselined_passes'], dict(current=1, pick=3))
        self.assertEqual(verdict['corridor'], 'improved')
        self.assertFalse(verdict['row11_closed'])
        lost = {**picked, 53: FAIL_BOTH}                            # the current controller passes 53 re-baselined
        self.assertEqual(G.reading(SUMO, self.current(), lost)['corridor'], 'not improved')
        noisy = {**picked, 25: result(recorded=["x"], events={"override_episodes": 2})}
        verdict = G.reading(SUMO, self.current(), noisy)
        self.assertEqual((verdict['corridor'], verdict['new_event_kinds']), ('not improved', {25: ['override_episodes']}))

    def test_a_failed_pick_drive_passes_no_gate(self):
        picked = {11: PASS, 25: result(failure='physics'), 39: FAIL_RECORDED, 53: FAIL_RECORDED, 67: FAIL_RECORDED}
        verdict = G.reading(SUMO, self.current(), picked)
        self.assertEqual(verdict['corridor'], 'improved')
        self.assertEqual(verdict['rebaselined_passes']['pick'], 4)


if __name__ == '__main__':
    unittest.main()
