"""Movement checks M1-M4 and the ramp situations (traffic/v15_movement.py), on synthetic traces: no saved record is
read, so the tests don't depend on local outputs."""
import gzip
import json

from traffic import v15_movement as M

FAR = dict(bumper_gap_m=200., relative_speed_m_s=0.)


def _trace(speeds, accelerations=None, front=None):
    accelerations = accelerations or [None] * len(speeds)
    return [dict(v=v, a=a, front=front) for v, a in zip(speeds, accelerations)]


def test_the_thresholds_are_the_ones_set_in_the_write_up():
    s = M.SETTINGS
    assert (s['close_free_gap_m'], s['close_headway_s'], s['close_ttc_s']) == (20., 3., 6.)
    assert (s['m1_brake_m_s2'], s['m1_decisions'], s['m2_speed_m_s'], s['m2_limit_s']) == (2.0, 3, 5., 1.0)
    assert (s['m3_tolerance_m_s'], s['m4_swing_m_s']) == (2.0, 1.0)


def test_a_close_leader():
    assert not M.close_leader(None, 10.)
    assert M.close_leader(dict(bumper_gap_m=19., relative_speed_m_s=5.), 1.)          # within 20 m
    assert M.close_leader(dict(bumper_gap_m=50., relative_speed_m_s=0.), 20.)         # 2.5 s headway
    assert M.close_leader(dict(bumper_gap_m=50., relative_speed_m_s=-10.), 5.)        # 5 s closing time
    assert not M.close_leader(dict(bumper_gap_m=70., relative_speed_m_s=-10.), 5.)    # 7 s closing, 14 s headway


def test_m1_needs_three_hard_decisions_in_a_row_with_no_close_leader():
    speeds = [20., 19.8, 19.6, 19.4, 19.2]
    two = M.car_checks(_trace(speeds, [-2.5, -2.5, -1., -2.5, -2.5], FAR), True)
    assert two['m1_brake_run'] == 2 and not two['flags']['m1']
    three = M.car_checks(_trace(speeds, [-1., -2.0, -2.0, -2.0, 0.], FAR), True)
    assert three['m1_brake_run'] == 3 and three['flags']['m1'] and three['fails']
    close = M.car_checks(_trace(speeds, [-3.] * 5, dict(bumper_gap_m=10., relative_speed_m_s=-1.)), True)
    assert not close['flags']['m1']                       # braking behind a close leader is not flagged


def test_m2_flags_more_than_one_second_below_five_metres_per_second_on_an_open_ramp():
    assert not M.car_checks(_trace([4.] * 10 + [6.], front=FAR), True)['flags']['m2']     # 1.0 s: not more than 1 s
    crawl = M.car_checks(_trace([4.] * 11 + [6.], front=FAR), True)
    assert crawl['m2_crawl_s'] == 1.1 and crawl['flags']['m2']
    queued = M.car_checks(_trace([1.] * 30, front=dict(bumper_gap_m=8., relative_speed_m_s=0.)), True)
    assert not queued['flags']['m2']                     # crawling in a queue is not flagged


def test_m4_flags_an_accelerate_then_brake_of_one_metre_per_second():
    assert M.car_checks(_trace([10., 10.5, 10.9, 10.5, 10.]), True)['m4_reversals'] == 0
    swing = M.car_checks(_trace([10., 9., 10., 11., 10.5, 10.]), True)
    assert swing['m4_reversals'] == 1 and swing['flags']['m4']
    assert M.car_checks(_trace([5., 6., 7., 8., 9.]), True)['m4_reversals'] == 0


def test_m3_is_reported_only_where_mainline_cars_were_in_view_and_a_car_that_never_merges_fails():
    checks = M.car_checks(_trace([10., 11., 12.]), True, mainline_speed=15.)
    assert checks['m3'] == dict(merge_speed=12., mainline_speed=15., shortfall=3., passed=False)
    assert not checks['fails']                           # M3 is reported, not part of failing
    assert M.car_checks(_trace([10., 11., 12.]), True)['m3'] is None
    never = M.car_checks(_trace([10.] * 5), False, mainline_speed=15.)
    assert never['fails'] and never['m3'] is None and never['merge_speed'] is None


def test_situations():
    car = dict(ramp_leader_stopped_s=0., t_merge=5.)
    assert M.situation(car, 'x') == 'clear'
    assert M.situation(dict(car, t_merge=12.), 'x') == 'stream'
    assert M.situation(dict(car, t_merge=None), 'x') == 'stream'
    assert M.situation(dict(car, ramp_leader_stopped_s=2.), M.I110_CASE) == 'blocked'
    assert M.situation(car, M.I110_CASE) == 'i110'


def test_the_summary_compares_drivers_on_matched_cars_only():
    ok = M.car_checks(_trace([10.] * 3), True)
    bad = M.car_checks(_trace([4.] * 20, front=FAR), True)
    cars = {'a': {(1, 'agent_0'): ok, (2, 'agent_0'): bad, (3, 'agent_1'): ok},
            'b': {(1, 'agent_0'): bad, (2, 'agent_0'): bad}}
    situations = {(1, 'agent_0'): 'stream', (2, 'agent_0'): 'clear', (3, 'agent_1'): 'stream'}
    table = M.summary(cars, situations)
    assert table['all']['a']['cars'] == 2 and table['all']['b']['fails'] == 2 and table['all']['a']['fails'] == 1
    assert table['stream']['a']['fails'] == 0 and table['clear']['a']['m2'] == 1
    text = M.markdown(table, 'SUMO, post hoc')
    assert 'SUMO, post hoc' in text and '| stream (1) | a |' in text


def test_episode_cars_reads_a_saved_episode(tmp_path):
    def step(t, v, a, entry_active, merged=False, front=None, entry_front=None):
        return dict(time=t, agents=dict(agent_0=dict(
            v04_reward_audit=dict(measured_speed_m_s=v), realized_acceleration=a, merge_completed=merged,
            flow_context=dict(entry=dict(entry_active=entry_active, front=entry_front, rear=None),
                              corridors=dict(own=dict(front=front))))))
    main = dict(id='main_3', bumper_gap_m=30., relative_speed_m_s=8.)
    lines = [step(0., 12., None, True, entry_front=main), step(.1, 11.7, -3., True), step(.2, 11.4, -3., True),
             step(.3, 11.1, -3., True), step(.4, 11.5, 1., False, merged=True), step(.5, 12., 1., False)]
    with gzip.open(tmp_path / 'actions.jsonl.gz', 'wt', encoding='utf-8') as handle:
        for line in lines:
            handle.write(json.dumps(line) + '\n')
    cars = M.episode_cars(tmp_path)
    car = cars['agent_0']
    assert car['t_merge'] == .4 and car['mainline_speed'] == 20. and len(car['samples']) == 5
    assert car['checks']['m1_brake_run'] == 3 and car['checks']['flags']['m1']
    assert car['checks']['m3'] == dict(merge_speed=11.5, mainline_speed=20., shortfall=8.5, passed=False)
