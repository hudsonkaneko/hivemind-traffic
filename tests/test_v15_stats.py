"""Wilson intervals, Newcombe's paired interval and the exact McNemar test (traffic/v15_stats.py)."""
import random

import pytest

from traffic import v15_stats as stats


def test_wilson_known_values():
    assert stats.wilson(5, 10) == pytest.approx((0.236593, 0.763407), abs=1e-6)
    assert stats.wilson(0, 10) == pytest.approx((0., 0.277533), abs=1e-6)
    assert stats.wilson(10, 10) == pytest.approx((0.722467, 1.), abs=1e-6)
    assert stats.wilson(0, 0) == (0., 1.)
    with pytest.raises(ValueError):
        stats.wilson(11, 10)
    assert stats.rate([True, False, True, True]) == dict(passed=3, rows=4, rate=.75, **dict(zip(('low', 'high'), stats.wilson(3, 4))))


def test_paired_table_difference_and_symmetry():
    first = [True] * 6 + [True] * 9 + [False] * 2 + [False] * 43       # a=6, b=9, c=2, d=43
    second = [True] * 6 + [False] * 9 + [True] * 2 + [False] * 43
    result = stats.paired(first, second)
    assert (result['both'], result['first_only'], result['second_only'], result['neither']) == (6, 9, 2, 43)
    assert result['difference'] == pytest.approx(7 / 60)
    assert result['low'] < result['difference'] < result['high'] and result['low'] > 0
    swapped = stats.paired(second, first)
    assert (swapped['low'], swapped['high']) == pytest.approx((-result['high'], -result['low']))
    same = stats.paired([True, False] * 10, [True, False] * 10)
    assert same['difference'] == 0 and same['low'] < 0 < same['high'] and same['mcnemar_p'] == 1.
    with pytest.raises(ValueError):
        stats.paired([True], [True, False])


def test_mcnemar_exact():
    assert stats.mcnemar_exact(0, 0) == 1.
    assert stats.mcnemar_exact(0, 5) == pytest.approx(2 / 32)
    assert stats.mcnemar_exact(9, 2) == pytest.approx(2 * sum(__import__('math').comb(11, i) for i in range(3)) / 2 ** 11)
    assert stats.mcnemar_exact(4, 4) == 1.


@pytest.mark.parametrize('cells,n,lowest,highest', [((.35, .15, .05, .45), 60, .93, .97), ((.6, .05, .1, .25), 70, .93, .97),
                                                   ((.05, .03, .01, .91), 60, .92, 1.), ((.4, .05, .05, .5), 60, .92, 1.)])
def test_newcombe_interval_covers_about_95_percent(cells, n, lowest, highest):
    """About 95% where the drivers disagree often; conservative (wider) where disagreements are rare."""
    rng = random.Random(1234)
    truth = cells[1] - cells[2]
    edges = [sum(cells[:i + 1]) for i in range(4)]
    covered, trials = 0, 2000
    for _ in range(trials):
        counts = [0, 0, 0, 0]
        for _ in range(n):
            u = rng.random()
            counts[next(i for i, edge in enumerate(edges) if u < edge or i == 3)] += 1
        low, high = stats.newcombe_paired(*counts)
        covered += low <= truth <= high
    assert lowest <= covered / trials <= highest


def test_agreeing_drivers_do_not_get_a_zero_width_interval():
    for a, d in ((10, 10), (30, 30), (59, 1), (1, 59)):
        low, high = stats.newcombe_paired(a, 0, 0, d)
        assert low < 0 < high
