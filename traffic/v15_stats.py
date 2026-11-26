"""Pass-rate intervals and paired differences for the v15 benchmark report (plan v15, "The v15 benchmark").

- **Pass rate:** k of n rows passed, with the 95% Wilson score interval.
- **Paired difference** between two drivers on the same rows (same case, seed and background):
  the difference in pass rates, (b - c) / n, with Newcombe's (1998) hybrid score interval for paired
  proportions: the two Wilson intervals (no continuity correction) combined through the phi correlation
  of the 2 x 2 table, with ad - bc reduced by n / 2 when positive (Newcombe's corrected phi). Plus the
  exact two-sided McNemar test on the discordant rows.
  a = both pass, b = only the first passes, c = only the second passes, d = neither passes.

Caveat for the report: rows of one case share its network and traffic setting and differ only in seed,
so pooled intervals that treat the 70 rows as independent are narrower than the case structure
justifies. Per-case intervals (5 rows each) are wide.
"""
from __future__ import annotations

import math

Z95 = 1.959963984540054   # the 97.5th percentile of the standard normal


def wilson(k, n, z=Z95):
    """The Wilson score interval for k successes in n trials, as (low, high)."""
    if n <= 0:
        return (0., 1.)
    if not 0 <= k <= n:
        raise ValueError('Need 0 <= k <= n')
    p = k / n
    denominator = 1 + z * z / n
    centre = (p + z * z / (2 * n)) / denominator
    half = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / denominator
    return (max(0., centre - half), min(1., centre + half))


def rate(passes, z=Z95):
    """A list of booleans as k, n, the pass rate and its Wilson interval."""
    passes = [bool(p) for p in passes]
    k, n = sum(passes), len(passes)
    low, high = wilson(k, n, z)
    return dict(passed=k, rows=n, rate=k / n if n else None, low=low, high=high)


def table(first, second):
    """The paired 2 x 2 counts of two equal-length lists of booleans."""
    if len(first) != len(second):
        raise ValueError('Paired lists must have the same rows')
    a = sum(1 for x, y in zip(first, second) if x and y)
    b = sum(1 for x, y in zip(first, second) if x and not y)
    c = sum(1 for x, y in zip(first, second) if not x and y)
    d = sum(1 for x, y in zip(first, second) if not x and not y)
    return a, b, c, d


def newcombe_paired(a, b, c, d, z=Z95):
    """Newcombe's hybrid score interval for the paired difference p1 - p2 = (b - c) / n, as (low, high)."""
    n = a + b + c + d
    if n == 0:
        return (-1., 1.)
    p1, p2 = (a + b) / n, (a + c) / n
    l1, u1 = wilson(a + b, n, z)
    l2, u2 = wilson(a + c, n, z)
    denominator = (a + b) * (c + d) * (a + c) * (b + d)
    numerator = a * d - b * c
    if numerator > 0:
        # Newcombe's corrected phi: without it, two drivers agreeing on every row (b = c = 0) get a
        # zero-width interval at a pass rate of one half.
        numerator = max(numerator - n / 2, 0.)
    phi = numerator / math.sqrt(denominator) if denominator > 0 else 0.
    difference = p1 - p2
    lower = difference - math.sqrt(max(0., (p1 - l1) ** 2 - 2 * phi * (p1 - l1) * (u2 - p2) + (u2 - p2) ** 2))
    upper = difference + math.sqrt(max(0., (u1 - p1) ** 2 - 2 * phi * (u1 - p1) * (p2 - l2) + (p2 - l2) ** 2))
    return (max(-1., lower), min(1., upper))


def mcnemar_exact(b, c):
    """The exact two-sided McNemar p-value: a binomial test of b against b + c at one half."""
    m = b + c
    if m == 0:
        return 1.
    tail = sum(math.comb(m, i) for i in range(min(b, c) + 1)) / 2 ** m
    return min(1., 2 * tail)


def paired(first, second, z=Z95):
    """Two drivers' pass/fail on the same rows: counts, rates, difference, interval and McNemar p."""
    a, b, c, d = table(first, second)
    n = a + b + c + d
    low, high = newcombe_paired(a, b, c, d, z)
    return dict(rows=n, both=a, first_only=b, second_only=c, neither=d,
                first_rate=(a + b) / n if n else None, second_rate=(a + c) / n if n else None,
                difference=(b - c) / n if n else None, low=low, high=high, mcnemar_p=mcnemar_exact(b, c))
