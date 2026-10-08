"""Explicit speed envelopes; opting in does not certify real-world safety."""

MPH_TO_M_S = 0.44704
TARGET_35_MPH_M_S = 35 * MPH_TO_M_S


def speed_limit(profile):
    if profile == 'low-speed':
        return 3.0
    if profile == '35mph':
        return 16.0  # Overspeed guard, not the cruise target.
    raise ValueError('Unknown physical speed profile')
