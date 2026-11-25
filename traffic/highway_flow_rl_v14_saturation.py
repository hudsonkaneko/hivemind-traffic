"""V14 drive saturation (round 24): a request clipped only by the grade-limited drive is not an intervention.

User decision, November 25, 2026: "All three" (round 24 tests multi-seed validation, a cut-in charge
and a decision on whether a request above the grade limit is an intervention; documentation/
highway-merge-training-plan-v14.md, "After round 23: diagnosis").

Measured after round 23 (outputs/test-runs/case9-probe-01/-02): the round-16 starting policy completes
case 9's stable merges only when a constant from -0.15 to 0 m/s^2 is added to the merging cars'
requests. From +0.05 up the cars ask for more than the car can give up the grade; traffic/road_grade.py's
drive bound clips the request, the clip counts as an intervention, and the merges fail on intervention
fraction. Recounting those runs without grade-only clips left 0-0.9% other interventions.

The rule: a decision whose override has drive_grade as its only cause is applied exactly as before
(clipped to the drive ceiling: full drive on that grade), but is not counted as an intervention: not in
the intervention reward term, the override counts, the per-car and merge intervention fractions or the
merge-quality samples. It is counted as a drive saturation instead (v14_drive_limit.saturated). A clip
with any other cause as well (the comfort rate limit, a following-gap or speed bound, a lane refusal, a
native change) is an intervention as before. v05's unsafe_request term still charges the request above
the ceiling, 0.5 * excess / 4.5 per decision, so asking far past the drive keeps a small, graded cost.
"""

SETTINGS = dict(intervention=False, causes=('drive_grade',))


def validate_settings(settings):
    settings = dict(settings)
    if 'causes' in settings:
        settings['causes'] = tuple(settings['causes'])
    if settings != SETTINGS:
        raise ValueError('Unreviewed v14 drive-saturation settings')
    return settings


def saturated(info, settings=SETTINGS):
    """True for an override whose causes are only the drive bound."""
    causes = set(info.get('override_causes') or ())
    return bool(info.get('override')) and bool(causes) and causes <= set(settings['causes'])
