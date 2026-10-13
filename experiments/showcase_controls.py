"""Pure settings/command boundary for the opt-in v08 sidebar.

Future sliders are deliberately absent from the accepted settings schema.
Nothing in this module writes a chassis pose or imports Isaac.
"""
import math

MPH = .44704
# label, default, minimum, maximum, integer, units
RESTART_CONTROLS = {
    'traffic_count': ('Background cars', 80, 4, 80, True, 'cars'),
    'ego_mph': ('Main-car target', 35., 5., 35., False, 'mph'),
    'traffic_mph': ('Traffic speed (inner lane)', 15., 5., 25., False, 'mph'),
    'following_s': ('Following time gap', 1.8, 1.8, 3., False, 's'),
    'render_hz': ('Display update rate', 30, 10, 30, True, 'Hz'),
    'resolution_percent': ('Render resolution', 100, 50, 100, True, '%'),
}
LIVE_CONTROLS = {
    'camera_distance_m': ('Follow distance', 20., 8., 60., False, 'm'),
    'camera_height_m': ('Follow height', 17., 3., 40., False, 'm'),
    'path_horizon_m': ('Path preview length', 120., 20., 200., False, 'm'),
    'statistics_hz': ('Statistics refresh', 5, 1, 5, True, 'Hz'),
    'wheel_distance_m': ('Wheel animation distance', 150., 30., 200., False, 'm'),
}
# Display-only examples, never experiment settings or active outputs.
FUTURE_CONTROLS = {
    'Traffic and drivers': [
        ('Speed variation', 0, 0, 10, 'mph'), ('Traffic arrival rate', 0, 0, 100, 'cars/min'),
        ('Lane distribution bias', 0, -100, 100, '%'), ('AV share', 0, 0, 100, '%'),
        ('Communicating AV share', 0, 0, 100, '%'), ('Driver reaction delay', 1, 0, 3, 's'),
        ('Driver variability', 0, 0, 100, '%'), ('Overtaking preference', 50, 0, 100, '%')],
    'Advanced dynamics': [
        ('Acceleration limit', 3, .5, 5, 'm/s²'), ('Comfort braking limit', 3, .5, 6, 'm/s²'),
        ('Jerk limit', 2, .5, 5, 'm/s³'), ('Steering-rate limit', .5, .1, 1, 'rad/s'),
        ('Lane-change duration', 5, 2, 10, 's')],
    'LiDAR and V2V': [
        ('LiDAR-equipped cars', 0, 0, 8, 'cars'), ('LiDAR update rate', 10, 1, 30, 'Hz'),
        ('LiDAR range', 100, 10, 200, 'm'), ('LiDAR sampling density', 50, 1, 100, '%'),
        ('Measurement noise', 0, 0, 1, 'm'), ('Sensor dropout', 0, 0, 100, '%'),
        ('Communication range', 100, 10, 500, 'm'), ('Message frequency', 10, 1, 30, 'Hz'),
        ('Communication latency', 0, 0, 500, 'ms'), ('Packet loss', 0, 0, 100, '%')],
    'Future viewing': [('Camera smoothing', 0, 0, 1, 's'), ('Playback speed', 1, .25, 2, '×'),
                       ('Detailed-model distance', 100, 10, 300, 'm')],
}


def defaults():
    return {key: row[1] for key, row in {**RESTART_CONTROLS, **LIVE_CONTROLS}.items()}


def validate_settings(values):
    expected = defaults()
    if not isinstance(values, dict) or values.keys() != expected.keys():
        raise ValueError('Exactly the supported sidebar settings are required')
    for key, (_, _, low, high, integer, _) in {**RESTART_CONTROLS, **LIVE_CONTROLS}.items():
        value = values[key]
        if (type(value) not in (int, float) or not math.isfinite(value)
                or not low <= value <= high or (integer and type(value) is not int)):
            raise ValueError(f'{key} must be {"an integer" if integer else "finite"} in [{low}, {high}]')
    if values['render_hz'] not in (10, 20, 30):
        raise ValueError('Display rate must be 10, 20 or 30 Hz')
    if values['resolution_percent'] not in (50, 75, 100):
        raise ValueError('Resolution must be 50, 75 or 100 percent')
    if values['traffic_mph'] > values['ego_mph']:
        raise ValueError('Traffic speed cannot exceed the main-car target in this nonreactive showcase')
    return dict(values)


class SidebarState:
    """Bounded, latest-value requests. Simulator consumes at a frame boundary."""
    def __init__(self, values, *, live_driving=False):
        self.live_driving=live_driving
        self.live_keys=set(LIVE_CONTROLS) | (set(RESTART_CONTROLS)-{'render_hz','resolution_percent'} if live_driving else set())
        self.active = validate_settings(values)
        self.pending = dict(self.active)
        self.live_requests = {}
        self.restart_request = None
        self.events = []
        self.event_limit = 2000
        self.locked = False

    def edit(self, key, value):
        if self.locked:
            return
        if key not in self.pending:
            raise ValueError('Display-only or unknown control')
        candidate = {**self.pending, key: value}
        validate_settings(candidate)
        self.pending = candidate
        if key in self.live_keys:
            self.live_requests[key] = value

    def consume_live(self, tick):
        changes = {k: v for k, v in self.live_requests.items() if self.active[k] != v}
        self.live_requests.clear()
        if changes:
            if len(self.events) >= self.event_limit:
                raise RuntimeError('Sidebar event limit reached; start a fresh run')
            self.active.update(changes)
            self.events.append(dict(tick=tick, type='live_request' if self.live_driving else 'live_display', changes=changes))
        return changes

    def request_restart(self):
        self.restart_request = validate_settings(self.pending)
        self.locked = True

    @property
    def dirty(self):
        return any(self.pending[k] != self.active[k] for k in RESTART_CONTROLS if k not in self.live_keys)


def restart_assessment_config(config, drive_s):
    """Explicitly assess a shortened stop, never qualify the original episode."""
    if config['version'] not in ('v08','v09') or type(drive_s) is not int or not 15 <= drive_s <= config['drive_s']:
        raise ValueError('Invalid bounded restart duration')
    return {**config, 'drive_s': drive_s, 'duration_s': drive_s + config['settle_s'] + config['brake_s'],
            'minimum_passes': 0, 'minimum_lane_changes': 0}
