"""Low-level controller that carries out the merge policy's commands on a physics car (no simulator).

User decision (November 19, 2026): "Yes, policy drives it". The policy and the v14 environment
decide a longitudinal acceleration and a lane every 0.1 s; this controller turns them into wheel
commands (drive torque, brake torque, steering angle) for a PhysX car at every physics tick.
It adds no behaviour of its own: it does not pick speeds, gaps or lanes.

Longitudinal: at each decision the controller records the car's measured speed v0 and follows
v0 + a_cmd * (t - t0) for that decision, with a force feedforward m * (a_cmd + g * grade) plus a
proportional speed correction. A slowly learned force bias absorbs what the feedforward does not
model (wheel damping, rolling losses); it is updated only from the error between commanded and
measured acceleration of unsaturated decisions. Positive force goes to the driven wheels as
drive torque, negative force to all four wheels as brake torque, both clipped to the vehicle's
torque limits, so a command beyond the car's ability is reported as saturation, not hidden.

Lateral: a path-tracking law on the reference path the environment builds from the SUMO lanes
(traffic/physical_drive.py), measured at the front bumper (where SUMO measures lane position):
steer = atan(wheelbase * path curvature) + heading error + atan(-k * lateral error / (v + v0))
plus an integral of the lateral error (critically damped with the Stanley term), limited in
angle and rate. The path is smoothed over a few metres (SUMO lane shapes have corners) and the
curvature feedforward is taken a short preview ahead, to lead the steering and tire lag.
In a steady turn the car's heading trails the path heading at the bumper by
atan(curvature * rear-axle-to-bumper distance) less the rear tires' slip angle, and the front
wheels need the understeer angle on top of the geometric one; both come from the car's own
mass and tire stiffness through the linear bicycle model (cornering_terms).

Pure Python (math only) so it can be unit-tested outside Isaac Sim and imported inside it.

Controller versions (plan v15 "Physics gap proposal", user decision of November 27, 2026, relayed by the main Claude
session: "Yes, as recommended", decision 11). CONTROLLERS names them; 'stage5' is the controller every physics drive
used before, and stays the default, unchanged:
- 'c1': the grade feedforward takes the road model's slope under the axles, 0.05 s ahead (state['road_grade'], which
  the physics server computes from the lane-graph surface), instead of the chassis pitch; the force bias is not
  learned from a decision in which that slope changed by more than 0.005, nor from any decision that starts within
  0.5 s after one; and the acceleration feedforward includes the wheels' rotational inertia (m + n I / r^2);
- 'c2': c1 with the speed correction gain raised from 2 to 6 per second.
Both were fixed before any drive with them; they are not tuned on drive results.
"""
from __future__ import annotations

import math
from dataclasses import asdict, dataclass, replace

GRAVITY = 9.81


@dataclass(frozen=True)
class VehicleSpec:
    """Measured geometry and declared actuator limits of one physics car."""
    mass_kg: float
    wheelbase_m: float
    front_bumper_m: float          # front bumper ahead of the chassis origin
    rear_axle_m: float             # rear axle behind the chassis origin
    wheel_radius_m: float
    max_drive_nm_per_wheel: float
    max_brake_nm_per_wheel: float
    driven_wheels: int = 2
    braked_wheels: int = 4
    max_steer_rad: float = .5
    max_steer_rate_rad_s: float = .5
    # Linear-bicycle cornering terms (rad per m/s2 of lateral acceleration), 0 for a car without tire slip:
    understeer_rad_per_m_s2: float = 0.     # m_front / C_front - m_rear / C_rear
    rear_slip_rad_per_m_s2: float = 0.      # m_rear / C_rear: the body points this much into the turn
    wheels: int = 4
    wheel_moi_kg_m2: float = 0.             # each wheel's moment of inertia (used only with wheel_inertia_feedforward)

    def feedforward_mass_kg(self, wheel_inertia):
        """The mass the acceleration feedforward multiplies: with wheel inertia, m + n I / r^2."""
        if not wheel_inertia:
            return self.mass_kg
        return self.mass_kg + self.wheels * self.wheel_moi_kg_m2 / self.wheel_radius_m ** 2


def cornering_terms(mass_kg, wheelbase_m, center_of_mass_x_m, front_axle_x_m, rear_axle_x_m,
                    front_tire_n_rad, rear_tire_n_rad):
    """Understeer gradient and rear slip coefficient of a linear bicycle model (two tires per axle)."""
    front_mass = mass_kg * (center_of_mass_x_m - rear_axle_x_m) / wheelbase_m
    rear_mass = mass_kg - front_mass
    front, rear = 2. * front_tire_n_rad, 2. * rear_tire_n_rad
    return dict(understeer_rad_per_m_s2=front_mass / front - rear_mass / rear, rear_slip_rad_per_m_s2=rear_mass / rear,
                front_axle_mass_kg=front_mass, rear_axle_mass_kg=rear_mass)


@dataclass(frozen=True)
class ControllerGains:
    speed_kp: float = 2.            # 1/s: speed error to acceleration correction
    bias_gain: float = .3           # fraction of an unsaturated decision's acceleration error learned as force bias
    bias_limit_m_s2: float = 1.     # bias force limit, as acceleration
    heading_gain: float = 1.
    lateral_gain: float = 3.        # Stanley gain on the front-bumper lateral error (about 1/s)
    lateral_soft_speed_m_s: float = 2.
    lateral_integral_limit: float = 2.   # m*s; the integral gain is lateral_gain**2 / (4 * speed): critical damping
    integral_band_m: float = .2          # integrate only near the path, not through a lane change's transient
    curvature_window_m: float = 6.
    curvature_preview_s: float = .3    # curvature feedforward taken this far ahead, to lead the steering lag
    path_smoothing_m: float = 8.       # moving-average window over the reference path (SUMO lanes are polylines)
    steady_command_m_s2: float = .2    # learn the force bias only when consecutive commands differ less than this
    stop_speed_m_s: float = .3
    # Physics gap versions (see the module docstring); these defaults are the 'stage5' controller.
    grade_source: str = 'pitch'        # 'pitch': state['grade'] (chassis); 'road': state['road_grade'] (road model)
    grade_preview_s: float = 0.        # how far ahead the server takes the road slope ('road' only)
    bias_pause_slope_change: float | None = None   # no bias learning from a decision whose road slope changed more
    bias_pause_hold_s: float = 0.      # ... nor from a decision starting within this long after such a decision
    wheel_inertia_feedforward: bool = False


_C1 = ControllerGains(grade_source='road', grade_preview_s=.05, bias_pause_slope_change=.005, bias_pause_hold_s=.5,
                      wheel_inertia_feedforward=True)
CONTROLLERS = {'stage5': ControllerGains(), 'c1': _C1, 'c2': replace(_C1, speed_kp=6.)}


def controller_gains(name):
    if name not in CONTROLLERS:
        raise ValueError(f'Unknown controller version {name!r}; one of {sorted(CONTROLLERS)}')
    return CONTROLLERS[name]


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class PathGeometry:
    """Arc length, heading and curvature of a polyline of (x, y) points, optionally smoothed.

    SUMO lane shapes are polylines with corners (up to about 3 degrees on the LA ramp), which no
    car follows exactly; a moving average over smoothing_m metres rounds them (on a 3-degree
    corner and an 8 m window the smoothed path stays within about 5 cm of the corner).
    """

    def __init__(self, points, smoothing_m=0.):
        pts = [(float(x), float(y)) for x, y in points]
        if smoothing_m > 0. and len(pts) > 2:
            step = sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(pts, pts[1:])) / (len(pts) - 1)
            half = int(round(smoothing_m / max(step, 1e-6) / 2))
            smoothed = []
            for i in range(len(pts)):
                k = min(half, i, len(pts) - 1 - i)          # shrink the window at the ends: they stay fixed
                window = pts[i - k:i + k + 1]
                smoothed.append((sum(p[0] for p in window) / len(window), sum(p[1] for p in window) / len(window)))
            pts = smoothed
        clean = [pts[0]]
        for p in pts[1:]:
            if math.hypot(p[0] - clean[-1][0], p[1] - clean[-1][1]) > 1e-6:
                clean.append(p)
        if len(clean) < 2:
            raise ValueError('A reference path needs two distinct points')
        self.points = clean
        self.s = [0.]
        self.heading = []
        for a, b in zip(clean, clean[1:]):
            self.s.append(self.s[-1] + math.hypot(b[0] - a[0], b[1] - a[1]))
            self.heading.append(math.atan2(b[1] - a[1], b[0] - a[0]))
        self.length = self.s[-1]

    def project(self, x, y):
        """(arc length, signed left offset, segment index) of the nearest path point to (x, y)."""
        best = (math.inf, 0., 0., 0)
        for i, (a, b) in enumerate(zip(self.points, self.points[1:])):
            dx, dy = b[0] - a[0], b[1] - a[1]
            seg = self.s[i + 1] - self.s[i]
            u = ((x - a[0]) * dx + (y - a[1]) * dy) / (seg * seg)
            # Extend the first and last segments so a point just past either end projects onto the line.
            u = u if (i == 0 and u < 0) or (i == len(self.heading) - 1 and u > 1) else min(1., max(0., u))
            px, py = a[0] + u * dx, a[1] + u * dy
            d = math.hypot(x - px, y - py)
            if d < best[0]:
                side = math.copysign(d, dx * (y - a[1]) - dy * (x - a[0]))
                best = (d, self.s[i] + u * seg, side, i)
        return best[1], best[2], best[3]

    def heading_at(self, s):
        for i in range(len(self.heading)):
            if s <= self.s[i + 1] or i == len(self.heading) - 1:
                return self.heading[i]

    def curvature(self, s, window):
        """Mean curvature (1/m, positive to the left) over [s - window/2, s + window/2], clipped to the path."""
        lo, hi = max(0., s - window / 2), min(self.length, s + window / 2)
        if hi - lo < 1e-6:
            return 0.
        return wrap(self.heading_at(hi) - self.heading_at(lo)) / (hi - lo)


class LowLevelController:
    """Acceleration and path tracking for one car; one instance per physics car."""

    def __init__(self, vehicle: VehicleSpec, gains: ControllerGains = ControllerGains()):
        self.vehicle, self.gains = vehicle, gains
        self.reset()

    def reset(self):
        self.decision = None
        self.bias_n = 0.
        self.integral = 0.
        self.steer = 0.
        self.saturated = False
        self.history = []
        self.pause_until = -math.inf      # no bias learning from decisions starting before this (c1, c2)
        self.last_road_grade = None

    def begin_decision(self, *, time, acceleration, path, speed):
        """Start a 0.1 s decision: the commanded acceleration and the reference path for it."""
        g, v = self.gains, self.vehicle
        previous = self.decision
        if previous is not None and time > previous['time'] + 1e-9:
            measured = (speed - previous['speed']) / (time - previous['time'])
            error = previous['acceleration'] - measured
            steady = abs(float(acceleration) - previous['acceleration']) < g.steady_command_m_s2
            paused = False
            if g.bias_pause_slope_change is not None:
                if previous['road_low'] is None:
                    raise RuntimeError('This controller version needs state["road_grade"] from the physics server')
                if previous['road_high'] - previous['road_low'] > g.bias_pause_slope_change:
                    self.pause_until = max(self.pause_until, float(time) + g.bias_pause_hold_s)
                paused = previous['time'] < self.pause_until - 1e-6     # starting exactly hold_s after: learned
            if not previous['saturated'] and previous['speed'] > g.stop_speed_m_s and steady and not paused:
                limit = g.bias_limit_m_s2 * v.mass_kg
                self.bias_n = min(limit, max(-limit, self.bias_n + g.bias_gain * v.mass_kg * error))
            self.history.append(dict(time=previous['time'], commanded=previous['acceleration'],
                                     measured=measured, saturated=previous['saturated'], bias_paused=paused))
        # The road slope's range over a decision starts from the last tick before it, so a change across the
        # boundary counts.
        last = self.last_road_grade
        self.decision = dict(time=float(time), acceleration=float(acceleration), speed=float(speed),
                             path=PathGeometry(path, g.path_smoothing_m), saturated=False,
                             road_low=last, road_high=last)

    def command(self, state, time, dt):
        """Wheel commands for one physics tick.

        state: x, y (chassis origin), yaw (rad, from +x), speed (forward, m/s), grade (sine of the
        nose-up pitch) and, for the 'road' grade source, road_grade (the road model's slope under the
        axles, taken grade_preview_s ahead by the server). Returns steering (rad, positive left), drive
        and brake torque per wheel (N*m) and diagnostics.
        """
        g, v, d = self.gains, self.vehicle, self.decision
        if d is None:
            raise RuntimeError('No decision has been given to the controller')
        speed = float(state['speed'])
        # longitudinal
        reference = d['speed'] + d['acceleration'] * (time - d['time'])
        wanted = d['acceleration'] + g.speed_kp * (reference - speed)
        road = state.get('road_grade')
        if road is not None:
            road = float(road)
            self.last_road_grade = road
            d['road_low'] = road if d['road_low'] is None else min(d['road_low'], road)
            d['road_high'] = road if d['road_high'] is None else max(d['road_high'], road)
        if g.grade_source == 'road':
            if road is None:
                raise RuntimeError('This controller version needs state["road_grade"] from the physics server')
            grade = road
        elif g.grade_source == 'pitch':
            grade = float(state['grade'])
        else:
            raise ValueError(f'Unknown grade source {g.grade_source!r}')
        if g.wheel_inertia_feedforward:
            force = v.feedforward_mass_kg(True) * wanted + v.mass_kg * GRAVITY * grade + self.bias_n
        else:
            force = v.mass_kg * (wanted + GRAVITY * grade) + self.bias_n
        hold = speed < g.stop_speed_m_s and d['acceleration'] <= 0.
        drive = brake = 0.
        if hold:
            brake = v.max_brake_nm_per_wheel
        elif force >= 0.:
            drive = force * v.wheel_radius_m / v.driven_wheels
        else:
            brake = -force * v.wheel_radius_m / v.braked_wheels
        saturated = drive > v.max_drive_nm_per_wheel or brake > v.max_brake_nm_per_wheel
        drive, brake = min(drive, v.max_drive_nm_per_wheel), min(brake, v.max_brake_nm_per_wheel)
        d['saturated'] |= saturated and not hold
        # lateral, measured at the front bumper
        yaw = float(state['yaw'])
        fx = state['x'] + v.front_bumper_m * math.cos(yaw)
        fy = state['y'] + v.front_bumper_m * math.sin(yaw)
        s, lateral, _ = d['path'].project(fx, fy)
        lead = d['path'].curvature(s + max(speed, 0.) * g.curvature_preview_s, g.curvature_window_m)
        curvature = d['path'].curvature(s, g.curvature_window_m)
        lateral_acceleration = speed * speed * curvature
        trail = math.atan(curvature * (v.front_bumper_m + v.rear_axle_m)) - v.rear_slip_rad_per_m_s2 * lateral_acceleration
        heading_error = wrap(d['path'].heading_at(s) - trail - yaw)
        if abs(lateral) < g.integral_band_m:
            self.integral = min(g.lateral_integral_limit, max(-g.lateral_integral_limit, self.integral + lateral * dt))
        moving = max(speed, 0.) + g.lateral_soft_speed_m_s
        wanted_steer = (math.atan(v.wheelbase_m * lead) + v.understeer_rad_per_m_s2 * speed * speed * lead
                        + g.heading_gain * heading_error
                        + math.atan2(-g.lateral_gain * lateral, moving)
                        - g.lateral_gain ** 2 / (4. * moving) * self.integral)
        wanted_steer = min(v.max_steer_rad, max(-v.max_steer_rad, wanted_steer))
        step = v.max_steer_rate_rad_s * dt
        self.steer = min(self.steer + step, max(self.steer - step, wanted_steer))
        return dict(steer_rad=self.steer, drive_nm=drive, brake_nm=brake, saturated=saturated, hold=hold,
                    reference_speed_m_s=reference, wanted_acceleration_m_s2=wanted, force_n=force,
                    bias_n=self.bias_n, lateral_error_m=lateral, heading_error_rad=heading_error,
                    path_curvature=curvature, path_s_m=s, grade_used=grade)

    def describe(self):
        g = self.gains
        law = ('longitudinal: per-decision speed reference v0 + a_cmd * (t - t0), force feedforward '
               'm * (a + g * grade) + learned bias + proportional speed correction; lateral: '
               'curvature feedforward + heading error + Stanley term at the front bumper + small integral')
        changes = []
        if g.grade_source == 'road':
            changes.append(f'grade from the road model under the axles, {g.grade_preview_s:g} s ahead')
        if g.bias_pause_slope_change is not None:
            changes.append(f'no bias learning from a decision whose road slope changes by more than '
                           f'{g.bias_pause_slope_change:g}, nor for {g.bias_pause_hold_s:g} s after')
        if g.wheel_inertia_feedforward:
            changes.append(f'acceleration feedforward mass m + n I / r^2 = '
                           f'{self.vehicle.feedforward_mass_kg(True):.1f} kg')
        if changes:
            law += '; versions: ' + '; '.join(changes)
        return dict(vehicle=asdict(self.vehicle), gains=asdict(g), law=law)
