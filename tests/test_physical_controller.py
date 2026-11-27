"""The low-level controller against a simple kinematic car (no physics engine, no simulator)."""
import hashlib
import math
import unittest
from dataclasses import replace
from types import SimpleNamespace

from traffic.physical_controller import (CONTROLLERS, GRAVITY, ControllerGains, LowLevelController, PathGeometry,
                                         VehicleSpec, controller_gains)

CAR = VehicleSpec(mass_kg=1800., wheelbase_m=3., front_bumper_m=2.408, rear_axle_m=1.5, wheel_radius_m=.4017,
                  max_drive_nm_per_wheel=700., max_brake_nm_per_wheel=1500.)


class KinematicCar:
    """Rear-axle kinematic bicycle with a point-mass longitudinal model and a hidden resistance."""

    def __init__(self, x, y, yaw, speed, resistance_n=0., grade=0.):
        self.rx, self.ry, self.yaw, self.speed = x - 1.5 * math.cos(yaw), y - 1.5 * math.sin(yaw), yaw, speed
        self.resistance_n, self.grade = resistance_n, grade

    def state(self):
        return dict(x=self.rx + 1.5 * math.cos(self.yaw), y=self.ry + 1.5 * math.sin(self.yaw), yaw=self.yaw,
                    speed=self.speed, grade=self.grade)

    def bumper(self):
        s = self.state()
        return s['x'] + CAR.front_bumper_m * math.cos(self.yaw), s['y'] + CAR.front_bumper_m * math.sin(self.yaw)

    def step(self, control, dt):
        force = (2 * control['drive_nm'] - 4 * control['brake_nm'] * (self.speed > 0)) / CAR.wheel_radius_m
        force -= self.resistance_n + CAR.mass_kg * 9.81 * self.grade
        self.speed = max(0., self.speed + force / CAR.mass_kg * dt)
        self.rx += self.speed * math.cos(self.yaw) * dt
        self.ry += self.speed * math.sin(self.yaw) * dt
        self.yaw += self.speed * math.tan(control['steer_rad']) / CAR.wheelbase_m * dt


def straight(y=0., length=200., start=0.):
    return [(start + s, y) for s in range(0, int(length) + 1, 2)]


def arc(radius, start_angle, length):
    return [(radius * math.cos(start_angle + s / radius), radius * math.sin(start_angle + s / radius))
            for s in range(0, int(length) + 1, 2)]


def drive(car, controller, path_for, acceleration_for, seconds, dt=1 / 120.):
    log, t = [], 0.
    for decision in range(int(round(seconds / .1))):
        controller.begin_decision(time=t, acceleration=acceleration_for(t), path=path_for(car), speed=car.speed)
        for _ in range(12):
            control = controller.command(car.state(), t, dt)
            car.step(control, dt)
            t += dt
        log.append(dict(time=t, control=control, bumper=car.bumper(), speed=car.speed))
    return log


class PathGeometryTests(unittest.TestCase):
    def test_projection_and_curvature(self):
        path = PathGeometry(arc(300., 0., 120.))
        s, left, _ = path.project(299., 30.)
        self.assertAlmostEqual(left, 300. - math.hypot(299., 30.), places=1)   # inside the left-turning arc
        self.assertAlmostEqual(path.curvature(s, 6.), 1 / 300., places=4)
        self.assertAlmostEqual(PathGeometry(straight()).curvature(20., 6.), 0.)

    def test_points_past_either_end_project_onto_the_line(self):
        path = PathGeometry(straight(length=20.))
        self.assertAlmostEqual(path.project(-3., 1.)[0], -3.)
        self.assertAlmostEqual(path.project(23., -1.)[1], -1.)


class LateralTests(unittest.TestCase):
    def test_settles_onto_a_lane_from_half_a_metre_at_highway_speed(self):
        car = KinematicCar(0., .5, 0., 25.)
        log = drive(car, LowLevelController(CAR), lambda c: straight(start=c.bumper()[0] - 4.), lambda t: 0., 6.)
        late = [abs(row['bumper'][1]) for row in log if row['time'] > 4.]
        self.assertLess(max(late), .05)
        self.assertLess(min(row['bumper'][1] for row in log), .05)     # no large overshoot to the other side

    def test_follows_a_curve_without_steady_error(self):
        radius = 250.
        car = KinematicCar(radius, 0., math.pi / 2, 25.)
        log = drive(car, LowLevelController(CAR), lambda c: arc(radius, math.atan2(c.bumper()[1], c.bumper()[0]) - .02, 140.),
                    lambda t: 0., 8.)
        errors = [abs(math.hypot(*row['bumper']) - radius) for row in log if row['time'] > 5.]
        self.assertLess(max(errors), .05)


class LongitudinalTests(unittest.TestCase):
    def test_learns_unmodelled_resistance_and_tracks_the_command(self):
        car = KinematicCar(0., 0., 0., 15., resistance_n=400., grade=.05)
        controller = LowLevelController(CAR)
        drive(car, controller, lambda c: straight(start=c.bumper()[0] - 4.), lambda t: 1., 4.)
        late = [abs(h['commanded'] - h['measured']) for h in controller.history if h['time'] > 2.]
        self.assertLess(max(late), .05)

    def test_reports_saturation_beyond_the_torque_limit(self):
        car = KinematicCar(0., 0., 0., 15., grade=.05)
        controller = LowLevelController(CAR)
        log = drive(car, controller, lambda c: straight(start=c.bumper()[0] - 4.), lambda t: 2., 1.)
        self.assertTrue(all(row['control']['saturated'] for row in log))
        self.assertTrue(all(row['control']['drive_nm'] == CAR.max_drive_nm_per_wheel for row in log))

    def test_holds_the_brake_when_stopped_with_no_positive_command(self):
        car = KinematicCar(0., 0., 0., 0.)
        controller = LowLevelController(CAR)
        controller.begin_decision(time=0., acceleration=0., path=straight(), speed=0.)
        control = controller.command(car.state(), 0., 1 / 120.)
        self.assertTrue(control['hold'])
        self.assertEqual(control['brake_nm'], CAR.max_brake_nm_per_wheel)


class RoadCar(KinematicCar):
    """The kinematic car with the road model's slope in its state, as the physics server gives it to c1 and c2."""

    def state(self):
        return dict(super().state(), road_grade=self.grade)


def fingerprint(controller, car):
    """Every tick's wheel commands over 8 s of varying grade and command, with a grade step and a command jump."""
    rows, t, dt = [], 0., 1 / 120.
    for k in range(80):
        car.grade = .04 * math.sin(k / 9.) + (.03 if 30 <= k < 34 else 0.)
        a = 1.2 * math.sin(k / 7.) + (2.5 if 50 <= k < 53 else 0.)
        controller.begin_decision(time=t, acceleration=a, path=straight(start=car.bumper()[0] - 4.), speed=car.speed)
        for _ in range(12):
            c = controller.command(car.state(), t, dt)
            car.step(c, dt)
            t += dt
            rows.append(repr((c['steer_rad'], c['drive_nm'], c['brake_nm'], c['bias_n'], c['force_n'], c['saturated'])))
    return hashlib.sha256('\n'.join(rows).encode()).hexdigest(), car.speed


# The fingerprint of the controller at commit ec714ca (the one every physics drive before decision 11 used),
# measured on Windows, Python 3.11.9, before the controller versions were added.
STAGE5_FINGERPRINT = ('9286c7d30bcc4ad2770b4934c10552dc610acf1e15c7b50685ff8a1f72d9c1c5', 12.782909855486613)
AMERICANO = replace(CAR, wheels=4, wheel_moi_kg_m2=1.225)     # the v14 package's wheel moment of inertia


class ControllerVersionTests(unittest.TestCase):
    def test_the_default_and_stage5_are_the_unchanged_controller(self):
        self.assertEqual(CONTROLLERS['stage5'], ControllerGains())
        for controller in (LowLevelController(CAR), LowLevelController(CAR, controller_gains('stage5')),
                           LowLevelController(AMERICANO, CONTROLLERS['stage5'])):
            self.assertEqual(fingerprint(controller, KinematicCar(0., .3, 0., 12., resistance_n=300.)),
                             STAGE5_FINGERPRINT)

    def test_versions_are_as_approved(self):
        c1, c2 = CONTROLLERS['c1'], CONTROLLERS['c2']
        self.assertEqual(sorted(CONTROLLERS), ['c1', 'c2', 'stage5'])
        self.assertEqual((c1.grade_source, c1.grade_preview_s, c1.bias_pause_slope_change, c1.bias_pause_hold_s,
                          c1.wheel_inertia_feedforward, c1.speed_kp), ('road', .05, .005, .5, True, 2.))
        self.assertEqual(c2, replace(c1, speed_kp=6.))
        self.assertEqual(replace(c1, grade_source='pitch', grade_preview_s=0., bias_pause_slope_change=None,
                                 bias_pause_hold_s=0., wheel_inertia_feedforward=False), ControllerGains())
        with self.assertRaises(ValueError):
            controller_gains('c3')

    def test_feedforward_mass_includes_the_wheels_inertia(self):
        self.assertAlmostEqual(AMERICANO.feedforward_mass_kg(True), 1800. + 4 * 1.225 / .4017 ** 2, places=9)
        package = replace(AMERICANO, wheel_radius_m=.401728)           # the package's wheel radius
        self.assertAlmostEqual(package.feedforward_mass_kg(True), 1830.36, places=2)
        self.assertEqual(AMERICANO.feedforward_mass_kg(False), 1800.)

    def test_c1_takes_the_grade_from_the_road_model_not_the_pitch(self):
        controller = LowLevelController(AMERICANO, CONTROLLERS['c1'])
        controller.begin_decision(time=0., acceleration=.5, path=straight(), speed=15.)
        state = dict(KinematicCar(0., 0., 0., 15.).state(), grade=.1, road_grade=.02)
        control = controller.command(state, 0., 1 / 120.)
        self.assertEqual(control['grade_used'], .02)
        self.assertAlmostEqual(control['force_n'], AMERICANO.feedforward_mass_kg(True) * .5 + 1800. * GRAVITY * .02,
                               places=6)
        stage5 = LowLevelController(AMERICANO)
        stage5.begin_decision(time=0., acceleration=.5, path=straight(), speed=15.)
        self.assertAlmostEqual(stage5.command(state, 0., 1 / 120.)['force_n'], 1800. * (.5 + GRAVITY * .1), places=6)

    def test_road_versions_need_the_road_grade(self):
        for name in ('c1', 'c2'):
            controller = LowLevelController(AMERICANO, CONTROLLERS[name])
            controller.begin_decision(time=0., acceleration=0., path=straight(), speed=15.)
            with self.assertRaises(RuntimeError):
                controller.command(KinematicCar(0., 0., 0., 15.).state(), 0., 1 / 120.)

    def test_c1_does_not_learn_the_bias_across_a_slope_break_and_for_half_a_second_after(self):
        car = RoadCar(0., 0., 0., 15., resistance_n=400., grade=.02)
        controller = LowLevelController(AMERICANO, CONTROLLERS['c1'])
        t, dt, bias = 0., 1 / 120., []
        for k in range(40):
            controller.begin_decision(time=t, acceleration=.3, path=straight(start=car.bumper()[0] - 4.),
                                      speed=car.speed)
            bias.append(controller.bias_n)
            for i in range(12):
                car.grade = .02 if (k, i) < (20, 6) else .05       # a slope break inside decision 20 (2.0-2.1 s)
                car.step(controller.command(car.state(), t, dt), dt)
                t += dt
        paused = {round(h['time'], 6) for h in controller.history if h['bias_paused']}
        # decision 20 (with the break) and those starting before 2.1 s + 0.5 s are not learned from
        self.assertEqual(paused, {round(.1 * k, 6) for k in range(20, 26)})
        self.assertTrue(all(abs(bias[k + 1] - bias[k]) < 1e-9 for k in range(20, 26)))
        self.assertGreater(abs(bias[27] - bias[26]), 1e-6)                 # learning resumes
        late = [abs(h['commanded'] - h['measured']) for h in controller.history if h['time'] > 3.]
        self.assertLess(max(late), .05)

    def test_versions_track_the_command_with_unmodelled_resistance(self):
        for name in ('c1', 'c2'):
            car = RoadCar(0., 0., 0., 15., resistance_n=400., grade=.05)
            controller = LowLevelController(AMERICANO, CONTROLLERS[name])
            drive(car, controller, lambda c: straight(start=c.bumper()[0] - 4.), lambda t: 1., 4.)
            late = [abs(h['commanded'] - h['measured']) for h in controller.history if h['time'] > 2.]
            self.assertLess(max(late), .05, name)
            self.assertFalse(any(h['bias_paused'] for h in controller.history), name)

    def test_c2_corrects_a_speed_error_faster_than_c1(self):
        errors = {}
        for name in ('c1', 'c2'):
            car = RoadCar(0., 0., 0., 15., resistance_n=800.)
            controller = LowLevelController(AMERICANO, CONTROLLERS[name])
            controller.begin_decision(time=0., acceleration=0., path=straight(), speed=15.)
            for k in range(12):
                car.step(controller.command(car.state(), k / 120., 1 / 120.), 1 / 120.)
            errors[name] = abs(car.speed - 15.)
        self.assertLess(errors['c2'], errors['c1'])

    def test_describe_names_the_changes(self):
        self.assertNotIn('versions', LowLevelController(AMERICANO).describe()['law'])
        law = LowLevelController(AMERICANO, CONTROLLERS['c1']).describe()['law']
        for text in ('road model under the axles, 0.05 s ahead', 'more than 0.005, nor for 0.5 s after', '1830.4 kg'):
            self.assertIn(text, law)


class ServerRoadGradeTests(unittest.TestCase):
    """scripts/physics_drive_server.Server.road_grade on a synthetic lane surface (no Isaac Sim)."""

    @staticmethod
    def server(name, profile):
        from scripts.physics_drive_server import Server, Surface
        server = Server.__new__(Server)
        lane = [(float(x), 0., profile(float(x))) for x in range(0, 201)]
        server.surface = Surface({'lane': dict(points=lane)})
        server.gains = CONTROLLERS[name]
        server.axles = dict(front_x_m=1.50005, rear_x_m=-1.50005)
        return server

    def test_constant_slope(self):
        server = self.server('c1', lambda x: .03 * x)
        grade = server.road_grade(dict(x=50., y=0., yaw=0., speed=20.))
        self.assertAlmostEqual(grade, .03 / math.hypot(1., .03), places=9)

    def test_takes_the_slope_ahead_of_the_axles(self):
        server = self.server('c1', lambda x: 0. if x <= 100. else .05 * (x - 100.))
        # axles at 97.5 and 100.5 m now; 0.05 s ahead at 20 m/s they are at 98.5 and 101.5 m
        grade = server.road_grade(dict(x=99., y=0., yaw=0., speed=20.))
        self.assertAlmostEqual(grade, .075 / math.hypot(3.0001, .075), places=4)
        stopped = server.road_grade(dict(x=99., y=0., yaw=0., speed=0.))
        self.assertAlmostEqual(stopped, .025 / math.hypot(3.0001, .025), places=4)

    def test_follows_the_heading(self):
        server = self.server('c1', lambda x: .03 * x)
        backwards = server.road_grade(dict(x=50., y=0., yaw=math.pi, speed=0.))
        self.assertAlmostEqual(backwards, -.03 / math.hypot(1., .03), places=9)


if __name__ == '__main__':
    unittest.main()
