"""The low-level controller against a simple kinematic car (no physics engine, no simulator)."""
import math
import unittest

from traffic.physical_controller import ControllerGains, LowLevelController, PathGeometry, VehicleSpec

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


if __name__ == '__main__':
    unittest.main()
