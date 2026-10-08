"""One explicitly stepped PhysX scene with separately scheduled RTX rendering.

Importing this module does not start Isaac Sim. Construct the session only after
SimulationApp starts, then author a vehicle/scene/sensors before ``start()``.
This is deliberately separate from the stopped-timeline ``PhysicsSession`` used
by the original headless evidence probes. Do not mix either with Isaac Lab.

The Isaac Sim 6 SimulationManager owns the simulation clock and publishes it to
Fabric. ``RenderingManager.render`` pumps Kit with automatic physics disabled.
Every operation checks the native physics counter, making a hidden extra step
an error rather than an unrecorded change to the experiment.
"""
from __future__ import annotations

import gc
import json
import math
import time


_PLAY_SIMULATIONS = '/app/player/playSimulations'


def _positive_rate(value, name):
    if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
        raise ValueError(f'{name} must be a positive integer')
    return value


def verify_clock_transition(before, after, *, steps, dt):
    """Validate native ``(step_count, simulation_seconds)`` observations.

    Counts must be exact. A small time tolerance accommodates the installed
    PhysX float timestep; it is far smaller than one tick. No elapsed simulation
    time is permitted during a render-only update.
    """
    before_count, before_time = before
    after_count, after_time = after
    if not all(math.isfinite(value) for value in (before_time, after_time)):
        raise RuntimeError('SimulationManager returned non-finite simulation time')
    if after_count - before_count != steps:
        raise RuntimeError(
            f'Unexpected physics advancement: expected {steps} steps, '
            f'observed {after_count - before_count}')
    expected = steps * dt
    tolerance = 1e-12 if steps == 0 else max(1e-7, abs(expected) * 1e-7)
    actual = after_time - before_time
    if not math.isclose(actual, expected, rel_tol=0, abs_tol=tolerance):
        raise RuntimeError(
            f'Unexpected simulation-clock advancement: expected {expected:.12g}s, '
            f'observed {actual:.12g}s')


class RenderedPhysicsSession:
    """Fresh, CPU-PhysX stage at fixed physics/render rates; one lifecycle only.

    Example (inside Isaac Sim)::

        session = RenderedPhysicsSession(physics_hz=120, render_hz=30)
        vehicle = PhysxVehicle(session.stage, physics_hz=120)
        # Author road and sensors here; apply a brake command before start.
        session.start()
        for tick in range(960):
            # Apply wheel commands here. Never write the vehicle pose.
            session.step()
            if session.physics_steps % session.render_every_steps == 0:
                session.render()
        vehicle = None  # Also release sensors, contacts, prims, and stage handles.
        session.close()

    ``start`` includes the installed manager's physics initialization/warm-up.
    Its absolute time/count are recorded separately. ``time_s`` and
    ``physics_steps`` are measured from that recorded post-initialization origin.
    LiDAR timestamps must be compared with ``absolute_time_s`` (after validating
    the sensor's clock domain), not blindly with zero-based episode time.

    The timeline remains playing for sensors. Pause a demo by withholding calls
    to ``step``, not by stopping/restarting the timeline. ``render`` may be called
    while application-level paused, but cannot manufacture a new sensor time.
    Callers must never call ``SimulationApp.update`` after ``start``.
    """

    _active = None

    def __init__(self, physics_hz=120, render_hz=30, *, physics_scene_path='/World/PhysicsScene'):
        self.physics_scene_path = physics_scene_path
        self.physics_hz = _positive_rate(physics_hz, 'physics_hz')
        self.render_hz = _positive_rate(render_hz, 'render_hz')
        if physics_hz % render_hz:
            raise ValueError('render_hz must divide physics_hz exactly')
        from traffic.physics_session import PhysicsSession
        if RenderedPhysicsSession._active is not None or PhysicsSession._active is not None:
            raise RuntimeError('Another physics session owns the default USD context')

        import carb
        import omni.kit.app
        import omni.physx
        import omni.timeline
        import omni.usd
        import isaacsim.core.experimental.utils.app as app_utils
        from isaacsim.core.rendering_manager import RenderingManager
        from isaacsim.core.simulation_manager import SimulationManager
        from pxr import UsdUtils

        self.dt = 1.0 / physics_hz
        self.render_every_steps = physics_hz // render_hz
        self.app = omni.kit.app.get_app()
        self.context = omni.usd.get_context()
        self.timeline = omni.timeline.get_timeline_interface()
        self.settings = carb.settings.get_settings()
        self.app_utils = app_utils
        self.manager = SimulationManager
        self.renderer = RenderingManager
        # Exposed only for contact subscription/querying, never for stepping.
        self.sim = omni.physx.get_physx_simulation_interface()
        self.physx = omni.physx.get_physx_interface()
        self.cache = UsdUtils.StageCache.Get()
        self.stage = None
        self.stage_id = None
        self.started = False
        self.closed = False
        self.physics_steps = 0
        self.render_count = 0
        self.lifecycle = []
        self.startup = None
        self._origin = None
        self._last_clock = None
        self._play_setting_before = self.settings.get_as_bool(_PLAY_SIMULATIONS)
        if not self.timeline.is_stopped():
            raise RuntimeError('RenderedPhysicsSession requires a stopped timeline at construction')
        if self.manager.get_active_physics_engine() != 'physx':
            raise RuntimeError('Rendered vehicle demo requires the installed PhysX engine')

        if self.context.get_stage() is not None:
            previous_id = self.context.get_stage_id()
            self.sim.detach_stage()
            if not self.context.close_stage():
                raise RuntimeError('Could not close the startup USD context stage')
            self.app.update()  # Safe: timeline stopped and no owned physics.
            self._erase_cached(previous_id)
        if not self.context.new_stage():
            raise RuntimeError('Could not create fresh rendered-physics USD stage')
        self.app.update()  # Drain stage-open events before authoring.
        if not self.timeline.is_stopped():
            raise RuntimeError('Timeline started unexpectedly during stage setup')
        self.stage = self.context.get_stage()
        self.stage_id = self.cache.GetId(self.stage).ToLongInt()
        if not self.stage_id:
            raise RuntimeError('Fresh stage is missing from the USD stage cache')
        # Set render period before callers create Fabric histories/render products.
        self.renderer.set_dt(1.0 / render_hz)
        # CPU PhysX -> USD -> renderer. Sensor Fabric histories remain supported;
        # this only disables the PhysX-to-Fabric pose path, not RTX or Fabric time.
        self.manager.enable_fabric(False)
        RenderedPhysicsSession._active = self
        self._mark('created', stage_id=self.stage_id, physics_hz=physics_hz,
                   render_hz=render_hz, device='cpu', pose_update_path='USD')

    def _mark(self, event, **values):
        row = dict(event=event, wall_monotonic_s=time.perf_counter(), **values)
        self.lifecycle.append(row)
        print('RENDERED_PHYSICS_LIFECYCLE=' + json.dumps(row), flush=True)

    def _erase_cached(self, stage_id):
        from pxr import Usd
        cache_id = Usd.StageCache.Id.FromLongInt(stage_id)
        present = bool(self.cache.Find(cache_id))
        if present:
            self.cache.Erase(cache_id)
        return dict(old_stage_cached_before_erase=present,
                    old_stage_cached_after_erase=bool(self.cache.Find(cache_id)))

    def _clock(self):
        return (int(self.manager.get_num_physics_steps()),
                float(self.manager.get_simulation_time()))

    def _assert_owned(self, *, running):
        if self.closed or self.context.get_stage_id() != self.stage_id:
            raise RuntimeError('Rendered physics session no longer owns its stage')
        if running and (not self.started or not self.timeline.is_playing()):
            raise RuntimeError('Rendered physics session must remain started/playing')

    def _assert_current_clock(self):
        current = self._clock()
        verify_clock_transition(self._last_clock, current, steps=0, dt=self.dt)
        return current

    def start(self):
        """Initialize the authored single scene and record its warm-up origin."""
        self._assert_owned(running=False)
        if self.started or not self.timeline.is_stopped():
            raise RuntimeError('start requires a fresh stopped session')
        self.app.update()  # Authoring/scene discovery only, before committed play.
        from pxr import UsdPhysics
        authored = sorted(str(p.GetPath()) for p in self.stage.Traverse()
                          if p.IsA(UsdPhysics.Scene))
        registered = sorted(scene.path for scene in self.manager.get_physics_scenes())
        if authored != [self.physics_scene_path] or registered != authored:
            raise RuntimeError(
                f'Expected one registered physics scene {self.physics_scene_path}; '
                f'authored={authored}, registered={registered}')
        self.manager.setup_simulation(dt=self.dt, device='cpu')
        actual_dt = self.manager.get_physics_scenes()[0].get_dt()
        if not math.isclose(actual_dt, self.dt, rel_tol=0, abs_tol=1e-12):
            raise RuntimeError(f'Physics timestep mismatch: requested {self.dt}, got {actual_dt}')
        before = self._clock()
        self.settings.set_bool(_PLAY_SIMULATIONS, True)
        self.app_utils.play(commit=True)
        if not self.timeline.is_playing() or self.manager.get_physics_simulation_view() is None:
            raise RuntimeError('Committed timeline play did not initialize the physics simulation')
        self._origin = self._clock()
        if not math.isfinite(self._origin[1]):
            raise RuntimeError('Invalid simulation clock after physics initialization')
        self._last_clock = self._origin
        self.started = True
        self.physx.update_transformations(False, True, True, False)
        self.startup = dict(before_steps=before[0], before_time_s=before[1],
                            origin_steps=self._origin[0], origin_time_s=self._origin[1],
                            initialization_steps=self._origin[0] - before[0],
                            initialization_note='Installed manager warm-up precedes episode tick zero')
        self._mark('started', **self.startup)
        return self.snapshot()

    @property
    def absolute_time_s(self):
        self._assert_owned(running=True)
        return self._assert_current_clock()[1]

    @property
    def time_s(self):
        return self.absolute_time_s - self._origin[1]

    def snapshot(self):
        """Small serializable clock record; no USD or sensor objects retained."""
        absolute = self.absolute_time_s
        return dict(physics_steps=self.physics_steps, render_count=self.render_count,
                    time_s=absolute - self._origin[1], absolute_time_s=absolute,
                    native_physics_steps=self._last_clock[0], physics_hz=self.physics_hz,
                    render_hz=self.render_hz, origin_time_s=self._origin[1],
                    origin_physics_steps=self._origin[0])

    def step(self):
        """Advance precisely one physics tick, then publish the resulting USD pose."""
        self._assert_owned(running=True)
        before = self._assert_current_clock()
        self.manager.step(steps=1, update_fabric=False)
        after = self._clock()
        verify_clock_transition(before, after, steps=1, dt=self.dt)
        self.physics_steps += 1
        # Also bound accumulated clock drift, not merely each individual delta.
        expected = self.physics_steps * self.dt
        if not math.isclose(after[1] - self._origin[1], expected, rel_tol=0,
                            abs_tol=max(1e-6, expected * 1e-7)):
            raise RuntimeError('Accumulated physics clock drift exceeds the fixed-tick tolerance')
        self._last_clock = after
        self.physx.update_transformations(False, True, True, False)
        return self.snapshot()

    def render(self):
        """Pump RTX/UI with automatic simulation off, and prove no physics tick ran."""
        self._assert_owned(running=True)
        before = self._assert_current_clock()
        previous = self.settings.get_as_bool(_PLAY_SIMULATIONS)
        # The installed renderer restores this setting on normal return only;
        # protect the exception path too, without altering installed code.
        try:
            self.renderer.render()
        finally:
            self.settings.set_bool(_PLAY_SIMULATIONS, previous)
        after = self._clock()
        verify_clock_transition(before, after, steps=0, dt=self.dt)
        self._last_clock = after
        self.render_count += 1
        self._assert_owned(running=True)
        return self.snapshot()

    def close(self):
        """Stop physics and release this stage; caller first releases all prim handles."""
        if self.closed:
            return
        self._mark('closing', stage_id=self.stage_id, physics_steps=self.physics_steps,
                   render_count=self.render_count)
        self.app_utils.stop(commit=True)
        self.started = False
        self.sim.detach_stage()
        self.stage = None
        gc.collect()
        if not self.context.close_stage():
            raise RuntimeError('USD context refused to close rendered-physics stage')
        self.app.update()  # Stopped timeline; drain stage-close events.
        gc.collect()
        cache_state = self._erase_cached(self.stage_id)
        self.settings.set_bool(_PLAY_SIMULATIONS, self._play_setting_before)
        self.closed = True
        RenderedPhysicsSession._active = None
        self._mark('closed', stage_id=self.stage_id,
                   context_has_stage=self.context.get_stage() is not None, **cache_state)
        if cache_state['old_stage_cached_after_erase']:
            raise RuntimeError('Closed rendered stage remains in the stage cache')

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.close()
