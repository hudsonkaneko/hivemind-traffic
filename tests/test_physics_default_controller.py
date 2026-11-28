"""Decision 17 (plan v15, November 27, 2026): C2 is the physics server's default low-level controller; 'stage5' stays
selectable and keeps its frozen gains (its fingerprint test is tests/test_physical_controller.py). No Isaac Sim."""
import inspect
from dataclasses import asdict

from scripts import physics_drive_server as server
from traffic.physical_controller import CONTROLLERS, ControllerGains


def test_c2_is_the_servers_default_and_stage5_stays_selectable():
    assert server.DEFAULT_CONTROLLER == 'c2' and set(CONTROLLERS) == {'stage5', 'c1', 'c2'}
    assert "parser.add_argument('--controller', default=DEFAULT_CONTROLLER, choices=sorted(CONTROLLERS))" in \
        inspect.getsource(server.main)
    assert CONTROLLERS['stage5'] == ControllerGains()                     # Stage 5's gains, unchanged
    assert asdict(CONTROLLERS['c2']) != asdict(CONTROLLERS['stage5'])


def test_the_physics_gap_runner_names_its_controller():
    from experiments import v15_physics_gap as gap
    assert "extra=('--controller', controller)" in inspect.getsource(gap._remote)
