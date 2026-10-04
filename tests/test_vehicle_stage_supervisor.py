import pytest

from experiments.verify_vehicle_stage import source_files, validate_bounds


def test_bounded_configuration():
    validate_bounds(dict(physics_hz=120, control_hz=60, repeats=10, max_process_seconds=300))


@pytest.mark.parametrize('key,value', [('physics_hz', 60), ('control_hz', 120),
    ('repeats', 11), ('repeats', 0), ('max_process_seconds', 601),
    ('max_process_seconds', float('nan')), ('max_process_seconds', True)])
def test_reject_unbounded_configuration(key, value):
    config = dict(physics_hz=120, control_hz=60, repeats=10, max_process_seconds=300)
    config[key] = value
    with pytest.raises(ValueError):
        validate_bounds(config)


def test_lab_sources_are_captured():
    files = source_files('lab')
    assert 'environments/__init__.py' in files
    assert 'traffic/physics_session.py' in files
    assert len(files) == len(set(files))
