"""Filesystem-only preflight. A pass is NOT a GPU/physics compatibility claim."""
from pathlib import Path

from hivemind.launcher import SetupError, isaac_runtime, require_file


def check_showcase(root, config, local):
    root = Path(root)
    runtime = isaac_runtime(root, local)
    version = require_file(runtime.parent / 'VERSION', 'Use an extracted standalone Isaac Sim installation.')
    for name in ('highway_v02.usda', 'navigation.json', 'manifest.json', 'parameters.json'):
        require_file(root / config['source_directory'] / name, 'Restore the tracked highway_usd/_v02 files.')
    profile = config.get('vehicle_profile', 'prepared')
    if profile == 'prepared':
        for name in ('world.usda', 'layout.usda', 'physics.usda', 'model.json', 'assets/vehicle.usda'):
            require_file(root / config['vehicle_directory'] / name,
                         'The prepared car is local-only. Use --vehicle public for the block-car demo; '
                         'do not download unknown vehicle packages.')
    elif profile == 'public':
        factories = [p for base in ('exts', 'extscache')
                     for p in (runtime.parent / base).glob('omni.physx.vehicle*/omni/physxvehicle/scripts/helpers/Factory.py')]
        if not factories:
            raise SetupError('Public car needs the installed omni.physx.vehicle Factory sample. '
                             'See documentation/clean-clone.md for the tested Isaac version.')
    else:
        raise SetupError('Unknown vehicle profile: ' + str(profile))
    return runtime, dict(runtime=str(runtime), isaac_version=version.read_text().strip(),
                         vehicle_profile=profile, required_files_available=True,
                         check_scope='Paths only; dependencies, GPU, composition and driving still require a runtime test')
