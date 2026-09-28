"""Portable runtime discovery; no user-specific checkout paths."""
import os
from pathlib import Path
import shutil

ROOT = Path(__file__).resolve().parents[1]


def find_sumo():
    candidates = [os.environ.get('SUMO_BINARY'), shutil.which('sumo')]
    if os.environ.get('SUMO_HOME'):
        candidates.append(str(Path(os.environ['SUMO_HOME']) / 'bin' / ('sumo.exe' if os.name == 'nt' else 'sumo')))
    if os.name == 'nt':
        for key in ('ProgramFiles', 'ProgramFiles(x86)'):
            if os.environ.get(key):
                candidates.append(str(Path(os.environ[key]) / 'Eclipse/Sumo/bin/sumo.exe'))
    for candidate in candidates:
        if candidate and Path(candidate).is_file():
            return Path(candidate).resolve()
    raise FileNotFoundError('SUMO not found. Set SUMO_BINARY to sumo executable or install SUMO on PATH.')


def find_isaac():
    home = os.environ.get('ISAAC_SIM_PATH')
    candidates = [Path(home) / ('python.bat' if os.name == 'nt' else 'python.sh')] if home else []
    if os.name == 'nt':
        candidates.append(Path('C:/isaacsim/python.bat'))
    for candidate in candidates:
        if candidate.is_file():
            return candidate.resolve()
    raise FileNotFoundError('Set ISAAC_SIM_PATH to the Isaac Sim installation directory.')
