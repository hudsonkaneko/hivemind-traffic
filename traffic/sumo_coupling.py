"""SUMO coupling for the simulation environments: TraCI over a socket (default) or libsumo in process.

v15 Phase 0 (user, November 25, 2026: "tell the other session to start phase 0"). libsumo runs
SUMO inside this Python process: no socket, and one simulation per process. SUMO's docs say
parallel libsumo instances need separate processes (Python's multiprocessing), and that sumo-gui
does not work with libsumo on Windows. So TraCI stays the default and the GUI path.

`use('libsumo')` rebinds the module-level name `traci` in the two environment modules that use it:
- environments/i110_merge_env.py starts SUMO for every v05-v14 environment
  (traci.start + traci.getConnection + a socket timeout);
- environments/highway_flow_rl_v13_env.py catches traci.TraCIException.
Both look the name up at call time. No environment source is edited, so the training runners'
pinned source hashes are unchanged. `use('traci')` restores the real module.

The backend can also come from the environment variable HIGHWAYSIM_SUMO_BACKEND ('traci' or
'libsumo'), which worker processes inherit.
"""
from __future__ import annotations

import os

BACKENDS = ('traci', 'libsumo')
ENV_VAR = 'HIGHWAYSIM_SUMO_BACKEND'
PINNED_VERSION = 'SUMO 1.27.1'          # libsumo, sumo-data, traci and the installed sumo binary
_current = 'traci'


class _NoSocket:
    """libsumo has no socket; the environments' 30 s socket timeout has nothing to apply to."""

    def settimeout(self, _seconds):
        return None


class LibsumoConnection:
    """The traci.Connection surface the environments use (domains, simulationStep, close)."""

    _socket = _NoSocket()

    def __init__(self, owner):
        self._owner = owner

    def __getattr__(self, name):
        return getattr(self._owner.libsumo, name)

    def close(self):
        self._owner.close()


class LibsumoTraci:
    """Stands in for the traci module: start / getConnection with one simulation per process."""

    def __init__(self):
        import libsumo
        version = libsumo.getVersion()[1]
        if version != PINNED_VERSION:
            raise RuntimeError(f'libsumo reports {version!r}; this project pins {PINNED_VERSION!r}')
        self.libsumo = libsumo
        self.TraCIException = libsumo.TraCIException
        self.FatalTraCIError = libsumo.FatalTraCIError
        self.exceptions = libsumo.exceptions
        self.label = None

    def start(self, command, label='default', verbose=False, **_ignored):
        if self.label is not None:
            raise RuntimeError('libsumo runs one simulation per process; close the open one first')
        self.libsumo.start([str(part) for part in command])
        self.label = label

    def getConnection(self, label):
        if label != self.label:
            raise KeyError(f'No open libsumo simulation labelled {label!r}')
        return LibsumoConnection(self)

    def close(self):
        if self.label is not None:
            self.label = None
            self.libsumo.close()


def use(backend=None):
    """Select the coupling for this process; returns the backend name."""
    global _current
    backend = backend or os.environ.get(ENV_VAR, 'traci')
    if backend not in BACKENDS:
        raise ValueError(f'Unknown SUMO backend {backend!r}; expected one of {BACKENDS}')
    from environments import highway_flow_rl_v13_env, i110_merge_env
    if backend == 'traci':
        import traci
        module = traci
    else:
        module = LibsumoTraci()
    for environment in (i110_merge_env, highway_flow_rl_v13_env):
        environment.traci = module
    _current = backend
    return backend


def current():
    return _current


def module():
    """The coupling now in use (the traci module or the libsumo stand-in), for code written against it
    directly, such as the flat v15 environment."""
    from environments import i110_merge_env
    return i110_merge_env.traci
