"""Backend-isolated environments; importing a Lab task must not import SUMO."""

__all__ = ["HighwayParallelEnv", "parallel_env"]


def __getattr__(name):
    if name in __all__:
        from .highway_parallel_env import HighwayParallelEnv, parallel_env
        return {'HighwayParallelEnv': HighwayParallelEnv, 'parallel_env': parallel_env}[name]
    raise AttributeError(f'module {__name__!r} has no attribute {name!r}')

