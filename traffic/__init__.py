"""Traffic simulation backends and controllers."""

__all__ = ["SumoBackend", "VehicleCommand"]


def __getattr__(name):
    # Sensor-only utilities must remain importable before the live runner adds
    # SUMO's tools directory to sys.path. Load the legacy backend on demand.
    if name in __all__:
        from . import sumo_backend
        return getattr(sumo_backend, name)
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")

