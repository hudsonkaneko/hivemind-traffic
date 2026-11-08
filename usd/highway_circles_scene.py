"""Portable snapshot of a composed highway without invalidating live handles."""
from pathlib import Path


def export_initial_scene(stage,directory):
    from pxr import Sdf
    directory=Path(directory).resolve()
    filename=directory/'highway-traffic-world.usda'
    if filename.exists():raise FileExistsError(filename)
    root=stage.GetRootLayer()
    original=root.ExportToString()
    layers=[]
    for identifier in root.subLayerPaths:
        layer=Sdf.Layer.Find(identifier)
        if layer is None or layer.anonymous:raise ValueError('Expected packaged file-backed source layers')
        path=Path(layer.realPath).resolve()
        if not path.is_relative_to(directory):raise ValueError('Source layer is outside run package')
        layers.append('./'+path.relative_to(directory).as_posix())
    saved=Sdf.Layer.CreateNew(str(filename))
    saved.TransferContent(root)
    saved.subLayerPaths=layers
    saved.Save()
    if root.ExportToString()!=original:raise RuntimeError('Snapshot changed the live composition')
    return filename
