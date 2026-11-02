"""Compose observed I-80 tracks and GIS linework for the existing replay viewer.

This is an evidence inspection scene, never a SUMO network or trip-demand model.
Inputs are local, qualified-source copies; no download or training occurs here.
"""
import argparse
from collections import defaultdict
import json
import math
from pathlib import Path

from pxr import Gf, Usd, UsdGeom, UsdLux

from scripts.build_visual_twin import ROOT, author_motion, new_stage, sha

SURVEY_FOOT_M = 1200 / 3937
ORIGIN_FT = (6042500.0, 2132800.0)


def local_metres(point, origin=ORIGIN_FT):
    """Subtract State Plane origin BEFORE survey-foot conversion; Z is unknown."""
    if len(point) < 2 or not all(math.isfinite(v) for v in (*point[:2], *origin)):
        raise ValueError('Finite horizontal coordinates required')
    return tuple((point[i] - origin[i]) * SURVEY_FOOT_M for i in range(2))


def clip_segment(a, b, bounds):
    """Clip a finite line segment to xmin,ymin,xmax,ymax (Liang-Barsky)."""
    xmin, ymin, xmax, ymax = bounds
    dx, dy = b[0] - a[0], b[1] - a[1]
    lo, hi = 0.0, 1.0
    for p, q in ((-dx, a[0]-xmin), (dx, xmax-a[0]),
                 (-dy, a[1]-ymin), (dy, ymax-a[1])):
        if p == 0:
            if q < 0:
                return None
        elif p < 0:
            lo = max(lo, q/p)
        else:
            hi = min(hi, q/p)
        if lo > hi:
            return None
    return [(a[0]+lo*dx, a[1]+lo*dy), (a[0]+hi*dx, a[1]+hi*dy)]


def observation_recording(rows):
    """Keep 10 Hz measured positions; estimate heading only for visual placement."""
    tracks = defaultdict(list)
    keys = set()
    for row in rows:
        if row['location'] != 'i-80':
            raise ValueError('Expected I-80 observations')
        identifier, t = int(row['vehicle_id']), int(row['global_time'])
        if (identifier, t) in keys:
            raise ValueError('Duplicate vehicle/time observation')
        keys.add((identifier, t))
        x, y = local_metres((float(row['global_x']), float(row['global_y'])))
        speed = float(row['v_vel']) * .3048
        if not math.isfinite(speed) or speed < 0:
            raise ValueError('Invalid observed speed')
        tracks[identifier].append((t, x, y, speed))
    if not tracks:
        raise ValueError('No observations')
    first = min(t for _, t in keys)
    frames = defaultdict(list)
    for identifier, track in sorted(tracks.items()):
        track.sort()
        # Do not interpolate across an unobserved interval.
        if any(b[0]-a[0] != 100 for a, b in zip(track, track[1:])):
            raise ValueError('Track has a gap; split it before display')
        for i, (t, x, y, speed) in enumerate(track):
            a, b = track[max(0, i-5)], track[min(len(track)-1, i+5)]
            dx, dy = b[1]-a[1], b[2]-a[2]
            heading = math.degrees(math.atan2(dx, dy)) % 360 if dx or dy else 0
            frames[t].append(dict(id=f'ngsim_{identifier}', x=x, y=y,
                                 heading=heading, speed=speed))
    return dict(source='NGSIM observations; no requested-trip ledger',
                epoch_origin_ms=first,
                heading_method='Centered up-to-1s position secant for display only; stationary fallback north',
                frames=[dict(time=(t-first)/1000, vehicles=frames[t]) for t in sorted(frames)])


def build_inspection(sample, gis, esri, vehicle, output):
    rows = json.loads(sample.read_text(encoding='utf-8'))
    recording = observation_recording(rows)
    geography = json.loads(gis.read_text(encoding='utf-8'))
    modern = json.loads(esri.read_text(encoding='utf-8'))
    sr = modern['spatialReference']
    if sr.get('latestWkid', sr['wkid']) != 2227:
        raise ValueError('Caltrans geometry must be NAD83 California zone 3 survey feet')
    if modern.get('exceededTransferLimit'):
        raise ValueError('Truncated geospatial query')
    if not vehicle.is_file():
        raise FileNotFoundError(vehicle)
    output.mkdir(parents=True, exist_ok=False)
    frames = recording['frames']
    points = [(v['x'], v['y']) for f in frames for v in f['vehicles']]
    bounds = [min(p[0] for p in points)-30, min(p[1] for p in points)-20,
              max(p[0] for p in points)+30, max(p[1] for p in points)+20]
    static = new_stage(output/'environment.usda')
    for path, kind in [('/World/Environment', 'group'),
                       ('/World/Environment/Highway', 'component')]:
        Usd.ModelAPI(UsdGeom.Xform.Define(static, path).GetPrim()).SetKind(kind)
    UsdGeom.Scope.Define(static, '/World/Debug')

    def linework(path, paths, color, width, source_id):
        segments = []
        for pts in paths:
            pts = [local_metres(p) for p in pts]
            for a, b in zip(pts, pts[1:]):
                clipped = clip_segment(a, b, bounds)
                if clipped and math.dist(*clipped) > .001:
                    segments.append(clipped)
        if not segments:
            return False
        curve = UsdGeom.BasisCurves.Define(static, path)
        curve.CreateTypeAttr('linear')
        curve.CreateWrapAttr('nonperiodic')
        curve.CreateCurveVertexCountsAttr([2]*len(segments))
        curve.CreatePointsAttr([(x,y,.04) for seg in segments for x,y in seg])
        curve.CreateWidthsAttr([width]); curve.SetWidthsInterpolation('constant')
        curve.CreateDisplayColorAttr([color])
        curve.GetPrim().SetCustomDataByKey('source_id', source_id)
        return True

    counts = defaultdict(int)
    for r in geography['emeryville']:
        if r['attributes']['TYPE'] == 'Detector':
            continue
        counts['historical'] += linework(
            f"/World/Environment/Highway/Historical_{r['record_index']}",
            r['paths'], (.65,.67,.70), .18, f"emeryville.shp:{r['record_index']}")
    for r in geography['camera-coverage']:
        counts['camera'] += linework(f"/World/Debug/CameraCoverage_{r['record_index']}",
            r['paths'], (.15,.7,.3), .12, f"camera-coverage.shp:{r['record_index']}")
    for f in modern['features']:
        oid = int(f['attributes']['OBJECTID'])
        counts['modern'] += linework(f'/World/Debug/ModernCaltrans_{oid}',
            f['geometry']['paths'], (1.,.35,.04), .45, f'Caltrans SHN Lines:{oid}')
    xmin,ymin,xmax,ymax = bounds
    ground = UsdGeom.Cube.Define(static, '/World/Environment/InspectionGround')
    ground.CreateSizeAttr(1)
    ground.AddTranslateOp().Set(((xmin+xmax)/2,(ymin+ymax)/2,-.12))
    ground.AddScaleOp().Set((xmax-xmin+30,ymax-ymin+30,.2))
    ground.CreateDisplayColorAttr([(.07,.09,.12)])
    ground.GetPrim().SetCustomDataByKey('limitation','Flat inspection background; not a reconstructed driving surface')
    UsdGeom.Scope.Define(static, '/World/Lighting')
    UsdLux.DomeLight.Define(static, '/World/Lighting/Sky').CreateIntensityAttr(700)
    sun = UsdLux.DistantLight.Define(static, '/World/Lighting/Sun')
    sun.CreateIntensityAttr(2500); sun.AddRotateXYZOp().Set((-35,-25,20))
    UsdGeom.Scope.Define(static, '/World/Cameras')
    for name in ('Overview','Follow'):
        camera = UsdGeom.Camera.Define(static, '/World/Cameras/'+name)
        camera.CreateFocalLengthAttr(22 if name == 'Overview' else 35)
        camera.CreateHorizontalApertureAttr(36); camera.CreateVerticalApertureAttr(22.5)
        camera.CreateClippingRangeAttr((.1,5000))
    static.GetRootLayer().Save()
    ids = author_motion(frames, vehicle, output, 'NGSIM measured positions; visual heading estimate; no simulation')
    world = new_stage(output/'world.usda')
    world.GetRootLayer().subLayerPaths = ['./motion.usda','./environment.usda']
    world.SetStartTimeCode(0); world.SetEndTimeCode(frames[-1]['time']*60)
    world.SetInterpolationType(Usd.InterpolationTypeLinear)
    world.GetRootLayer().Save()
    if world.GetCompositionErrors():
        raise RuntimeError(str(world.GetCompositionErrors()))
    (output/'recording.json').write_text(json.dumps(recording),encoding='utf-8')
    manifest = dict(ids=ids, duration_s=frames[-1]['time'], source_time_origin=0,
        road_bounds_m=bounds, units='metres', up_axis='Z', origin_survey_ft=ORIGIN_FT,
        crs='EPSG:2227, based on NGSIM metadata; no vertical reference',
        coordinate_rule='(State Plane coordinate - origin) * 1200/3937',
        geometry='Historical GIS linework plus modern Caltrans/ArcGIS centerlines; no driving surface',
        viewer_title='I-80 / Powell | evidence inspection',
        viewer_heading='OBSERVED TRACKS / SITE NOT ADMITTED',
        viewer_subtitle='2005 GIS (gray) + modern Esri context (orange). Flat inspection plane; no physics.',
        viewer_footer='Green: camera coverage. Heading estimated for display.\nTrack disappearance is NOT trip completion.\nRequested demand, outside queues and initial missing vehicles are unknown.',
        source_hashes={str(p):sha(p) for p in (sample,gis,esri,vehicle)},
        linework_counts=dict(counts), source_vehicle_samples=len(points),
        limitations=['No independent anchor validation or calibrated dynamics',
                     'One visual car model for all observations; source dimensions/classes differ',
                     'Positions linearly interpolated for display; no gap-model inference',
                     'Original scene and accounted synthetic demo remain separate'])
    (output/'manifest.json').write_text(json.dumps(manifest,indent=2),encoding='utf-8')
    return manifest


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    for name in ('sample','gis','esri','vehicle','output'):
        p.add_argument('--'+name,type=Path,required=True)
    args = p.parse_args()
    if not args.output.resolve().is_relative_to(ROOT):
        p.error('Scene outputs must remain in this checkout')
    print(json.dumps(build_inspection(args.sample,args.gis,args.esri,args.vehicle,args.output),indent=2))
