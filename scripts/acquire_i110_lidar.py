"""Bounded, cached public EPT acquisition for the MLK--Vernon development tile.

Run with the isolated geo environment. Downloads data only; no Isaac or PeMS.
"""
import argparse
import concurrent.futures as cf
import hashlib
import json
from pathlib import Path
import urllib.request

import laspy
import numpy as np
from pyproj import Transformer

BASE = 'https://s3-us-west-2.amazonaws.com/usgs-lidar-public/CA_LosAngeles_1_B23/'
ORIGIN_LONLAT = (-118.2812851, 34.0109392)
BOUNDS = (-85., -940., 85., 45.)


def acquire(output, max_depth=16, max_nodes=1600, max_points=15000000):
    output.mkdir(parents=True, exist_ok=True)
    records = []

    def get(name, url):
        p = output / name
        if not p.exists():
            with urllib.request.urlopen(url, timeout=45) as response:
                body = response.read(12000001)
            if len(body) > 12000000:
                raise ValueError('Per-file 12 MB limit exceeded')
            p.write_bytes(body)
        record = dict(path=name, url=url, bytes=p.stat().st_size,
                      sha256=hashlib.sha256(p.read_bytes()).hexdigest())
        return p, record

    p, receipt = get('ept.json', BASE+'ept.json'); records.append(receipt)
    ept = json.loads(p.read_text())
    if ept['srs']['horizontal'] != '3857':
        raise ValueError('Unexpected EPT CRS')
    origin = Transformer.from_crs(4326, 6340, always_xy=True).transform(*ORIGIN_LONLAT)
    to_ept = Transformer.from_crs(6340, 3857, always_xy=True)
    corners = [to_ept.transform(origin[0]+x, origin[1]+y)
               for x in BOUNDS[::2] for y in BOUNDS[1::2]]
    box = [min(x for x,y in corners), min(y for x,y in corners),
           max(x for x,y in corners), max(y for x,y in corners)]
    bounds = ept['bounds']; side = bounds[3]-bounds[0]

    def intersects(key):
        d,x,y,z = map(int, key.split('-')); s = side/2**d
        return (d <= max_depth and bounds[0]+x*s <= box[2]
                and bounds[0]+(x+1)*s >= box[0] and bounds[1]+y*s <= box[3]
                and bounds[1]+(y+1)*s >= box[1])

    todo = ['0-0-0-0']; visited = set(); nodes = {}
    while todo:
        k = todo.pop()
        if k in visited: continue
        visited.add(k)
        p, receipt = get('hierarchy-'+k+'.json', BASE+'ept-hierarchy/'+k+'.json')
        records.append(receipt)
        for key,n in json.loads(p.read_text()).items():
            if not intersects(key): continue
            if n == -1: todo.append(key)
            elif n > 0: nodes[key] = n
    plan = dict(source=BASE, origin_lonlat=ORIGIN_LONLAT, origin_6340=origin,
                bounds_local_m=BOUNDS, bbox_3857=box, max_depth=max_depth,
                nodes=nodes, points_before_clip=sum(nodes.values()))
    (output/'plan.json').write_text(json.dumps(plan, indent=2))
    print(json.dumps(dict(nodes=len(nodes), points=sum(nodes.values()),
                          deepest=max(int(k.split('-')[0]) for k in nodes))), flush=True)
    if len(nodes) > max_nodes or sum(nodes.values()) > max_points:
        raise ValueError('Acquisition budget exceeded; inspect plan before changing bounds')
    with cf.ThreadPoolExecutor(max_workers=6) as pool:
        results = list(pool.map(lambda k:get('node-'+k+'.laz', BASE+'ept-data/'+k+'.laz'), sorted(nodes)))
    records.extend(r for p,r in results)
    back = Transformer.from_crs(3857, 6340, always_xy=True)
    arrays = []
    for path, receipt in results:
        cloud = laspy.read(path)
        x,y = back.transform(np.asarray(cloud.x), np.asarray(cloud.y))
        x,y = x-origin[0], y-origin[1]
        keep = (x>=BOUNDS[0])&(x<=BOUNDS[2])&(y>=BOUNDS[1])&(y<=BOUNDS[3])
        arrays.append(np.column_stack([x[keep],y[keep],np.asarray(cloud.z)[keep],
                                       np.asarray(cloud.classification)[keep],np.asarray(cloud.intensity)[keep]]))
    points = np.concatenate(arrays)
    np.save(output/'points.npy', points)
    classes, counts = np.unique(points[:,3], return_counts=True)
    manifest = dict(**plan, retained_points=len(points),
                    classes={str(int(c)):int(n) for c,n in zip(classes,counts)},
                    receipts=records, points_sha256=hashlib.sha256((output/'points.npy').read_bytes()).hexdigest(),
                    vertical='Source Z preserved; project NAVD88, EPT vertical CRS absent; no vertical datum transformation',
                    epoch='Project 2023--2024; individual flight times not yet qualified',
                    limitation='Classifications and source geometry are not an independent survey/accuracy validation')
    (output/'manifest.json').write_text(json.dumps(manifest, indent=2))
    print(json.dumps({k:manifest[k] for k in ('retained_points','classes','points_sha256')}), flush=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    if not args.output.resolve().is_relative_to(root): parser.error('Output must stay within checkout')
    acquire(args.output)
