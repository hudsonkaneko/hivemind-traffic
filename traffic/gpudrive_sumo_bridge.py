"""Explicit SUMO-map/continuous-motion bridge for an untrained transfer test.

GPUDrive owns ego kinematics; SUMO owns other vehicles. No steering-to-lane
conversion, learned updates, or hidden reference-controller fallback.
"""
from __future__ import annotations
import math
import xml.etree.ElementTree as ET
import numpy as np


def wrap(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


def center_from_front(x, y, heading_deg, length):
    yaw = math.radians(90 - heading_deg)
    return x - .5 * length * math.cos(yaw), y - .5 * length * math.sin(yaw), wrap(yaw)


def front_from_center(car):
    return (car['x'] + .5 * car['length'] * math.cos(car['yaw']),
            car['y'] + .5 * car['length'] * math.sin(car['yaw']),
            (90 - math.degrees(car['yaw'])) % 360)


def bicycle_step(car, acceleration, steering, dt=.1):
    """GPUDrive classic COM bicycle equations; full vehicle length wheelbase."""
    beta = math.atan(.5 * math.tan(steering))
    # Upstream starts from velocity magnitude each tick, including after a
    # braking step whose final velocity vector points backwards.
    speed = abs(car['speed'])
    mean_speed = speed + .5 * acceleration * dt
    return dict(car,
        x=car['x'] + mean_speed * math.cos(car['yaw'] + beta) * dt,
        y=car['y'] + mean_speed * math.sin(car['yaw'] + beta) * dt,
        yaw=wrap(car['yaw'] + mean_speed * math.cos(beta) * math.tan(steering) / car['length'] * dt),
        speed=abs(speed + acceleration * dt), signed_step_speed=speed + acceleration * dt,
        acceleration=acceleration)


def corners(car):
    c, s = math.cos(car['yaw']), math.sin(car['yaw'])
    return np.asarray([(car['x'] + c*x - s*y, car['y'] + s*x + c*y)
                       for x,y in [(-car['length']/2,-car['width']/2),
                                   (car['length']/2,-car['width']/2),
                                   (car['length']/2,car['width']/2),
                                   (-car['length']/2,car['width']/2)]])


def overlaps(a, b):
    """Full-size oriented rectangle contact, independent of SUMO lane matching."""
    p, q = corners(a), corners(b)
    for points in (p, q):
        for edge in (points[1]-points[0], points[3]-points[0]):
            axis=np.asarray([-edge[1],edge[0]])
            u,v=p@axis,q@axis
            if u.max() < v.min() or v.max() < u.min():
                return False
    return True


class RoadMap:
    def __init__(self, path, segment_length=5.):
        self.lanes, self.roads, ribbons = {}, [], []
        for edge in ET.parse(path).findall('edge'):
            lanes = edge.findall('lane')
            for i,lane in enumerate(lanes):
                points=np.asarray([list(map(float,p.split(',')))[:2] for p in lane.get('shape').split()])
                width=float(lane.get('width',3.2))
                self.lanes[lane.get('id')]=dict(points=points,width=width,edge=edge.get('id'),
                    speed=float(lane.get('speed')), length=float(lane.get('length')))
                for a,b in zip(points,points[1:]):
                    size=np.linalg.norm(b-a)
                    if size < 1e-8: continue
                    ribbons.append((a,b,width/2))
                    normal=np.asarray([-(b-a)[1],(b-a)[0]])/size
                    definitions=[(0.,3)]
                    # Lane centers, interior dividing lines and outer boundaries.
                    if i==0: definitions.append((-width/2,1))
                    if i==len(lanes)-1: definitions.append((width/2,1))
                    else: definitions.append((width/2,2))
                    for offset,kind in definitions:
                        count=max(1,math.ceil(size/segment_length))
                        for j in range(count):
                            self.roads.append(dict(start=a+(b-a)*j/count+normal*offset,
                                end=a+(b-a)*(j+1)/count+normal*offset,type=kind))
        self.a=np.asarray([r[0] for r in ribbons]); self.d=np.asarray([r[1]-r[0] for r in ribbons])
        self.width=np.asarray([r[2] for r in ribbons]);self.dd=np.sum(self.d*self.d,axis=1)
        self.midpoints=np.asarray([(r['start']+r['end'])/2 for r in self.roads])

    def visible_roads(self, car):
        distance=np.linalg.norm(self.midpoints-[car['x'],car['y']],axis=1)
        order=np.argsort(distance,kind='stable')
        return [self.roads[i] for i in order if distance[i] <= 50.][:200]

    def inside(self, point, tolerance=.05):
        t=np.clip(np.sum((point-self.a)*self.d,axis=1)/self.dd,0,1)
        distance=np.linalg.norm(point-self.a-t[:,None]*self.d,axis=1)
        return bool(np.any(distance <= self.width+tolerance))

    def offroad(self, car):
        return not all(self.inside(p) for p in corners(car))
