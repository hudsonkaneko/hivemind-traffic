"""Minimal RTX box-traffic load probe. NOT a controller or physics benchmark."""
import argparse
from dataclasses import asdict
import json
from pathlib import Path
import sys
import time

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from experiments.probe_support import process_sample, write_json
from traffic.runtime_metrics import ScopedGC


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--cars', type=int, required=True)
    p.add_argument('--sensors', type=int, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--frames', type=int, default=240)
    args = p.parse_args()
    if not 0 <= args.sensors <= min(args.cars,4) or not 2 <= args.cars <= 50 or not 30 <= args.frames <= 600:
        p.error('Bounded probe requires 2..50 cars, 0..4 sensors, 30..600 frames')
    output = args.output.resolve()
    if not output.is_relative_to(ROOT/'outputs') or not output.is_dir():
        p.error('Output must be an existing project outputs directory')
    result = dict(passed=False)
    app = timeline = None
    sensors = []
    try:
        from isaacsim import SimulationApp
        app = SimulationApp({'headless':True, 'width':960, 'height':540, 'enable_motion_bvh':True,
                             'disable_viewport_updates':True})
        import omni.usd
        import omni.timeline
        import omni.replicator.core as rep
        from pxr import UsdGeom, UsdLux, Gf
        from isaacsim.sensors.experimental.rtx import Lidar, LidarSensor, parse_generic_model_output_data
        from isaacsim.core.rendering_manager import RenderingManager
        stage = omni.usd.get_context().get_stage()
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(stage, 1.)
        def box(path, center, scale):
            prim = UsdGeom.Cube.Define(stage, path)
            prim.CreateSizeAttr(1.)
            op = prim.AddTranslateOp(); op.Set(Gf.Vec3d(*center))
            prim.AddScaleOp().Set(Gf.Vec3f(*scale))
            return op
        box('/World/Road', (500,-3.2,-.1), (1000,6.4,.2))
        UsdLux.DomeLight.Define(stage, '/World/Light').CreateIntensityAttr(1000)
        positions = [(20 + 14*(i//2), -4.8 if i%2==0 else -1.6) for i in range(args.cars)]
        ops=[]
        for i, (x,y) in enumerate(positions):
            v = UsdGeom.Xform.Define(stage, f'/World/Vehicles/car_{i}')
            op=v.AddTranslateOp(); op.Set(Gf.Vec3d(x,y,0)); ops.append(op)
            box(f'/World/Vehicles/car_{i}/Body',(-2.5,0,.8),(5,2,1.6))
        latest={}; counts={i:0 for i in range(args.sensors)}; errors=[]
        class LoadCapture(rep.Writer):
            def __init__(self):
                self.data_structure='renderProduct'; self.annotators=[rep.annotators.get('GenericModelOutput')]
            def initialize(self, sensor_id):
                self.__init__(); self.sensor_id=sensor_id
            def write(self, payload):
                try:
                    for product in payload.get('renderProducts',{}).values():
                        raw=product.get('GenericModelOutput')
                        if isinstance(raw,dict):raw=raw.get('data')
                        if raw is None:continue
                        data=parse_generic_model_output_data(raw)
                        if not data.numElements:continue
                        old=latest.get(self.sensor_id,{})
                        if int(data.timestampNs)>old.get('timestamp',-1):counts[self.sensor_id]+=1
                        latest[self.sensor_id]=dict(timestamp=int(data.timestampNs),scan_complete=int(data.scanComplete),
                            xyz=np.column_stack((data.x,data.y,data.z)),flags=np.array(data.flags),offset=np.array(data.timeOffsetNs))
                except Exception as error:
                    if len(errors)<8:errors.append(repr(error))
        rep.WriterRegistry.register(LoadCapture)
        attributes={}
        for i in range(args.sensors):
            lidar=Lidar.create(f'/World/Vehicles/car_{i}/Lidar',config='Example_Rotary',translations=np.array([0,0,1.8]),aux_output_level='FULL')
            prim=lidar.prims[0]
            for attr in prim.GetAttributes():
                if attr.GetName().startswith('omni:sensor:Core:emitterState:s001:'):
                    value=attr.Get()
                    if hasattr(value,'__len__') and len(value)==128:attr.Set(value[:32])
            for name,value in [('numberOfEmitters',32),('patternFiringRateHz',7200.),('accumulateOutputs',True),('elementsCoordsType','CARTESIAN'),('outputFrameOfReference','WORLD'),('outputMotionCompensationState','NONCOMPENSATED')]:
                if not prim.GetAttribute('omni:sensor:Core:'+name).Set(value):raise RuntimeError('Attribute failed '+name)
            sensor=LidarSensor(lidar,annotators=[]);sensor.attach_writer('LoadCapture',sensor_id=i);sensors.append(sensor)
            attributes[str(i)]={a.GetName():str(a.Get()) for a in prim.GetAttributes() if a.GetName().startswith('omni:sensor')}
        write_json(output/'sensor-attributes.json',attributes)
        timeline=omni.timeline.get_timeline_interface()
        timeline.set_target_framerate(60); RenderingManager.set_dt(1/30)
        timeline.set_play_every_frame(True);timeline.set_end_time(3600);timeline.set_looping(False);timeline.play()
        for _ in range(45):app.update()
        if errors or len(latest)!=args.sensors:raise RuntimeError('Sensor warmup failed '+repr(errors))
        with ScopedGC('deferred'):
            for _ in range(6):app.update()
            initial_counts=counts.copy(); before=process_sample(); started=time.perf_counter(); times=[]
            for frame in range(args.frames):
                tick=time.perf_counter()
                if tick-started>60:raise TimeoutError('Probe loop exceeded 60 seconds')
                for op,(x,y) in zip(ops,positions):op.Set(Gf.Vec3d(x+8*(frame+1)/30,y,0))
                app.update()
                times.append((time.perf_counter()-tick)*1000)
            wall=time.perf_counter()-started; after=process_sample()
        new_counts={str(k):v-initial_counts[k] for k,v in counts.items()}
        healthy=not errors and all(v>=args.frames/30*5 for v in new_counts.values()) and all(v['scan_complete'] for v in latest.values())
        result=dict(passed=healthy,cars=args.cars,sensors=args.sensors,frames=args.frames,wall_seconds=wall,
            represented_seconds=args.frames/30,real_time_factor=(args.frames/30)/wall,
            p50_frame_ms=float(np.percentile(times,50)),p95_frame_ms=float(np.percentile(times,95)),max_frame_ms=max(times),
            updates_per_second=args.frames/wall,resources_before=before,resources_after=after,
            sensor_new_packets=new_counts,latest_point_counts={str(k):len(v['xyz']) for k,v in latest.items()},errors=errors,
            caveat='Unpaced sensor/readback and moving-box probe; no controller, SUMO, GUI, physics or scan recording')
        write_json(output/'frame-ms.json',times)
    except Exception as error:
        import traceback;traceback.print_exc();result['error']=repr(error)
    finally:
        for sensor in sensors:
            try:sensor.detach_writer('LoadCapture')
            except Exception:pass
        if timeline:timeline.stop()
        write_json(output/'probe-result.json',result)
        print('RTX_PROBE='+json.dumps(result),flush=True)
        if app:app.close(exit_code=0 if result['passed'] else 1)
    return int(not result['passed'])


if __name__=='__main__':raise SystemExit(main())
