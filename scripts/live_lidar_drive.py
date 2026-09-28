"""Live SUMO -> RTX lidar -> clearance controller -> SUMO, with immutable evidence."""
import argparse
from datetime import datetime,timezone
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys
import time
import uuid
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from traffic.live_runtime import find_sumo
from traffic.lidar_control import forward_clearance,target_speed


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--gui',action='store_true')
    p.add_argument('--mode',choices=['stop','follow'],default='stop')
    p.add_argument('--speed',type=float,default=8.)
    p.add_argument('--gap',type=float,default=40.)
    p.add_argument('--seconds',type=float,default=25.)
    p.add_argument('--seed',type=int,default=42)
    p.add_argument('--fault-step',type=int,default=-1)
    p.add_argument('--ideal-sensor',action='store_true',help='Diagnostic only: disable configured angular/range noise')
    args=p.parse_args()
    if not all(np.isfinite(v) and v>0 for v in [args.speed,args.gap,args.seconds]): p.error('positive finite speed/gap/seconds required')
    if args.speed>12 or args.gap>100 or args.seconds>40: p.error('This validated small-road fixture supports speed<=12, gap<=100, seconds<=40')
    binary=find_sumo()
    # SUMO ships portable pure-Python TraCI/sumolib; do not mix venv binary packages.
    sys.path.append(str(binary.parent.parent/'tools'))
    import traci
    from traffic.live_lidar_sumo import LiveTraffic
    run=datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')+'-'+args.mode+'-'+uuid.uuid4().hex[:6]
    out=ROOT/'outputs/live_lidar'/run;out.mkdir(parents=True)
    (out/'source').mkdir()
    files=['scripts/live_lidar_drive.py','traffic/lidar_control.py','traffic/live_lidar_sumo.py','traffic/live_runtime.py',
           'scenarios/live_lidar/scenario.sumocfg','scenarios/single_vehicle/network.net.xml']
    for f in files: shutil.copyfile(ROOT/f,out/'source'/Path(f).name)
    def git(*a): return subprocess.check_output(['git',*a],cwd=ROOT,text=True).strip()
    manifest=dict(run=run,status='running',config=vars(args),python=sys.version,traci=traci.__file__,
        sumo_version=subprocess.check_output([str(binary),'--version'],text=True).splitlines()[0],
        git_commit=git('rev-parse','HEAD'),git_status=git('status','--porcelain'),
        gpu=subprocess.check_output(['nvidia-smi','--query-gpu=name,driver_version','--format=csv,noheader'],text=True).strip(),
        timing='SUMO 0.1s lockstep; poses held during fresh RTX acquisition; distinct sensor clock',
        authority='SUMO kinematics; lidar clearance controls ego speed; no PPO or physics steering')
    (out/'working-tree.patch').write_text(git('diff','HEAD'))
    (out/'manifest.json').write_text(json.dumps(manifest,indent=2))
    rows=[];traffic=None;app=None;began=time.monotonic();summary={'passed':False}
    try:
        from isaacsim import SimulationApp
        app=SimulationApp({'headless':not args.gui,'width':960,'height':540,'enable_motion_bvh':True})
        import omni.usd
        import omni.timeline
        import omni.replicator.core as rep
        from pxr import UsdGeom,UsdLux,Gf,Sdf
        from isaacsim.sensors.experimental.rtx import Lidar,LidarSensor,parse_generic_model_output_data
        from isaacsim.core.utils.viewports import set_camera_view
        traffic=LiveTraffic(binary,ROOT/'scenarios/live_lidar/scenario.sumocfg',mode=args.mode,speed=args.speed,gap=args.gap,seed=args.seed)
        state=traffic.snapshot();c=traffic.connection
        manifest['sumo_api_version']=c.getVersion()
        manifest['speed_mode']=c.vehicle.getSpeedMode('ego')
        stage=omni.usd.get_context().get_stage()
        UsdGeom.SetStageUpAxis(stage,UsdGeom.Tokens.z);UsdGeom.SetStageMetersPerUnit(stage,1.)
        stage.SetTimeCodesPerSecond(60);stage.SetFramesPerSecond(60)
        def box(path,center,size,color):
            b=UsdGeom.Cube.Define(stage,path);b.CreateSizeAttr(1)
            b.AddTranslateOp().Set(Gf.Vec3d(*center));b.AddScaleOp().Set(Gf.Vec3f(*size))
            b.CreateDisplayColorAttr([Gf.Vec3f(*color)])
            return b
        box('/World/Road',[200,-3.2,-.1],[400,6.4,.2],[.12,.13,.15])
        for x in range(0,300,6): box('/World/Markings/m'+str(x),[x,-3.2,.005],[3,.08,.01],[1,1,1])
        UsdLux.DomeLight.Define(stage,'/World/Light').CreateIntensityAttr(1000)
        # One reusable body asset; per-vehicle runtime pose opinions stay in session layer.
        asset=UsdGeom.Xform.Define(stage,'/VehicleAsset')
        box('/VehicleAsset/Body',[-2.5,0,.8],[5,2,1.6],[.12,.5,.85])
        UsdGeom.Imageable(asset).CreateVisibilityAttr().Set('invisible')
        transforms={}
        for vid in state['vehicles']:
            xf=UsdGeom.Xform.Define(stage,'/World/Vehicles/'+vid)
            xf.GetPrim().GetReferences().AddInternalReference('/VehicleAsset')
            UsdGeom.Imageable(xf).CreateVisibilityAttr().Set('inherited')
            transforms[vid]=xf.AddTranslateOp()
            xf.GetPrim().CreateAttribute('sumo:id',Sdf.ValueTypeNames.String).Set(vid)
        wall_x=state['vehicles']['ego']['x']+args.gap
        if args.mode=='stop': box('/World/Obstacle',[wall_x+2,-4.8,.8],[4,2,1.6],[.9,.25,.12])
        stage.GetRootLayer().Export(str(out/'static.usda'))
        stage.SetEditTarget(stage.GetSessionLayer())
        if args.gui:
            from isaacsim.core.experimental.utils.app import enable_extension
            enable_extension('isaacsim.sensors.rtx.nodes')
        lidar=Lidar.create('/World/Vehicles/ego/Lidar',config='Example_Rotary',translations=np.array([0,0,1.]),aux_output_level='FULL')
        if args.ideal_sensor:
            for name in ['azimuthErrorStd','elevationErrorStd','rangeAccuracyM']:
                attribute=lidar.prims[0].GetAttribute('omni:sensor:Core:'+name)
                if not attribute or not attribute.Set(0.):
                    raise RuntimeError('Cannot configure diagnostic sensor attribute '+name)
        sensor=LidarSensor(lidar,annotators=[])
        manifest['sensor_attributes']={a.GetName():str(a.Get()) for a in lidar.prims[0].GetAttributes() if a.GetName().startswith('omni:sensor')}
        manifest['isaac_version']=next((f.read_text().strip() for f in [Path(sys.executable).parent.parent.parent/'VERSION'] if f.exists()),'unknown')
        timeline=omni.timeline.get_timeline_interface();timeline.set_target_framerate(60)
        timeline.set_end_time(3600);timeline.set_looping(False)
        latest={};callback_errors=[]
        class Capture(rep.Writer):
            def __init__(self):
                self.data_structure='renderProduct';self.annotators=[rep.annotators.get('GenericModelOutput')]
            def write(self,payload):
                try:
                    for product in payload.get('renderProducts',{}).values():
                        raw=product.get('GenericModelOutput')
                        if isinstance(raw,dict): raw=raw.get('data')
                        if raw is None: continue
                        g=parse_generic_model_output_data(raw)
                        if not g.numElements: continue
                        if not str(g.elementsCoordsType).endswith('SPHERICAL') or not str(g.frameOfReference).endswith('SENSOR'):
                            raise RuntimeError('Unexpected lidar coordinate format')
                        latest.clear();latest.update(timestamp=int(g.timestampNs),az=np.array(g.x),el=np.array(g.y),
                            ranges=np.array(g.z),flags=np.array(g.flags),offset=np.array(g.timeOffsetNs),
                            start_position=np.array(g.frameStart.posM),end_position=np.array(g.frameEnd.posM),
                            frame_start=int(g.frameStart.timestampNs),frame_end=int(g.frameEnd.timestampNs))
                except Exception as e: callback_errors.append(repr(e))
        rep.WriterRegistry.register(Capture);sensor.attach_writer('Capture')
        if args.gui:
            sensor.attach_writer('draw-point-cloud',size=.04,color=[0,1,.5,1])
            import omni.ui as ui
            window=ui.Window('Live lidar control',width=430,height=150)
            with window.frame: label=ui.Label('Starting sensor...',word_wrap=True)
        timeline.play()
        previous_stamp=-1
        for step in range(round(args.seconds/.1)):
            if not app.is_running(): raise RuntimeError('Viewer closed before completion')
            ego=state['vehicles']['ego']
            for vid,v in state['vehicles'].items():
                if abs(v['angle']-90)>1e-5: raise RuntimeError('Straight-road heading contract violated')
                transforms[vid].Set(Gf.Vec3d(v['x'],v['y'],0))
            expected_pose=np.array([ego['x'],ego['y'],1.])
            # Drain two full rotary periods after pose edit before accepting a scan.
            # SUMO remains frozen; do not use a scan spanning old and new poses.
            epoch=timeline.get_current_time()
            acquired=False
            for frame in range(36):
                app.update()
                if callback_errors: raise RuntimeError(callback_errors[-1])
                if frame<17 or not latest or latest['timestamp']<=previous_stamp: continue
                s=latest
                ray_start=(s['timestamp']+int(s['offset'].min()))*1e-9
                if ray_start<epoch+.1: continue
                if max(np.linalg.norm(s['start_position']-expected_pose),np.linalg.norm(s['end_position']-expected_pose))>.001: continue
                acquired=True;break
            if not acquired:
                # No traffic step is allowed without a valid acquisition. Freeze and fail closed.
                raise RuntimeError('Fresh complete lidar scan timed out; SUMO held stationary')
            s=latest;previous_stamp=s['timestamp']
            clearance=forward_clearance(s['az'],s['el'],s['ranges'],s['flags'])
            fresh=step!=args.fault_step
            requested,reason=target_speed(clearance,ego['speed'],cruise=args.speed,fresh=fresh)
            # Truth is computed only after the controller decision, solely for evaluation.
            true_gap=(wall_x-ego['x']) if args.mode=='stop' else state['vehicles']['lead']['x']-5-ego['x']
            measurement_error=abs(clearance.distance-true_gap)
            np.savez_compressed(out/f'scan_{step:04d}.npz',**{k:s[k] for k in ['timestamp','az','el','ranges','flags','offset']})
            before=state['time'];state=traffic.step(requested)
            if abs(state['time']-before-.1)>1e-8: raise RuntimeError('SUMO step mismatch')
            after=state['vehicles']['ego']
            post_gap=(wall_x-after['x']) if args.mode=='stop' else state['vehicles']['lead']['x']-5-after['x']
            row=dict(step=step,traffic_time=before,sensor_time=s['timestamp']*1e-9,sensor_epoch=epoch,
                lidar_distance=clearance.distance,points=clearance.points,healthy=clearance.healthy,
                speed=ego['speed'],requested_speed=requested,realized_speed=after['speed'],reason=reason,
                true_gap=true_gap,post_gap=post_gap,error_m=measurement_error,ego_x=ego['x'],
                lead_speed=state['vehicles'].get('lead',{}).get('speed'),collisions=state['collisions'])
            rows.append(row)
            if args.gui:
                set_camera_view(eye=np.array([ego['x']-15,-24,15]),target=np.array([ego['x']+15,-4.8,0]))
                label.text=f'{args.mode.upper()} | SUMO {before:.1f}s\nLidar gap {clearance.distance:.2f} m | speed {after["speed"]:.2f} m/s\n{reason} | fresh scan {step+1}\nSUMO road-following; RTX distance-based speed control'
                if step==50:
                    from omni.kit.viewport.utility import get_active_viewport,capture_viewport_to_file
                    capture_viewport_to_file(get_active_viewport(),str(out/'preview.png'))
            if step%50==0: print(f'LIVE step={step} gap={clearance.distance:.2f} speed={after["speed"]:.2f}',flush=True)
        timeline.stop();sensor.detach_writer('Capture')
        minimum=min(r['post_gap'] for r in rows);error=max(r['error_m'] for r in rows)
        interventions=sum(abs(r['realized_speed']-r['requested_speed'])>.15 for r in rows)
        progress=state['vehicles']['ego']['x']-traffic.initial_x
        passed=(minimum>=2 and error<=.25 and progress>=5 and rows[-1]['realized_speed']<.2
                and not any(r['collisions'] for r in rows) and all(r['healthy'] for r in rows) and interventions==0)
        summary=dict(passed=bool(passed),steps=len(rows),min_gap_m=minimum,max_clearance_error_m=error,
            final_speed=rows[-1]['realized_speed'],progress_m=progress,speed_interventions=interventions,
            sensor_fault_actions=sum(r['reason']=='sensor-failsafe' for r in rows),
            wall_seconds=time.monotonic()-began,traffic_seconds=len(rows)*.1,
            limitation='Discrete sample-and-hold RTX; SUMO lane following, not physics steering or continuous-motion lidar')
        manifest['status']='passed' if passed else 'failed'
    except BaseException as e:
        import traceback;traceback.print_exc();manifest.update(status='failed',error=repr(e))
    finally:
        if traffic: traffic.close()
        (out/'telemetry.json').write_text(json.dumps(rows,indent=2))
        (out/'summary.json').write_text(json.dumps(summary,indent=2))
        manifest['hashes']={str(f.relative_to(out)):hashlib.sha256(f.read_bytes()).hexdigest() for f in out.rglob('*') if f.is_file() and f.name!='manifest.json'}
        (out/'manifest.json').write_text(json.dumps(manifest,indent=2))
        print('LIVE_RESULT='+json.dumps(dict(output=str(out),**summary)),flush=True)
        if app: app.close()
    return 0 if summary['passed'] else 1


if __name__=='__main__': raise SystemExit(main())
