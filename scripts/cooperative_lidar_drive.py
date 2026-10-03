"""Bounded two-car RTX perception and optional V2V intentions; SUMO owns motion."""
import argparse
from dataclasses import asdict
from datetime import datetime, timezone
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
from traffic.realtime_lidar import interpolate, packet_points
from traffic.lidar_avoidance import body_bounds, rectangle_gap
from traffic.lidar_tracking import LidarTracker
from traffic.cooperative_control import CooperativeController
from traffic.runtime_metrics import MemorySampler, ScopedGC, refresh_sensor_packets
from traffic.v2v import V2VBus, IntentPayload, ObservedObstacle


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--gui',action='store_true')
    parser.add_argument('--seconds',type=float,default=45.)
    parser.add_argument('--comm',choices=['none','ideal','degraded'],default='ideal')
    parser.add_argument('--seed',type=int,default=42)
    parser.add_argument('--points',action='store_true')
    parser.add_argument('--gc-mode',choices=['normal','deferred','freeze-established'],default='deferred')
    parser.add_argument('--dropout-step',type=int,default=-1)
    parser.add_argument('--dropout-vehicle',choices=['ego','peer'],default='ego')
    parser.add_argument('--unpaced',action='store_true')
    args=parser.parse_args()
    if not np.isfinite(args.seconds) or not 0<args.seconds<=120:
        parser.error('--seconds must be finite and in (0,120]')
    if args.dropout_step < -1: parser.error('--dropout-step must be >= -1')
    binary=find_sumo();sys.path.append(str(binary.parent.parent/'tools'))
    from traffic.fleet_sumo import FleetTraffic
    run=datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')+'-'+args.comm+'-'+uuid.uuid4().hex[:6]
    out=ROOT/'outputs/cooperative_lidar'/run;out.mkdir(parents=True)
    source=out/'source';source.mkdir()
    inputs=['scripts/cooperative_lidar_drive.py','traffic/fleet_sumo.py','traffic/cooperative_control.py',
            'traffic/lidar_tracking.py','traffic/v2v.py','traffic/runtime_metrics.py','traffic/realtime_lidar.py',
            'traffic/lidar_avoidance.py','traffic/lidar_control.py','traffic/live_runtime.py',
            'scenarios/live_lidar/scenario.sumocfg','scenarios/single_vehicle/network.net.xml']
    for name in inputs:
        destination=source/name;destination.parent.mkdir(parents=True,exist_ok=True)
        shutil.copyfile(ROOT/name,destination)
    def git(*parts):return subprocess.check_output(['git',*parts],cwd=ROOT,text=True).strip()
    manifest=dict(run=run,status='running',config=vars(args),command=[sys.executable,*sys.argv],
        python=sys.version,git_commit=git('rev-parse','HEAD'),git_status=git('status','--porcelain'),
        sumo_version=subprocess.check_output([str(binary),'--version'],text=True).splitlines()[0],
        gpu=subprocess.check_output(['nvidia-smi','--query-gpu=name,driver_version','--format=csv,noheader'],text=True).strip(),
        source_hashes={name:hashlib.sha256((ROOT/name).read_bytes()).hexdigest() for name in inputs},
        authority='SUMO; each controller consumes own lidar, own odometry and timestamped messages only',
        limitations='Two-car scripted static-box fixture; no learned policy or vehicle physics; bounded <=120s')
    (out/'working-tree.patch').write_text(git('diff','HEAD'))
    (out/'manifest.json').write_text(json.dumps(manifest,indent=2))
    app=None;traffic=None;timeline=None;sensors={};rows=[];summary={'passed':False};gc_profile=None
    obstacle=(100.,104.,-5.8,-3.8)
    manifest['evaluation_obstacles']=[obstacle]
    try:
        from isaacsim import SimulationApp
        app=SimulationApp({'headless':not args.gui,'width':960,'height':540,'enable_motion_bvh':True})
        import omni.usd
        import omni.timeline
        import omni.replicator.core as rep
        from pxr import UsdGeom,UsdLux,Gf,Sdf
        from isaacsim.sensors.experimental.rtx import Lidar,LidarSensor,parse_generic_model_output_data
        from isaacsim.core.rendering_manager import RenderingManager
        from isaacsim.core.utils.viewports import set_camera_view
        traffic=FleetTraffic(binary,ROOT/'scenarios/live_lidar/scenario.sumocfg',seed=args.seed)
        ids=traffic.vehicle_ids;state=traffic.snapshot();initial=state
        stage=omni.usd.get_context().get_stage()
        UsdGeom.SetStageUpAxis(stage,UsdGeom.Tokens.z);UsdGeom.SetStageMetersPerUnit(stage,1.)
        stage.SetTimeCodesPerSecond(60);stage.SetFramesPerSecond(60)
        def box(path,center,size,color):
            cube=UsdGeom.Cube.Define(stage,path);cube.CreateSizeAttr(1.)
            cube.AddTranslateOp().Set(Gf.Vec3d(*center));cube.AddScaleOp().Set(Gf.Vec3f(*size))
            cube.CreateDisplayColorAttr([Gf.Vec3f(*color)])
        box('/World/Road',[800,-3.2,-.1],[1600,6.4,.2],[.12,.13,.15])
        box('/World/Obstacle',[102,-4.8,.8],[4,2,1.6],[.9,.25,.12])
        for x in range(0,1500,6):box('/World/Markings/m'+str(x),[x,-3.2,.005],[3,.08,.01],[1,1,1])
        UsdLux.DomeLight.Define(stage,'/World/Light').CreateIntensityAttr(1000)
        translates={};rotates={}
        for vid in ids:
            vehicle=UsdGeom.Xform.Define(stage,'/World/Vehicles/'+vid)
            vehicle.GetPrim().CreateAttribute('sumo:id',Sdf.ValueTypeNames.String).Set(vid)
            translates[vid]=vehicle.AddTranslateOp();rotates[vid]=vehicle.AddRotateZOp()
            box('/World/Vehicles/'+vid+'/Body',[-2.5,0,.8],[5,2,1.6],
                [.12,.5,.85] if vid=='ego' else [.9,.75,.15])
        stage.GetRootLayer().Export(str(out/'static.usda'));stage.SetEditTarget(stage.GetSessionLayer())
        packets={vid:{} for vid in ids};errors=[]
        class Capture(rep.Writer):
            def __init__(self):
                self.data_structure='renderProduct';self.annotators=[rep.annotators.get('GenericModelOutput')]
            def initialize(self,vehicle_id,sink):
                # Registry.get constructs via __new__; overriding initialize
                # must explicitly initialize the annotators and data layout.
                self.__init__()
                self.vehicle_id=vehicle_id;self.sink=sink
            def write(self,payload):
                try:
                    for product in payload.get('renderProducts',{}).values():
                        raw=product.get('GenericModelOutput')
                        if isinstance(raw,dict):raw=raw.get('data')
                        if raw is None:continue
                        data=parse_generic_model_output_data(raw)
                        if not data.numElements:continue
                        modes=[str(data.elementsCoordsType).split('.')[-1],str(data.frameOfReference).split('.')[-1],str(data.motionCompensationState).split('.')[-1]]
                        if modes!=['CARTESIAN','WORLD','NONCOMPENSATED']:raise RuntimeError('Unexpected RTX coordinate contract '+repr(modes))
                        packet=dict(vehicle_id=self.vehicle_id,timestamp=int(data.timestampNs),scan_complete=int(data.scanComplete),
                            xyz=np.column_stack((data.x,data.y,data.z)),flags=np.array(data.flags),offset=np.array(data.timeOffsetNs),
                            start_position=np.array(data.frameStart.posM),end_position=np.array(data.frameEnd.posM),
                            frame_start=int(data.frameStart.timestampNs),frame_end=int(data.frameEnd.timestampNs),received_wall=time.perf_counter())
                        self.sink.clear();self.sink.update(packet)
                except Exception as exc:
                    if len(errors)<8:errors.append(self.vehicle_id+': '+repr(exc))
        rep.WriterRegistry.register(Capture)
        if args.gui and args.points:
            from isaacsim.core.experimental.utils.app import enable_extension
            enable_extension('isaacsim.sensors.rtx.nodes')
        manifest['sensor_attributes']={}
        for vid in ids:
            # Above the 1.6m body: a bumper-height mount self-occludes rear traffic.
            lidar=Lidar.create('/World/Vehicles/'+vid+'/Lidar',config='Example_Rotary',translations=np.array([0,0,1.8]),aux_output_level='FULL')
            prim=lidar.prims[0]
            for attribute in prim.GetAttributes():
                if attribute.GetName().startswith('omni:sensor:Core:emitterState:s001:'):
                    value=attribute.Get()
                    if hasattr(value,'__len__') and len(value)==128:attribute.Set(value[:32])
            for name,value in [('numberOfEmitters',32),('patternFiringRateHz',7200.),('accumulateOutputs',True),('elementsCoordsType','CARTESIAN'),('outputFrameOfReference','WORLD'),('outputMotionCompensationState','NONCOMPENSATED')]:
                if not prim.GetAttribute('omni:sensor:Core:'+name).Set(value):raise RuntimeError('Cannot set sensor '+name)
            sensor=LidarSensor(lidar,annotators=[]);sensors[vid]=sensor
            sensor.attach_writer('Capture',vehicle_id=vid,sink=packets[vid])
            if args.gui and args.points:sensor.attach_writer('draw-point-cloud',size=.04,color=[0,1,.5,1])
            manifest['sensor_attributes'][vid]={a.GetName():str(a.Get()) for a in prim.GetAttributes() if a.GetName().startswith('omni:sensor')}
        timeline=omni.timeline.get_timeline_interface();timeline.set_target_framerate(60)
        RenderingManager.set_dt(1/30);timeline.set_play_every_frame(True)
        timeline.set_end_time(3600);timeline.set_looping(False)
        if args.gui:
            import omni.ui as ui
            window=ui.Window('Two-car lidar | '+args.comm,width=510,height=170)
            with window.frame:label=ui.Label('Starting independent sensors...',word_wrap=True)
        def pose(vehicles):
            for vid,v in vehicles.items():
                translates[vid].Set(Gf.Vec3d(v['x'],v['y'],0));rotates[vid].Set(90-v['angle'])
        pose(state['vehicles']);timeline.play()
        for _ in range(36):app.update()
        if errors or not all(packets.values()):raise RuntimeError(errors[-1] if errors else 'Missing per-vehicle RTX warm-up packet')
        sensor_origin=timeline.get_current_time();traffic_origin=state['time']
        bus=V2VBus(run,ids,mode=args.comm,seed=args.seed,delay_s=.15,jitter_s=.15,drop_probability=.2,ttl_s=.8)
        manifest['transport']=dict(mode=args.comm,delay_s=.15,jitter_s=.15,drop_probability=.2,ttl_s=.8)
        (out/'manifest.json').write_text(json.dumps(manifest,indent=2))
        trackers={vid:LidarTracker() for vid in ids}
        controllers={vid:CooperativeController(vid,i) for i,vid in enumerate(ids)}
        tracks={vid:[] for vid in ids};previous={vid:-1 for vid in ids};accepted={vid:None for vid in ids}
        inboxes={vid:{} for vid in ids};memory=MemorySampler();memory.sample(0)
        sensor_counts={vid:0 for vid in ids};max_pose_error={vid:0. for vid in ids}
        contract=dict(sensor_origin=sensor_origin,traffic_origin=traffic_origin,control_hz=10,render_hz=30,
            scan_time='latest per-ray acquisition timestamp mapped through explicit epochs',sensor_height_m=1.8,
            sensor_ids={vid:'/World/Vehicles/'+vid+'/Lidar' for vid in ids})
        manifest['sensor_ids']=contract['sensor_ids']
        (out/'manifest.json').write_text(json.dumps(manifest,indent=2))
        with ScopedGC(args.gc_mode) as gc_profile:
            # GC preparation can age the last stationary scan past the 250 ms
            # failsafe bound. Refresh scans without advancing SUMO, then anchor
            # both clocks. This fixes setup ordering, not sensor nondeterminism.
            contract['post_setup_refresh']=refresh_sensor_packets(app.update,packets)
            if errors:raise RuntimeError(errors[-1])
            sensor_origin=timeline.get_current_time()
            contract['sensor_origin']=sensor_origin
            started=time.perf_counter();contract['loop_wall_origin']=started
            (out/'timing-contract.json').write_text(json.dumps(contract,indent=2))
            for step in range(round(args.seconds*10)):
                if not app.is_running():raise RuntimeError('Viewer closed before completion')
                begin=time.perf_counter()
                if begin-started>args.seconds+30:raise RuntimeError('Bounded loop wall budget exceeded')
                if errors:raise RuntimeError(errors[-1])
                now=state['time'];sensor_now=timeline.get_current_time()
                # Drain all receivers before any decision/publication in this batch.
                for vid in ids:
                    for message in bus.receive(vid,now):inboxes[vid][message.sender_id]=message
                    inboxes[vid]={sender:m for sender,m in inboxes[vid].items() if now<m.expires_at}
                decisions={};observations={};own_points={}
                for vid in ids:
                    if not (vid==args.dropout_vehicle and args.dropout_step>=0 and step>=args.dropout_step):accepted[vid]=packets[vid].copy()
                    packet=accepted[vid]
                    if not packet or packet['vehicle_id']!=vid:raise RuntimeError('Sensor identity violation '+vid)
                    points,healthy,age=packet_points(packet,state['vehicles'][vid],sensor_now,require_scan_complete=True)
                    receipt_age=begin-packet['received_wall'];is_new=packet['timestamp']>previous[vid]
                    own=state['vehicles'][vid]
                    pose_error=float(np.linalg.norm(packet['end_position']-np.array([own['x'],own['y'],contract['sensor_height_m']])))
                    max_pose_error[vid]=max(max_pose_error[vid],pose_error)
                    pose_valid=pose_error<=own['speed']*.1+.3
                    fresh=healthy and pose_valid and -.001<=receipt_age<=.25
                    if fresh and is_new:
                        scan_end=(packet['timestamp']+int(np.max(packet['offset'])))*1e-9
                        observation_time=traffic_origin+scan_end-sensor_origin
                        tracks[vid]=trackers[vid].update(points,observation_time)
                        previous[vid]=packet['timestamp']
                        sensor_counts[vid]+=1
                    decisions[vid]=controllers[vid].step(state['vehicles'][vid],tracks[vid],list(inboxes[vid].values()),now,fresh=fresh)
                    own_points[vid]=points
                    scan_id=f'{vid}_{step:04d}'
                    np.savez(out/(scan_id+'.npz'),vehicle_id=vid,sensor_path=contract['sensor_ids'][vid],
                        **{key:packet[key] for key in ('xyz','flags','offset','timestamp','scan_complete','frame_start','frame_end','start_position','end_position')})
                    observations[vid]=dict(scan_id=scan_id,packet_timestamp=packet['timestamp'],sensor_age=age,receipt_age=receipt_age,
                        healthy=healthy,fresh=fresh,is_new=is_new,pose_error_m=pose_error,pose_valid=pose_valid,
                        tracks=[asdict(t) for t in tracks[vid]],messages=[asdict(m) for m in inboxes[vid].values()])
                # Evaluation only, after both decisions: prove that a stream
                # contains returns on the other car, not just a self-assigned ID.
                peer_returns={}
                for vid in ids:
                    other=state['vehicles'][next(v for v in ids if v!=vid)]
                    bounds=body_bounds(other);pts=own_points[vid]
                    peer_returns[vid]=int(np.sum((pts[:,0]>=bounds[0]-2)&(pts[:,0]<=bounds[1]+2)&
                        (pts[:,1]>=bounds[2]-.3)&(pts[:,1]<=bounds[3]+.3)))
                for vid in ids:
                    decision=decisions[vid];own=state['vehicles'][vid]
                    observed=ObservedObstacle(*decision.observed_obstacle) if decision.observed_obstacle else None
                    bus.publish(vid,now,IntentPayload(own['x'],own['y'],own['speed'],decision.intended_lane,decision.phase,observed))
                control_end=time.perf_counter();before=state
                state=traffic.step({vid:(d.speed,d.lane_request) for vid,d in decisions.items()})
                if abs(state['time']-before['time']-.1)>1e-8:raise RuntimeError('SUMO control-step drift')
                traci_end=time.perf_counter();swept={}
                for vid in ids:
                    a=body_bounds(before['vehicles'][vid]);b=body_bounds(state['vehicles'][vid])
                    swept[vid]=(min(a[0],b[0])-.02,max(a[1],b[1])+.02,min(a[2],b[2])-.02,max(a[3],b[3])+.02)
                vehicle_gap=rectangle_gap(swept[ids[0]],swept[ids[1]])
                row=dict(step=step,traffic_time=now,sensor_now=sensor_now,ego=before['vehicles'],ego_after=state['vehicles'],
                    decisions={vid:asdict(d) for vid,d in decisions.items()},observations=observations,peer_returns=peer_returns,
                    obstacle_gaps={vid:rectangle_gap(swept[vid],obstacle) for vid in ids},vehicle_gap=vehicle_gap,
                    offroad={vid:body_bounds(state['vehicles'][vid])[2]<-6.4 or body_bounds(state['vehicles'][vid])[3]>0 for vid in ids},
                    collisions=state['collisions'],speed_interventions={vid:abs(state['vehicles'][vid]['speed']-decisions[vid].speed)>.15 for vid in ids})
                if args.gui:
                    a,b=(state['vehicles'][vid] for vid in ids)
                    center=(a['x']+b['x'])/2+5;separation=abs(a['x']-b['x'])
                    set_camera_view(eye=np.array([center-20,-35,max(22,separation*.7)]),target=np.array([center,-3.2,0]))
                    label.text=(f'LIVE RTX LIDAR | scripted control | V2V: {args.comm}\n'
                        f'Blue: {decisions["ego"].phase}  {a["speed"]:.1f} m/s\n'
                        f'Yellow: {decisions["peer"].phase}  {b["speed"]:.1f} m/s\n'
                        f'Messages delivered: {bus.telemetry["delivered_messages"]}  '
                        f'Dropped: {bus.telemetry["dropped_messages"]}\n'
                        'Each car checks its own lidar before acting.')
                render_cost=0.;sleep_cost=0.
                for frame in range(1,4):
                    pose({vid:interpolate(before['vehicles'][vid],state['vehicles'][vid],frame/3) for vid in ids})
                    tick=time.perf_counter();app.update();render_cost+=time.perf_counter()-tick
                    if not args.unpaced:
                        wait=started+(step*3+frame)/30-time.perf_counter()
                        if wait>0:
                            tick=time.perf_counter();time.sleep(wait);sleep_cost+=time.perf_counter()-tick
                drift=abs(timeline.get_current_time()-sensor_origin-(state['time']-traffic_origin))
                if drift>1e-4:raise RuntimeError('Sensor/traffic clock drift '+str(drift))
                elapsed=time.perf_counter()-started;row['deadline_lateness']=elapsed-(step+1)*.1
                row['profile_seconds']=dict(control_and_evidence=control_end-begin,traci=traci_end-control_end,render=render_cost,pacing=sleep_cost)
                rows.append(row)
                with (out/'telemetry.jsonl').open('a') as stream:stream.write(json.dumps(row)+'\n')
                memory.sample((step+1)*.1)
                if step%50==0:print(f'FLEET step={step} comm={args.comm} phase={decisions["ego"].phase} gap={vehicle_gap:.2f}',flush=True)
            wall=time.perf_counter()-started
        memory.sample(len(rows)*.1,force=True)
        rtf=len(rows)*.1/wall;misses=sum(r['deadline_lateness']>.1 for r in rows)
        safe=bool(rows and all(not r['collisions'] and not any(r['offroad'].values()) and min(r['obstacle_gaps'].values())>.3 and r['vehicle_gap']>.3 for r in rows))
        stale=[r for r in rows if not r['observations'][args.dropout_vehicle]['fresh']]
        behavior=controllers['ego'].phase=='complete' if args.dropout_step<0 else bool(stale and
            state['vehicles'][args.dropout_vehicle]['speed']<.2 and all(
                r['decisions'][args.dropout_vehicle]['reason']=='sensor-failsafe' and
                r['decisions'][args.dropout_vehicle]['lane_request'] is None for r in stale))
        isolation=all(any(r['peer_returns'][vid]>=5 and r['observations'][vid]['fresh'] for r in rows[:10]) for vid in ids)
        summary=dict(passed=bool(safe and behavior and isolation and .95<=rtf<=1.05 and misses==0),behavior_passed=bool(safe and behavior),
            steps=len(rows),traffic_seconds=len(rows)*.1,loop_wall_seconds=wall,loop_real_time_factor=rtf,
            deadline_misses_over_100ms=misses,min_vehicle_gap_m=min(r['vehicle_gap'] for r in rows),
            min_obstacle_gap_m=min(min(r['obstacle_gaps'].values()) for r in rows),
            final_phases={vid:controllers[vid].phase for vid in ids},progress_m={vid:state['vehicles'][vid]['x']-initial['vehicles'][vid]['x'] for vid in ids},
            speed_interventions=sum(sum(r['speed_interventions'].values()) for r in rows),communication=bus.telemetry,
            memory=memory.summary(),gc=gc_profile.summary(),limitations=manifest['limitations'])
        merge_time=next((r['traffic_time']-traffic_origin for r in rows if r['decisions']['ego']['lane_request']==1),None)
        completion_time=next((r['traffic_time']-traffic_origin for r in rows if r['decisions']['ego']['phase']=='complete'),None)
        summary.update(safe=safe,completed=controllers['ego'].phase=='complete',duration_sim_s=len(rows)*.1,
            duration_wall_s=wall,real_time_factor=rtf,late_over_100ms=misses,merge_time=merge_time,
            completion_time=completion_time,communications=bus.telemetry,final_phase=summary['final_phases'],
            sensor_counts=sensor_counts,max_sensor_pose_error_m=max_pose_error,
            peer_visibility_passed=isolation,initial_peer_returns={vid:max(r['peer_returns'][vid] for r in rows[:10]) for vid in ids})
        manifest['status']='passed' if summary['passed'] else 'failed'
    except BaseException as exc:
        import traceback;traceback.print_exc();manifest.update(status='failed',error=repr(exc),failure_reason=repr(exc))
        summary.update(error=repr(exc),failure_reason=repr(exc),steps=len(rows))
    finally:
        cleanup=[]
        for sensor in sensors.values():
            for writer in (['Capture','draw-point-cloud'] if args.gui and args.points else ['Capture']):
                try:sensor.detach_writer(writer)
                except Exception as exc:cleanup.append(repr(exc))
        if timeline:
            try:timeline.stop()
            except Exception as exc:cleanup.append(repr(exc))
        if traffic:
            try:traffic.close()
            except Exception as exc:cleanup.append(repr(exc))
        if gc_profile:(out/'gc-profile.json').write_text(json.dumps(gc_profile.summary(),indent=2))
        if cleanup:manifest['cleanup_errors']=cleanup
        (out/'telemetry.json').write_text(json.dumps(rows,indent=2));(out/'summary.json').write_text(json.dumps(summary,indent=2))
        manifest['hashes']={str(f.relative_to(out)):hashlib.sha256(f.read_bytes()).hexdigest() for f in out.rglob('*') if f.is_file() and f.name!='manifest.json'}
        (out/'manifest.json').write_text(json.dumps(manifest,indent=2))
        print('FLEET_RESULT='+json.dumps(dict(output=str(out),**summary)),flush=True)
        # Kit fast shutdown terminates the process; code after close cannot run.
        if app:app.close(exit_code=0 if summary['passed'] else 1)
    return 0 if summary['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
