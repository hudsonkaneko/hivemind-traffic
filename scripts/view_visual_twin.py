"""Interactive Isaac viewer for a prepared SUMO visual replay; no physics stepping."""
import argparse
import asyncio
import bisect
import hashlib
import json
import time
import traceback
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scene', type=Path, required=True)
    parser.add_argument('--headless', action='store_true')
    parser.add_argument('--frames', type=int, default=0, help='0 keeps the GUI open')
    parser.add_argument('--check-controls', action='store_true')
    parser.add_argument('--evidence', type=Path)
    args, _ = parser.parse_known_args()
    scene = args.scene.resolve()
    data = json.loads((scene.parent/'recording.json').read_text())
    manifest = json.loads((scene.parent/'manifest.json').read_text())
    frames = data['frames']
    times = [f['time']-frames[0]['time'] for f in frames]
    if args.headless and not args.frames:
        parser.error('--headless requires a finite --frames budget')
    evidence = (args.evidence or scene.parent.parent/('viewer-'+time.strftime('%Y%m%d-%H%M%S'))).resolve()
    evidence.mkdir(parents=True, exist_ok=True)
    if (evidence/'runtime.json').exists():
        parser.error('Choose a fresh evidence directory to preserve earlier runs')
    files = list(scene.parent.glob('*.usda'))
    before = {str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in files}
    # Import only after the user has handled the vendor license prompt.
    from isaacsim import SimulationApp
    app = SimulationApp({'headless':args.headless,'hide_ui':False,'width':1280,'height':800})
    exit_code = 0
    try:
        import omni.usd
        import omni.ui as ui
        import omni.timeline
        import carb
        carb.settings.get_settings().set('/persistent/app/viewport/displayOptions', 0)
        from pxr import Gf, UsdGeom
        from omni.kit.viewport.utility import get_active_viewport, capture_viewport_to_file
        context = omni.usd.get_context()
        context.open_stage(str(scene))
        for _ in range(30):
            app.update()
        stage = context.get_stage()
        stage.SetEditTarget(stage.GetSessionLayer())
        timeline = omni.timeline.get_timeline_interface()
        timeline.pause()
        viewport = get_active_viewport()
        viewport.updates_enabled = True
        state = dict(t=0., playing=False, follow=True, selected=manifest['ids'][0], rate=1.)
        checks = {}

        def seek(value):
            state['t'] = max(0., min(times[-1],float(value)))
            timeline.set_current_time(state['t'])

        def reset():
            state['playing'] = False
            seek(0)

        def toggle():
            if state['t'] >= times[-1]:
                seek(0)
            state['playing'] = not state['playing']

        def camera():
            path = '/World/Cameras/'+('Follow' if state['follow'] else 'Overview')
            selected = '/World/Vehicles/v_'+state['selected'].encode().hex()
            pos = UsdGeom.Xformable(stage.GetPrimAtPath(selected)).ComputeLocalToWorldTransform(state['t']*60).ExtractTranslation()
            if state['follow']:
                eye = pos+Gf.Vec3d(-11,-14,8)
                target = pos+Gf.Vec3d(-1,0,1)
            else:
                xmin,ymin,xmax,ymax=manifest.get('road_bounds_m',[0,-115,900,115])
                span=max(xmax-xmin,ymax-ymin,50)
                target=Gf.Vec3d((xmin+xmax)/2,(ymin+ymax)/2,0)
                eye=target+Gf.Vec3d(-.05*span,-.55*span,.65*span)
            matrix = Gf.Matrix4d().SetLookAt(eye,target,Gf.Vec3d(0,0,1)).GetInverse()
            xf = UsdGeom.Xformable(stage.GetPrimAtPath(path))
            ops = xf.GetOrderedXformOps()
            op = ops[0] if ops else xf.AddTransformOp()
            op.Set(matrix)
            viewport.camera_path = path

        window = ui.Window('Highway twin | recorded SUMO', width=450, height=410)
        with window.frame:
            with ui.VStack(spacing=8):
                ui.Label('SUPPLIED CAR / SUMO REPLAY', height=24)
                ui.Label('Synthetic road · fallback materials · no physics', height=22)
                with ui.HStack(height=30):
                    ui.Button('Play / Pause', clicked_fn=toggle)
                    ui.Button('Reset', clicked_fn=reset)
                    ui.Button('Follow', clicked_fn=lambda:state.update(follow=True))
                    ui.Button('Overview', clicked_fn=lambda:state.update(follow=False))
                ui.Label('Scrub replay seconds (pauses playback)', height=20)
                slider = ui.FloatSlider(min=0,max=times[-1])
                def scrub(model):
                    state['playing']=False
                    seek(model.as_float)
                slider.model.add_value_changed_fn(scrub)
                with ui.HStack(height=26):
                    ui.Label('Playback rate')
                    for rate in (.5,1,2):
                        ui.Button(f'{rate:g}x',clicked_fn=lambda r=rate:state.update(rate=r))
                combo = ui.ComboBox(0,*manifest['ids'])
                def select(model, item):
                    state['selected']=manifest['ids'][model.get_item_value_model().as_int]
                    context.get_selection().set_selected_prim_paths(['/World/Vehicles/v_'+state['selected'].encode().hex()],True)
                combo.model.add_item_changed_fn(select)
                status = ui.Label('',height=110,word_wrap=True)
                final_accounting=frames[-1].get('accounting',{})
                ending=(f"{final_accounting['unfinished']} unfinished trips at recording end."
                    if final_accounting else f"{len(frames[-1]['vehicles'])} vehicles present at recording end; no full trip ledger.")
                ui.Label('Recorded SUMO state; source-sample telemetry.\n'+ending,height=44,word_wrap=True)
        app.update()
        print('VISUAL_TWIN_READY: scene loaded; playback panel and viewport initialized', flush=True)
        if args.check_controls:
            toggle(); checks['play']=state['playing']
            toggle(); checks['pause']=not state['playing']
            seek(10); app.update(); checks['seek']=abs(timeline.get_current_time()-10)<1e-6
            reset(); checks['reset']=state['t']==0 and not state['playing']
            state.update(follow=False); camera(); checks['overview']=str(viewport.camera_path).endswith('/Overview')
            state.update(follow=True,selected=manifest['ids'][-1]); camera()
            checks['selection_follow']=str(viewport.camera_path).endswith('/Follow')
            state.update(selected=manifest['ids'][0]); seek(5)
        def capture_image(name):
            capture=capture_viewport_to_file(viewport,str(evidence/name))
            pending=asyncio.ensure_future(capture.wait_for_result())
            deadline=time.perf_counter()+45
            while not pending.done() and time.perf_counter()<deadline:
                app.update()
                time.sleep(.01)
            if not pending.done():
                pending.cancel()
                raise TimeoutError('Viewport capture did not complete within 45 seconds')
            pending.result()
            # Renderer completion precedes the worker's final PNG write.
            path = evidence/name
            while time.perf_counter()<deadline:
                if path.is_file():
                    try:
                        with path.open('rb') as stream:
                            stream.seek(-12, 2)
                            if stream.read()==b'\x00\x00\x00\x00IEND\xaeB`\x82':
                                return
                    except (OSError, ValueError):
                        pass
                app.update()
                time.sleep(.01)
            raise TimeoutError('Viewport PNG write did not complete within 45 seconds')
        durations=[]
        start_time=state['t']
        index=0
        last=time.perf_counter()
        while app.is_running() and (not args.frames or index<args.frames):
            now=time.perf_counter(); dt=min(now-last,.1); last=now
            if state['playing']:
                seek(state['t']+dt*state['rate'])
                if state['t']>=times[-1]: state['playing']=False
            frame=frames[max(0,bisect.bisect_right(times,state['t'])-1)]
            vehicle=next((v for v in frame['vehicles'] if v['id']==state['selected']),None)
            details=(f"{vehicle['speed']:.2f} m/s | x {vehicle['x']:.2f}, y {vehicle['y']:.2f} m" if vehicle else 'not present at this source sample')
            status.text=f"{'PLAYING' if state['playing'] else 'PAUSED'} | {state['t']:.2f} / {times[-1]:.2f} s\n{state['selected']} | {details}\n{len(frame['vehicles'])} present | source time {frame['time']:.2f} s"
            if 'accounting' in frame:
                ledger=frame['accounting']
                phase='DRAIN' if frame['time']>data['horizon_s'] else 'PRIMARY WINDOW'
                status.text+=f"\n{phase} | requested {ledger['requested']} | waiting {ledger['waiting']}\ncompleted {ledger['completed']} | unfinished {ledger['unfinished']} | removed {ledger['removed']}"
            camera()
            start=time.perf_counter(); app.update(); durations.append(time.perf_counter()-start)
            if index==min(120,max(1,args.frames//3)) and args.frames:
                capture_image('follow.png')
                if args.check_controls: state['playing']=True
            if args.frames and index==args.frames//2:
                state['follow']=False
            if args.frames and index==args.frames*3//4:
                capture_image('overview.png')
                state['follow']=True
            index+=1
        after={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in files}
        if args.check_controls:
            checks['playback_advanced']=state['t']>start_time
        report=dict(frames=index,control_callbacks=checks,static_layers_unchanged=before==after,
                    initial_replay_time=start_time,final_replay_time=state['t'],
                    captures=[p.name for p in evidence.glob('*.png')],
                    app_update_seconds_mean=sum(durations)/len(durations) if durations else None,
                    limitation='Bounded viewer check; update timing includes startup/warmup, not a throughput benchmark. Callback tests are not manual GUI interaction.',
                    scene_sha256=hashlib.sha256(scene.read_bytes()).hexdigest())
        (evidence/'runtime.json').write_text(json.dumps(report,indent=2))
        print('VISUAL_TWIN_RESULT='+json.dumps(report),flush=True)
        timeline.stop()
        context.close_stage()
        for _ in range(30): app.update()
        if not before==after or not all(checks.values()):
            raise RuntimeError('Viewer acceptance check failed')
    except BaseException:
        exit_code = 1
        failure = traceback.format_exc()
        (evidence/'failure.txt').write_text(failure)
        print(failure, flush=True)
        raise
    finally:
        app.close(exit_code=exit_code)


if __name__=='__main__':
    main()
