"""Opt-in Kit event timings; no stage mutation or physics stepping.

Application event intervals measure opportunities to service UI, not OS input
latency. NEW_FRAME deliveries are renderer completions, not monitor scanout.
Update-order brackets include all callbacks in that interval, not GPU-only time.
"""
import math
import time


def display_qualification_exclusions(config,paused_wall_s):
    return [name for name,excluded in (
        ('native_profiler_enabled',config.get('profile_render',False)),
        ('user_paused',paused_wall_s>0),('headless',not config['gui'])) if excluded]


def distribution_ms(values):
    ordered=sorted(values)
    if not ordered:return dict(count=0,p50_ms=None,p95_ms=None,max_ms=None)
    return dict(count=len(ordered),p50_ms=1000*ordered[math.ceil(.5*len(ordered))-1],
                p95_ms=1000*ordered[math.ceil(.95*len(ordered))-1],max_ms=1000*ordered[-1])


def assess_responsiveness(frames, completed, *, wall_s, simulation_s, render_hz, complete):
    """Do not confuse render submissions with delivered native renderer frames."""
    starts=[row['pre'] for row in frames if 'pre' in row]
    intervals=[b-a for a,b in zip(starts,starts[1:])]
    delivered_intervals=[b-a for a,b in zip(completed,completed[1:])]
    service=distribution_ms(intervals);delivery=distribution_ms(delivered_intervals)
    stats={}
    for first,last,label in (('pre','before_usd','pre_to_usd'),
            ('before_usd','after_usd','usd_order_bracket'),
            ('after_usd','before_ui','usd_to_ui'),('before_ui','after_ui','ui_order_bracket'),
            ('after_ui','before_hydra','ui_to_hydra'),
            ('before_hydra','after_hydra','hydra_order_bracket'),('after_hydra','post','hydra_to_post')):
        stats[label]=distribution_ms([row[last]-row[first] for row in frames if first in row and last in row])
    delivery_fps=(len(completed)-1)/(completed[-1]-completed[0]) if len(completed)>1 and completed[-1]>completed[0] else None
    rtf=simulation_s/wall_s if wall_s>0 else None
    finite=all(math.isfinite(t) for t in starts+completed+[wall_s,simulation_s])
    monotonic=all(t>=0 for t in intervals+delivered_intervals)
    checks=dict(complete=bool(complete),finite=finite,monotonic=monotonic,
        enough_frames=len(completed)>=max(2,int(simulation_s*render_hz)-2),
        realtime=rtf is not None and .99<=rtf<=1.01,
        native_fps=delivery_fps is not None and delivery_fps>=.99*render_hz,
        frame_p95=delivery['p95_ms'] is not None and delivery['p95_ms']<=1100/render_hz,
        frame_max=delivery['max_ms'] is not None and delivery['max_ms']<=100,
        service_p95=service['p95_ms'] is not None and service['p95_ms']<=50,
        service_max=service['max_ms'] is not None and service['max_ms']<=100)
    return dict(passed=all(checks.values()),gates=checks,requested_hz=render_hz,
        native_completed_frames=len(completed),native_delivery_fps=delivery_fps,
        application_updates=len(frames),application_service_intervals=service,
        native_delivery_intervals=delivery,update_order_timings=stats,
        scope='Kit event service and native NEW_FRAME notification delivery; not scanout or OS click-to-photon latency',
        unique_frame_ids_verified=False,
        notification_caveat='GPU completion notifications can arrive zero, one or several per app update; no 1:1 submission/completion claim',
        actual_mouse_latency_qualified=False)


class KitResponsiveness:
    def __init__(self,clock=time.perf_counter):
        import carb.eventdispatcher
        import omni.kit.app
        from isaacsim.core.rendering_manager import RenderingEvent,RenderingManager
        self.clock=clock;self.frames=[];self.completed=[];self.active=False;self.current=None
        self.renderer=RenderingManager;self.subscriptions=[]
        dispatcher=carb.eventdispatcher.get_eventdispatcher()
        def observe(event,name,order=0):
            self.subscriptions.append(dispatcher.observe_event(observer_name='highwaysim.v07.'+name,
                event_name=event,order=order,on_event=lambda e,n=name:self.mark(n)))
        observe(omni.kit.app.GLOBAL_EVENT_PRE_UPDATE,'pre')
        for name,order in (('before_usd',-11),('after_usd',-9),('before_ui',14),
                           ('after_ui',16),('before_hydra',29),('after_hydra',31)):
            observe(omni.kit.app.GLOBAL_EVENT_UPDATE,name,order)
        observe(omni.kit.app.GLOBAL_EVENT_POST_UPDATE,'post')
        self.callback=self.renderer.register_callback(RenderingEvent.NEW_FRAME,callback=self.frame_complete)

    def mark(self,name):
        if not self.active:return
        now=self.clock()
        if name=='pre':
            self.current={};self.frames.append(self.current)
        if self.current is not None:self.current[name]=now

    def frame_complete(self,*args,**kwargs):
        if self.active:self.completed.append(self.clock())

    def start(self):self.active=True
    def stop(self):self.active=False

    def close(self):
        self.stop();self.subscriptions.clear()
        self.renderer.deregister_callback(self.callback)


def prepared_runtime_inventory(manager):
    """Prepared assets need PhysX, not the sample vehicle authoring wizard."""
    if not manager.is_extension_enabled('omni.physx'):
        raise RuntimeError('Core PhysX must be enabled')
    if manager.is_extension_enabled('omni.physx.vehicle'):
        raise RuntimeError('Prepared showcase unexpectedly loaded vehicle authoring tools')
    enabled=sorted(ext['id'] for ext in manager.get_extensions() if ext.get('enabled'))
    return dict(core_physx_enabled=True,authoring_tools_enabled=False,
        prepared_asset_runtime=True,enabled_extension_count=len(enabled),enabled_extensions=enabled)
