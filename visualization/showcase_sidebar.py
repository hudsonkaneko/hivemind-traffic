"""Native Kit sidebar. Callbacks edit a pure mailbox, not physics or USD."""
from experiments.showcase_controls import (SidebarState, RESTART_CONTROLS, LIVE_CONTROLS,
                                           FUTURE_CONTROLS, defaults)


class ShowcaseSidebar:
    def __init__(self, state, settings, *, visible=True, live_driving=False, statistics=False):
        import omni.ui as ui
        self.state = state
        self.settings = SidebarState(settings,live_driving=live_driving)
        self.live_driving=live_driving
        self.live_status=''
        self.smoke_stage=0
        self.models = {}
        self.widgets = []
        self.future_widgets = []
        self._subscriptions = []
        self._updating = False
        self.window = ui.Window('Highway Sim Controls', width=370, height=720, visible=visible)
        with self.window.frame:
            with ui.ScrollingFrame():
                with ui.VStack(spacing=6, height=0):
                    ui.Label('SCRIPTED SHOWCASE | '+('v09 LIVE' if live_driving else 'v08'), height=24)
                    ui.Label('Main car: PhysX. Traffic: scripted.\nNo LiDAR, trained policy or V2V in this run.',
                             height=40, word_wrap=True)
                    self.telemetry = ui.Label('Preparing...', height=110 if statistics else 65, word_wrap=True)
                    with ui.HStack(height=26):
                        for title, mode in [('Follow', 'follow'), ('Traffic', 'traffic'), ('Overview', 'overview')]:
                            ui.Button(title, clicked_fn=lambda m=mode: self.set_camera(m))
                    with ui.HStack(height=26):
                        self.pause_button = ui.Button('Pause', clicked_fn=self.toggle_pause)
                        ui.Button('Show / hide path', clicked_fn=self.toggle_path)
                    with ui.CollapsableFrame('Traffic and driving | '+('LIVE' if live_driving else 'RESTART'), collapsed=False):
                        with ui.VStack(spacing=5, height=0):
                            ui.Label(('Automatic, gradual changes. No restart.\n' if live_driving else 'Changes take effect in a fresh run.\n')+'4-80 background cars, plus one main car.',
                                     height=36, word_wrap=True)
                            for key in ('traffic_count', 'ego_mph', 'traffic_mph', 'following_s'):
                                self._slider(ui, key, RESTART_CONTROLS[key])
                            with ui.HStack(height=25):
                                for title, count in [('Light (12)', 12), ('Medium (40)', 40), ('Dense (80)', 80)]:
                                    ui.Button(title, clicked_fn=lambda n=count: self.models['traffic_count'].set_value(n))
                            ui.Label('Traffic speed is for the inner lane; outer lanes scale.\nLower traffic speed before lowering the main-car target.\n'+('Changes wait through an active lane change.\nDensity changes are remote, at up to 4 cars/second.' if live_driving else 'Common angular speed preserves background spacing.'),
                                     height=68, word_wrap=True)
                    with ui.CollapsableFrame('Viewing | LIVE', collapsed=False):
                        with ui.VStack(spacing=5, height=0):
                            for key, row in LIVE_CONTROLS.items(): self._slider(ui, key, row)
                    with ui.CollapsableFrame('Rendering | RESTART', collapsed=True):
                        with ui.VStack(spacing=5, height=0):
                            # Discrete supported values, not invalid intermediate settings.
                            for key, options in [('render_hz', (10, 20, 30)), ('resolution_percent', (50, 75, 100))]:
                                ui.Label(RESTART_CONTROLS[key][0], height=20)
                                with ui.HStack(height=24):
                                    for value in options:
                                        ui.Button(f'{value} {RESTART_CONTROLS[key][-1]}',
                                                  clicked_fn=lambda k=key, v=value: self.set_setting(k, v))
                    self.status = ui.Label('', height=76, word_wrap=True)
                    with ui.HStack(height=28):
                        self.apply_button = ui.Button('Restart rendering' if live_driving else 'Apply & restart', clicked_fn=self.request_restart)
                        ui.Button('Reset defaults', clicked_fn=self.reset_defaults)
                    ui.Label('Restart brakes safely, closes Isaac, then relaunches.\nA fresh output folder preserves the previous run.',
                             height=36, word_wrap=True)
                    with ui.CollapsableFrame('Future controls | DISPLAY ONLY', collapsed=True):
                        with ui.VStack(spacing=5, height=0):
                            ui.Label('Not implemented. Disabled previews have NO effect.\nNumbers below are illustrative, not active settings.',
                                     height=38, word_wrap=True)
                            for group, controls in FUTURE_CONTROLS.items():
                                with ui.CollapsableFrame(group, collapsed=True):
                                    with ui.VStack(spacing=4, height=0):
                                        for title, value, low, high, units in controls:
                                            ui.Label(f'{title} ({units}) | Not implemented', height=22)
                                            model=ui.SimpleFloatModel(value)
                                            widget=ui.FloatSlider(model=model, min=low, max=high, enabled=False,
                                                height=20, tooltip='Display only. Not implemented; has no simulation effect.')
                                            self.future_widgets.append(widget)
        self.refresh_status()
    def dock(self):
        """Called after native windows have received their first layout frame."""
        import omni.ui as ui
        target=next((ui.Workspace.get_window(name) for name in ('Property','Stage')
                     if ui.Workspace.get_window(name)),None)
        if self.window.visible and target:
            self.window.dock_in(target, ui.DockPosition.SAME)
            self.window.focus()

    @property
    def docked(self):return self.window.docked

    def _slider(self, ui, key, spec):
        title, default, low, high, integer, units = spec
        ui.Label(f'{title} ({units})', height=20)
        model=(ui.SimpleIntModel if integer else ui.SimpleFloatModel)(self.settings.pending[key])
        self.models[key]=model
        with ui.HStack(height=24):
            slider=(ui.IntSlider if integer else ui.FloatSlider)(model=model, min=low, max=high, width=ui.Fraction(1))
            field=(ui.IntField if integer else ui.FloatField)(model=model, width=68)
            ui.Button('R', width=26, tooltip='Reset this setting', clicked_fn=lambda: model.set_value(default))
        self.widgets.extend((slider,field))
        def changed(m):
            if self._updating:return
            value=m.as_int if integer else round(m.as_float,4)
            self.set_setting(key,value)
        self._subscriptions.append(model.subscribe_value_changed_fn(changed))

    def set_setting(self, key, value):
        try:self.settings.edit(key,value)
        except ValueError as error:
            self.status.text=str(error)
            if key in self.models:
                self._updating=True
                self.models[key].set_value(self.settings.pending[key])
                self._updating=False
            return
        self.refresh_status()

    def refresh_status(self):
        s=self.settings
        if s.locked:
            self.status.text='Restart queued. Completing safe braking/hold...\nThe new run opens after Isaac shuts down cleanly.'
        else:
            dirty=[k for k in RESTART_CONTROLS if k not in s.live_keys and s.active[k]!=s.pending[k]]
            self.status.text=(('Pending: '+', '.join(dirty)) if dirty else 'All next-run settings match the active run.') + (
                f"\nActive: {s.active['traffic_count']} cars | {s.active['ego_mph']:g} mph ego | "
                f"{s.active['traffic_mph']:g} mph traffic\nNext run display: {s.pending['render_hz']} Hz, {s.pending['resolution_percent']}%")
            if self.live_driving:
                self.status.text=(self.live_status or 'Live driving controls ready.')+f"\nRequested: {s.pending['traffic_count']} cars | {s.pending['ego_mph']:g} mph ego | {s.pending['traffic_mph']:g} mph traffic\n"+('Rendering pending restart.' if dirty else 'Only rendering settings need restart.')
        self.apply_button.enabled=not s.locked
        for widget in self.widgets:widget.enabled=not s.locked

    def reset_defaults(self):
        if self.settings.locked:return
        for key,value in defaults().items():
            if key in self.models:self.models[key].set_value(value)
            else:self.set_setting(key,value)

    def set_camera(self,mode):self.state['camera']=mode
    def toggle_path(self):self.state['show_path']=not self.state.get('show_path',True)
    def toggle_pause(self):
        if self.settings.locked:return
        self.state['paused']=not self.state['paused']
        self.pause_button.text='Resume' if self.state['paused'] else 'Pause'

    def request_restart(self):
        self.settings.request_restart()
        self.state['paused']=False
        self.refresh_status()

    def smoke_action(self):
        """Exercise actual native widget model callbacks, not mouse latency."""
        self.models['camera_distance_m'].set_value(30.)
        self.models['camera_height_m'].set_value(12.)
        self.models['path_horizon_m'].set_value(80.)
        self.models['statistics_hz'].set_value(2)
        self.models['wheel_distance_m'].set_value(100.)
        self.models['traffic_count'].set_value(12)
        self.models['ego_mph'].set_value(25.)
        self.models['traffic_mph'].set_value(12.)
        self.models['following_s'].set_value(2.4)
        self.request_restart()

    def live_smoke_action(self,tick,hz):
        """Three native-callback transitions in one process; no relaunch."""
        milestones=((5,12,25.,12.,2.4),(35,40,35.,20.,2.0),(65,12,15.,10.,3.0))
        if self.smoke_stage>=len(milestones):return
        second,count,ego,traffic,gap=milestones[self.smoke_stage]
        if tick<second*hz:return
        # Respect the nonreactive-traffic speed relationship during edits.
        for key,value in (('traffic_count',count),('traffic_mph',min(traffic,self.settings.pending['traffic_mph'])),
                          ('ego_mph',ego),('traffic_mph',traffic),('following_s',gap)):
            self.models[key].set_value(value)
        self.smoke_stage+=1

    def close(self):
        self._subscriptions.clear()
        self.window.visible=False
        self.window.destroy()
        self.models.clear();self.widgets.clear();self.future_widgets.clear()
