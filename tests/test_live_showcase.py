import math
import unittest
from dataclasses import replace

from experiments.loop_showcase_config import make_config, fleet_specs, validate_config
from experiments.showcase_controls import defaults, SidebarState
from traffic.live_showcase import LiveTraffic,LiveShowcaseController,assess_live_smoke
from traffic.loop_showcase_control import ObjectTrack


class LiveTrafficTests(unittest.TestCase):
    def test_smoke_requires_actual_changes_not_only_successful_callbacks(self):
        with self.assertRaises(ValueError):make_config('v09',control_smoke=True,drive_s=45)
        with self.assertRaises(ValueError):make_config('v09',control_smoke=True,gui=False)
        validate_config(make_config('v09',control_smoke=True,drive_s=90,gui=False,panel_hidden=True))
        rows=[dict(sim_time_s=t,active_background_count=n) for t,n in ((34,12),(64,40),(89,12))]
        events=[dict(type='enter'),dict(type='exit')]
        self.assertTrue(assess_live_smoke(rows,events,3,False)['passed'])
        self.assertFalse(assess_live_smoke(rows,events,3,True)['passed'])
        self.assertFalse(assess_live_smoke(rows,events,2,False)['passed'])
        self.assertFalse(assess_live_smoke(rows,[],3,False)['passed'])
        self.assertFalse(assess_live_smoke(rows[:-1],events,3,False)['passed'])
        rows[1]['active_background_count']=12
        self.assertFalse(assess_live_smoke(rows,events,3,False)['passed'])

    def make(self,count=80):
        settings={**defaults(),'traffic_count':count}
        cfg=make_config('v09',controls=settings)
        validate_config(cfg)
        return LiveTraffic(fleet_specs(cfg),settings)

    def test_live_mode_does_not_modify_v08(self):
        legacy=SidebarState(defaults());live=SidebarState(defaults(),live_driving=True)
        for state in (legacy,live):state.edit('traffic_count',12);state.consume_live(1)
        self.assertEqual(legacy.active['traffic_count'],80)
        self.assertEqual(live.active['traffic_count'],12)
        self.assertFalse(live.dirty)
        live.edit('render_hz',20);live.consume_live(2)
        self.assertTrue(live.dirty);self.assertEqual(live.active['render_hz'],30)

    def test_every_initial_count_and_dormant_parking(self):
        for n in range(4,81):
            live=self.make(n)
            self.assertEqual(len(live.specs),80);self.assertEqual(len(live.active),n)
            for key in live.specs:
                pose=live.pose(key)
                if key not in live.active:self.assertGreater(math.hypot(*pose['position_m'][:2]),10000)

    def test_density_bounded_remote_and_reversible_without_id_reuse(self):
        live=self.make();old={k:live.identity(k) for k in live.active}
        live.configure({**live.settings,'traffic_count':4})
        for tick in range(2400):
            changes=live.reconcile(tick,(500,0))
            self.assertLessEqual(len(changes),1)
            if changes:
                event=changes[0];spec=live.specs[event['slot']]
                self.assertGreaterEqual(abs((spec.station_m+math.pi*500)%(math.tau*500)-math.pi*500),160)
        # Some nearby cars may remain protected until the ego/traffic advances.
        self.assertGreaterEqual(len(live.active),4)
        self.assertLess(len(live.active),12)
        live.configure({**live.settings,'traffic_count':80})
        for tick in range(2400,4900):live.reconcile(tick,(500,0))
        self.assertEqual(len(live.active),80)
        entered=[e for e in live.events if e['type']=='enter']
        self.assertTrue(entered)
        self.assertTrue(all(e['vehicle_id']!=old[e['slot']] for e in entered))
        self.assertTrue(all(b['tick']-a['tick']>=30 for a,b in zip(live.events,live.events[1:])))

    def test_speed_ramp_continuity_and_rundown(self):
        live=self.make(12);key=next(iter(live.active))
        live.configure({**live.settings,'traffic_mph':20.})
        before=live.pose(key);old=live.speed_m_s
        live.advance(1/120)
        self.assertLessEqual(live.speed_m_s-old,.5/120+1e-12)
        self.assertLess(math.dist(before['position_m'],live.pose(key)['position_m']),.1)
        live.prepare_control(1/60);self.assertTrue(live.ramping)
        old=live.speed_m_s;live.advance(2/120,maneuver=True)
        self.assertEqual(live.speed_m_s,old)
        self.assertEqual(live.reconcile(100,(500,0),maneuver=True),[])
        live.stopping=True
        for i in range(3,603):live.advance(i/120)
        self.assertEqual(live.speed_m_s,0.)
        self.assertEqual(live.reconcile(700,(500,0)),[])
        with self.assertRaises(ValueError):live.advance(10.)

    def test_controller_requires_explicit_admission(self):
        c=LiveShowcaseController('episode','ego',initial_lane=1,optimized=True)
        a=ObjectTrack('episode','background_000_g001',12,500.,70.,0.,5.)
        b=replace(a,vehicle_id='background_001_g001')
        c._expected_ids={a.vehicle_id};c._previous_tracks={a.vehicle_id:a}
        with self.assertRaises(RuntimeError):c.accept_lifecycle([a,b],[{'type':'enter','vehicle_id':'wrong'}])
        c.accept_lifecycle([a,b],[{'type':'enter','vehicle_id':b.vehicle_id}])
        self.assertEqual(c._expected_ids,{a.vehicle_id,b.vehicle_id})
        self.assertEqual(c._previous_tracks[b.vehicle_id],b)
        c.changing_traffic_speed=True
        self.assertFalse(c._candidate_safe(None,None,None))
        c.changing_traffic_speed=False;c.traffic_transition_pending=True
        self.assertFalse(c._candidate_safe(None,None,None))


if __name__=='__main__':unittest.main()
