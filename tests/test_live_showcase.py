import math
import json
from contextlib import nullcontext
from types import SimpleNamespace
from unittest.mock import patch, Mock

import numpy as np
import unittest
from dataclasses import replace

from experiments.loop_showcase_config import make_config, fleet_specs, validate_config
from experiments.showcase_controls import defaults, SidebarState
from traffic.live_showcase import LiveTraffic,LiveShowcaseController,assess_live_smoke
from traffic.loop_showcase_control import ObjectTrack, signed_station_gap
from traffic.bypass_validation import OrientedBox, box_clearance
from usd.showcase_fleet import LENGTH_M, WIDTH_M
from usd.live_showcase_fleet import LiveKinematicFleet


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


    def protected_pool(self, *, active_stations=(200., 1000., 1800., 2600.), dormant_lane=0):
        settings={**defaults(),'traffic_count':4}
        specs=[dict(vehicle_id=f'background_{i:03d}',lane=dormant_lane,station_m=10.,speed_m_s=6.7056)
               for i in range(80)]
        for index,station in zip((0,20,40,60),active_stations):
            specs[index].update(lane=0,station_m=station)
        return LiveTraffic(specs,settings)

    def request_one(self,live):
        live.configure({**live.settings,'traffic_count':len(live.active)+1})

    def test_safe_existing_phase_keeps_original_furthest_slot_priority(self):
        live=self.make(4)
        expected=max((abs(signed_station_gap(0.,s.station_m)),key)
                     for key,s in live.specs.items() if key not in live.active)[1]
        self.request_one(live)
        event=live.reconcile(0,(500.,0.))[0]
        self.assertEqual(event['slot'],expected)
        self.assertEqual(event['placement_method'],'existing_phase')
        self.assertEqual(event['old_phase_offset_m'],0.)
        self.assertEqual(event['new_phase_offset_m'],0.)

    def test_protected_phases_admit_only_one_remote_dormant_slot(self):
        live=self.protected_pool();self.request_one(live)
        before={key:live.pose(key) for key in live.active}
        original_specs=dict(live.specs);generations=dict(live.generations)
        event=live.reconcile(0,(500.,0.))[0]
        self.assertEqual(event['slot'],'background_079')
        self.assertEqual(event['placement_method'],'remote_gap_midpoint')
        self.assertEqual(event['actual_station_m'],1400.)
        self.assertAlmostEqual(event['actual_arc_gap_m'],1400.)
        self.assertEqual(event['old_phase_offset_m'],0.)
        self.assertAlmostEqual(event['new_phase_offset_m'],1390.)
        self.assertEqual(event['nearest_front_station_gap_m'],400.)
        self.assertEqual(event['nearest_rear_station_gap_m'],400.)
        self.assertEqual(event['entry_required_station_gap_m'],40.)
        self.assertEqual(event['entry_required_clearance_m'],.8)
        self.assertEqual(live.specs,original_specs)
        for key,pose in before.items():
            self.assertEqual(live.pose(key),pose)
            self.assertEqual(live.phase_offsets_m[key],0.)
            self.assertEqual(live.generations[key],generations[key])
        self.assertEqual(len(live.active),5)
        json.dumps(event,allow_nan=False)
        actual=live.pose(event['slot'])
        candidate=OrientedBox(*actual['position_m'][:2],actual['yaw_rad'],LENGTH_M,WIDTH_M)
        distances=[box_clearance(candidate,OrientedBox(*pose['position_m'][:2],pose['yaw_rad'],LENGTH_M,WIDTH_M))['clearance_m']
                   for pose in before.values()]
        self.assertAlmostEqual(event['minimum_background_clearance_m'],min(distances))
        self.assertGreaterEqual(min(distances),.8)

    def test_spacing_covers_future_maximum_speed_and_following_gap(self):
        live=self.make(4)
        # Admission safety must not disappear when a low-speed entry is later
        # ramped to the maximum allowed traffic speed and longest following gap.
        for lane in range(4):
            scale=(500+lane*3.7)/500
            allowed_center=live.entry_min_station_gap_m*scale
            required=25*.44704*scale*3+LENGTH_M+live.entry_min_clearance_m
            self.assertGreaterEqual(allowed_center,required)

    def test_existing_phase_is_rechecked_against_relocated_active_cars(self):
        for retained_offset in (False,True):
            with self.subTest(retained_offset=retained_offset):
                live=self.protected_pool()
                # Represent an earlier accepted relocation, retaining its static
                # original layout. A later dormant entry must see this location.
                live.phase_offsets_m['background_000']=1000.
                key='background_079'
                if retained_offset:
                    live.phase_offsets_m[key]=1190.
                else:
                    live.specs[key]=replace(live.specs[key],station_m=1200.)
                self.request_one(live)
                before=live.pose('background_000')
                event=live.reconcile(0,(500.,0.))[0]
                self.assertEqual(event['slot'],key)
                self.assertEqual(event['placement_method'],'remote_gap_midpoint')
                self.assertNotAlmostEqual(event['actual_station_m'],1200.)
                self.assertEqual(live.pose('background_000'),before)
                self.assertGreaterEqual(abs(signed_station_gap(1200.,event['actual_station_m'])),40.)

    def test_no_admissible_gap_is_pending_and_failed_search_is_throttled(self):
        settings={**defaults(),'traffic_count':79}
        specs=[dict(vehicle_id=f'background_{i:03d}',lane=0,
                    station_m=i*math.tau*500/80,speed_m_s=6.7056) for i in range(80)]
        live=LiveTraffic(specs,settings);self.request_one(live)
        original=dict(live.phase_offsets_m);active=set(live.active)
        ego=(500*math.cos(3.),500*math.sin(3.))
        with patch.object(live,'_entry_placement',wraps=live._entry_placement) as search:
            with patch.object(live,'_entry_geometry',wraps=live._entry_geometry) as checks:
                for tick in range(30):self.assertEqual(live.reconcile(tick,ego),[])
                self.assertEqual(search.call_count,1)
                self.assertLessEqual(checks.call_count,160)
                self.assertEqual(live.reconcile(30,ego),[])
                self.assertEqual(search.call_count,2)
        self.assertEqual(live.active,active)
        self.assertEqual(live.phase_offsets_m,original)
        self.assertEqual(live.events,[])
        self.assertEqual(live.pending_lifecycle,[])

    def test_gap_midpoint_search_handles_seam_and_ties_deterministically(self):
        length=math.tau*500
        # The wraparound gap is deliberately larger than all other gaps.
        positions=(400.,1100.,1800.,2500.)
        outputs=[]
        for _ in range(2):
            live=self.protected_pool(active_stations=positions)
            self.request_one(live)
            ego_station=length/2
            for key in live.specs:
                if key not in live.active:
                    live.phase_offsets_m[key]=ego_station-live.specs[key].station_m
            event=live.reconcile(0,(500*math.cos(ego_station/500),500*math.sin(ego_station/500)))[0]
            outputs.append(event)
            self.assertEqual(event['placement_method'],'remote_gap_midpoint')
            self.assertAlmostEqual(event['actual_station_m'],(2500.+length+400.)/2%length)
            self.assertGreaterEqual(event['nearest_front_station_gap_m'],40.)
            self.assertGreaterEqual(event['nearest_rear_station_gap_m'],40.)
            self.assertEqual(event['slot'],'background_079')
        self.assertEqual(outputs[0],outputs[1])

    def test_empty_lane_uses_remote_antipode_and_null_neighbor_gaps(self):
        live=self.protected_pool(dormant_lane=2);self.request_one(live)
        event=live.reconcile(0,(500.,0.))[0]
        self.assertEqual(event['placement_method'],'empty_lane_opposite')
        self.assertAlmostEqual(event['actual_station_m'],math.pi*500)
        self.assertIsNone(event['nearest_front_station_gap_m'])
        self.assertIsNone(event['nearest_rear_station_gap_m'])
        self.assertGreaterEqual(event['minimum_background_clearance_m'],.8)
        json.dumps(event,allow_nan=False)

    def test_exact_clearance_cannot_be_bypassed_by_nominal_or_fallback_entry(self):
        live=self.make(4);self.request_one(live)
        with patch('traffic.live_showcase.nearest_clearance',return_value={'clearance_m':.799}) as exact:
            self.assertEqual(live.reconcile(0,(500.,0.)),[])
            self.assertGreater(exact.call_count,0)
            self.assertLessEqual(exact.call_count,160)
            for args,_ in exact.call_args_list:
                self.assertEqual((args[0].length_m,args[0].width_m),(4.8,1.9))
                self.assertEqual(len(args[1]),4)
        self.assertEqual(len(live.active),4)
        self.assertTrue(all(value==0 for value in live.phase_offsets_m.values()))

    def test_exit_uses_actual_phase_and_reentry_advances_generation(self):
        live=self.protected_pool();self.request_one(live)
        entered=live.reconcile(0,(500.,0.))[0];key=entered['slot']
        live.configure({**live.settings,'traffic_count':4})
        self.assertEqual(live.reconcile(29,(500.,0.)),[])
        exited=live.reconcile(30,(500.,0.))[0]
        # Nominal station10 is protected; only the actual remote1400 placement
        # makes this slot the furthest eligible removal.
        self.assertEqual(exited['slot'],key)
        self.assertAlmostEqual(exited['actual_station_m'],1400.)
        self.assertEqual(exited['old_phase_offset_m'],exited['new_phase_offset_m'])
        self.assertEqual(exited['vehicle_id'],entered['vehicle_id'])
        self.assertGreater(math.hypot(*live.pose(key)['position_m'][:2]),10000)
        self.request_one(live)
        again=live.reconcile(60,(500.,0.))[0]
        self.assertEqual(again['slot'],key)
        self.assertEqual(again['placement_method'],'existing_phase')
        self.assertNotEqual(again['vehicle_id'],entered['vehicle_id'])
        self.assertEqual(live.generations[key],2)
        self.assertEqual([e['tick'] for e in live.events],[0,30,60])

    def test_maneuver_and_stopping_freeze_admission_without_phase_changes(self):
        live=self.protected_pool();self.request_one(live)
        before=dict(live.phase_offsets_m)
        self.assertEqual(live.reconcile(0,(500.,0.),maneuver=True),[])
        live.stopping=True
        self.assertEqual(live.reconcile(30,(500.,0.)),[])
        self.assertEqual(live.phase_offsets_m,before)
        self.assertEqual(live.events,[])
        live.stopping=False
        self.assertEqual(len(live.reconcile(60,(500.,0.))),1)

    def test_relocated_entry_next_step_preserves_common_angular_motion(self):
        live=self.protected_pool();self.request_one(live)
        event=live.reconcile(0,(500.,0.))[0];key=event['slot']
        stations={k:live.station(k) for k in live.active}
        poses={k:live.pose(k) for k in live.active}
        live.configure({**live.settings,'traffic_mph':25.})
        speed_before=live.speed_m_s;live.advance(1/120)
        advance=(speed_before+live.speed_m_s)/2/120
        for other in live.active:
            self.assertAlmostEqual(signed_station_gap(stations[other],live.station(other)),advance,places=10)
            scale=(500+live.specs[other].lane*3.7)/500
            self.assertAlmostEqual(math.dist(poses[other]['position_m'],live.pose(other)['position_m']),advance*scale,delta=1e-8)
        self.assertEqual(live.phase_offsets_m[key],event['new_phase_offset_m'])

    def test_fake_native_adapter_teleports_only_entering_index_then_targets_continuously(self):
        # This exercises the unchanged adapter with reversed native ordering;
        # fake views prove index/target contracts, not actual PhysX behavior.
        live=self.protected_pool();self.request_one(live)
        fleet=LiveKinematicFleet.__new__(LiveKinematicFleet)
        fleet.live=live;fleet.maneuver=False
        fleet._ordered_specs=list(reversed(list(live.specs.values())))
        fleet._target_time_s=0.;fleet._target_count=0
        fleet._float_tensor=lambda value:value
        fleet._index_tensor=lambda value:value
        fleet._invalidate_caches=Mock()
        values=np.asarray([(*live.pose(s.vehicle_id)['position_m'],*live.pose(s.vehicle_id)['quaternion_xyzw'])
                           for s in fleet._ordered_specs],dtype=np.float32)
        original=values.copy();teleports=[];targets=[]
        def teleport(data,indices):
            values[indices]=data[indices];teleports.append(indices.copy())
        fleet._view=SimpleNamespace(set_transforms=teleport,
            set_kinematic_targets=lambda data,indices:targets.append((data.copy(),indices.copy())))
        fleet._measured=lambda:(values.copy(),np.zeros((80,6),dtype=np.float32))
        fleet.stage=SimpleNamespace(GetPrimAtPath=lambda path:path);fleet.dynamic_layer=object()
        visibility=[]
        usd=SimpleNamespace(EditContext=lambda *args:nullcontext())
        geom=SimpleNamespace(Imageable=lambda path:SimpleNamespace(GetVisibilityAttr=lambda:
            SimpleNamespace(Set=lambda value:visibility.append((path,value)))))
        with patch.dict('sys.modules',{'pxr':SimpleNamespace(Usd=usd,UsdGeom=geom)}):
            fleet.reconcile(0,(500.,0.))
        event=live.events[-1];index=next(i for i,s in enumerate(fleet._ordered_specs) if s.vehicle_id==event['slot'])
        self.assertEqual([indices.tolist() for indices in teleports],[[index]])
        np.testing.assert_array_equal(np.delete(values,index,axis=0),np.delete(original,index,axis=0))
        self.assertEqual(visibility,[(live.specs[event['slot']].render_path,'inherited')])
        entry=values[index].copy()
        fleet.update(1/120)
        self.assertEqual(len(targets),1)
        self.assertEqual(targets[0][1].tolist(),list(range(80)))
        self.assertLess(math.dist(entry[:3],targets[0][0][index,:3]),.1)
        self.assertGreater(math.dist(entry[:3],targets[0][0][index,:3]),.01)
        self.assertEqual(len(teleports),1)
        self.assertEqual(fleet._target_count,1)


if __name__=='__main__':unittest.main()
