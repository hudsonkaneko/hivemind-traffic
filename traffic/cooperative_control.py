"""Scripted local lidar controller with optional timestamped intentions.

No neighbor truth, obstacle truth, shared tracker, or central joint action enters
this API. The fixture provides lane centres and a home lane, not obstacle poses.
"""
from dataclasses import dataclass
import math
from traffic.lidar_avoidance import LANES
from traffic.lidar_control import Clearance, target_speed


@dataclass(frozen=True)
class FleetDecision:
    speed: float
    lane_request: int | None
    intended_lane: int | None
    phase: str
    reason: str
    clearance: float
    target_clear: bool
    observed_obstacle: tuple | None


class CooperativeController:
    def __init__(self,vehicle_id,home_lane,cruise=6.):
        self.vehicle_id=vehicle_id;self.home_lane=home_lane;self.cruise=cruise
        self.phase='approach' if home_lane==0 else 'cruise'
        self.pass_end=None;self.obstacle=None

    def step(self,ego,tracks,messages,now,*,fresh):
        if not fresh:
            speed,reason=target_speed(Clearance(0,False,0),ego['speed'],fresh=False)
            return FleetDecision(speed,None,None,self.phase,reason,0.,False,None)
        # Bound partial-surface uncertainty explicitly. Tracker IDs remain local.
        boxes=[]
        for track in tracks:
            age=now-track.last_seen
            if not 0<=age<=.5:continue
            dt=max(0.,now-track.state_time)
            # The tracker already predicts to state_time. Do not extrapolate
            # a second time over its entire age or time since last detection.
            longitudinal_pad=2.5+min(2.,track.motion_uncertainty)
            lateral_pad=.3+min(.4,track.motion_uncertainty)
            boxes.append((track.xmin+track.vx*dt-longitudinal_pad,track.xmax+track.vx*dt+longitudinal_pad,
                          track.ymin+track.vy*dt-lateral_pad,track.ymax+track.vy*dt+lateral_pad,track))
        def overlaps_lane(b,lane):return b[2]<LANES[lane]+1.1 and b[3]>LANES[lane]-1.1
        def forward(lanes):
            ahead=[max(0.,b[0]-ego['x']) for b in boxes if b[1]>ego['x'] and any(overlaps_lane(b,l) for l in lanes)]
            return min(ahead,default=80.)
        def clear(lane):
            for b in boxes:
                track=b[4]
                if overlaps_lane(b,lane) and (track.observed_count<2 or track.motion_uncertainty>2.):
                    if b[1]>ego['x']-30 and b[0]<ego['x']+40:return False
                # Intersect continuous relative-motion intervals, not a few
                # sampled horizons (a fast car could cross between samples).
                low,high=0.,5.
                for start,velocity,limit in (
                    (b[0]-ego['x'],track.vx-ego['speed'],6.),
                    (ego['x']-b[1],ego['speed']-track.vx,11.),
                    (b[2],track.vy,LANES[lane]+1.1),
                    (-b[3],-track.vy,-LANES[lane]+1.1)):
                    if abs(velocity)<1e-9:
                        if start>=limit:low,high=1.,0.;break
                    elif velocity>0:high=min(high,(limit-start)/velocity)
                    else:low=max(low,(limit-start)/velocity)
                if low<high:return False
            return True
        request=None;intent=None;reason='lidar-cruise';desired_cruise=self.cruise
        target_clear=clear(1-self.home_lane)
        if self.home_lane==0:
            candidates=[b for b in boxes if overlaps_lane(b,0) and b[0]>ego['x']+3 and b[0]-ego['x']<70
                and b[4].observed_count>=3 and abs(b[4].vx)<1.]
            if self.phase in ('approach','requesting') and candidates:
                nearest=min(candidates,key=lambda b:b[0]);self.phase='requesting'
                self.pass_end=max(self.pass_end or -math.inf,nearest[1]+2.)
                self.obstacle=tuple(float(v) for v in nearest[:4])
            if self.phase=='requesting':
                intent=1;reason='request-gap'
                if clear(1):self.phase='outbound';request=1;reason='merge-after-local-clearance'
            elif self.phase=='outbound':
                intent=1;reason='changing-lane'
                if abs(ego['y']-LANES[1])<.02:self.phase='passing';intent=None
            elif self.phase=='passing':
                if self.pass_end is not None and ego['x']-5>self.pass_end+8 and clear(0):
                    self.phase='returning';request=0;intent=0;reason='return-after-clearance'
            elif self.phase=='returning':
                intent=0
                if abs(ego['y']-LANES[0])<.02:self.phase='complete';intent=None
        else:
            # A message can make this car yield; it never overrides lidar safety
            # or grants the other car permission to merge.
            for message in messages:
                payload=message.payload
                if (message.sender_id!=self.vehicle_id and message.sent_at<=now<message.expires_at
                    and payload.target_lane==self.home_lane and payload.phase in ('requesting','outbound')
                    and 0<payload.x-ego['x']<60):
                    desired_cruise=min(desired_cruise,2.);reason='yield-to-intent';break
            self.phase='yielding' if reason=='yield-to-intent' else 'cruise'
        lanes=[0,1] if self.phase in ('outbound','returning') else [1 if self.phase=='passing' else self.home_lane]
        distance=forward(lanes)
        speed,_=target_speed(Clearance(distance,True,len(boxes)),ego['speed'],cruise=desired_cruise)
        return FleetDecision(speed,request,intent,self.phase,reason,distance,target_clear,self.obstacle)
