"""Read-only ramp exposure diagnostics; timestep flags are not gap opportunities."""
import json
from traci import constants as tc


def lane_flags(state):
    known = state >= 0 and state != tc.LCA_UNKNOWN
    return dict(known=known,
        wants_left=bool(state & tc.LCA_LEFT) if known else None,
        blocked=bool(state & tc.LCA_BLOCKED) if known else None)


class InteractionRecorder:
    def __init__(self, path, dt=.1):
        self.file = path.open('x', encoding='utf-8')
        self.dt = dt
        self.ramps = {f'ramp_{i:03}':dict(entry_sb2_s=None,first_through_lane_s=None,
                      left_desire_steps=0,blocked_desire_steps=0,unknown_lane_state_steps=0,
                      stopped_on_ramp_steps=0) for i in range(8)}
        self.relations = set()
        self.relation_steps = 0

    def observe(self, conn, time, states):
        rows = {}
        relationships = []
        for vid in sorted(self.ramps):
            if vid not in states:
                continue
            s = states[vid]
            row = {k:s[k] for k in ('road_id','lane_index','lane_position','speed','acceleration','leader','waiting_time')}
            record = self.ramps[vid]
            if s['road_id']=='mlk_on' and s['speed']<.1:
                record['stopped_on_ramp_steps'] += 1
            if s['road_id']=='sb2' and record['entry_sb2_s'] is None:
                record['entry_sb2_s'] = time
            if s['road_id'] in {'sb2','sb3'}:
                if s['lane_index']>=1 and record['first_through_lane_s'] is None:
                    record['first_through_lane_s'] = time
                if s['lane_index']==0:
                    model, effective = conn.vehicle.getLaneChangeState(vid,1)
                    flags = lane_flags(model)
                    row['left_change_state'] = dict(model=model,effective=effective,**flags)
                    record['unknown_lane_state_steps'] += not flags['known']
                    record['left_desire_steps'] += flags['wants_left'] is True
                    record['blocked_desire_steps'] += flags['wants_left'] is True and flags['blocked'] is True
            rows[vid] = row
        for follower in ['agent_0',*sorted(self.ramps)]:
            s = states.get(follower)
            if not s or s['road_id'] not in {'sb2','sb3'} or not s.get('leader'):
                continue
            leader,gap = s['leader']
            if ((follower=='agent_0' and leader in self.ramps) or (follower in self.ramps and leader=='agent_0')):
                if leader not in states or gap+2.5>120:
                    continue
                relationships.append(dict(follower=follower,leader=leader,bumper_gap_m=gap+2.5))
                self.relations.add((follower,leader))
        self.relation_steps += bool(relationships)
        ego = states.get('agent_0')
        frame = dict(time_s=time,ramps=rows,agent_0={k:ego[k] for k in
            ('road_id','lane_index','lane_position','speed','acceleration','leader')} if ego else None,
            direct_relationships=relationships)
        self.file.write(json.dumps(frame,sort_keys=True,allow_nan=False)+'\n')

    def close(self):
        self.file.close()

    def summary(self):
        return dict(ramps=self.ramps,direct_relationships=[list(x) for x in sorted(self.relations)],
            interaction_steps=self.relation_steps,interaction_observed=self.relation_steps>0,
            limits='Native last-step desire/block bitsets are timestep observations, not independent rejected opportunities. Junction waiting is separate from lateral blocking. Exact state, no human intent or calibrated gap model.')
