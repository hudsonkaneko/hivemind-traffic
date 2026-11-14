"""V06 Phase 0b: the follower secure gap SUMO lane changers require behind them.

Asks SUMO (traci.vehicle.getSecureGap) on a throwaway one-lane network for the
ACC and Krauss controlled vehicle types. No episode, no policy, no ledger charge.
"""
import json, subprocess, sys, tempfile
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from traffic.live_runtime import find_sumo
import traci

sumo = find_sumo()
netconvert = sumo.with_name('netconvert.exe')
d = Path(tempfile.mkdtemp())
(d/'n.nod.xml').write_text('<nodes><node id="a" x="0" y="0"/><node id="b" x="3000" y="0"/></nodes>')
(d/'e.edg.xml').write_text('<edges><edge id="e" from="a" to="b" numLanes="1" speed="30"/></edges>')
subprocess.run([str(netconvert), '-n', str(d/'n.nod.xml'), '-e', str(d/'e.edg.xml'), '-o', str(d/'net.xml')],
               check=True, capture_output=True)
common = 'length="4.8" minGap="2.5" emergencyDecel="9" maxSpeed="26.8224" speedDev="0" sigma="0" accel="2" decel="4.5"'
types = {'ACC tau1.5': 'carFollowModel="ACC" tau="1.5"'}
for tau in (1.5, 2.0, 3.0):
    types[f'Krauss tau{tau}'] = f'carFollowModel="Krauss" tau="{tau}"'
routes = '<routes>' + ''.join(f'<vType id="t{i}" {v} {common}/>' for i, v in enumerate(types.values()))
routes += '<route id="r" edges="e"/>' + ''.join(f'<vehicle id="v{i}" type="t{i}" route="r" depart="0" departPos="{100*i}"/>' for i in range(len(types))) + '</routes>'
(d/'r.rou.xml').write_text(routes)
traci.start([str(sumo), '-n', str(d/'net.xml'), '-r', str(d/'r.rou.xml'), '--no-step-log', 'true', '--no-warnings', 'true'])
traci.simulationStep()
cases = [('cut-in seen (AV 8.75, leader 19.05)', 8.75, 19.05), ('same speed 20', 20., 20.),
         ('AV 15, leader 20', 15., 20.), ('AV 20, leader 15', 20., 15.)]
out = {}
for i, name in enumerate(types):
    out[name] = {c: round(traci.vehicle.getSecureGap(f'v{i}', v, vl, 4.5), 2) for c, v, vl in cases}
traci.close()
print(json.dumps(out, indent=1))
