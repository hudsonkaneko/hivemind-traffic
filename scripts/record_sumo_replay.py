"""Record the existing trained SUMO policy, including background vehicles."""
import argparse
import hashlib
import json
import sys
from pathlib import Path

parser = argparse.ArgumentParser()
parser.add_argument('--source-repo', type=Path, required=True)
parser.add_argument('--checkpoint', type=Path, required=True)
parser.add_argument('--seed', type=int, default=2001)
args = parser.parse_args()
sys.dont_write_bytecode = True
sys.path.insert(0, str(args.source_repo.resolve()))
from environments.highway_parallel_env import HighwayParallelEnv, DEFAULT_CONFIG
from policies.ppo import CheckpointPolicy
root = Path(__file__).resolve().parents[1]
output = root / 'outputs' / 'sumo_replay'
output.mkdir(parents=True, exist_ok=True)
env = HighwayParallelEnv()
policy = CheckpointPolicy(args.checkpoint)
policy.reset(args.seed)
frames = []
try:
    observations, _ = env.reset(seed=args.seed)
    while True:
        backend = env._require_backend()
        connection = backend._require_connection()
        vehicles = []
        for vehicle_id in backend.active_vehicle_ids:
            x, y = connection.vehicle.getPosition(vehicle_id)
            vehicles.append(dict(id=vehicle_id, type=connection.vehicle.getTypeID(vehicle_id),
                                 x=x, y=y, heading=connection.vehicle.getAngle(vehicle_id),
                                 speed=connection.vehicle.getSpeed(vehicle_id),
                                 length=connection.vehicle.getLength(vehicle_id),
                                 width=connection.vehicle.getWidth(vehicle_id)))
        frames.append(dict(time=connection.simulation.getTime(), vehicles=vehicles))
        if not env.agents:
            break
        observations, _, _, _, _ = env.step(policy.act(observations))
finally:
    env.close()
network = Path(DEFAULT_CONFIG).parent / 'network.net.xml'
(output / 'network.net.xml').write_bytes(network.read_bytes())
payload = dict(seed=args.seed, checkpoint_sha256=hashlib.sha256(args.checkpoint.read_bytes()).hexdigest(),
               network_sha256=hashlib.sha256(network.read_bytes()).hexdigest(), source_timestep=0.2,
               policy='shared PPO, no communication', frames=frames)
(output / 'recording.json').write_text(json.dumps(payload))
print(json.dumps(dict(frames=len(frames), output=str(output))))
