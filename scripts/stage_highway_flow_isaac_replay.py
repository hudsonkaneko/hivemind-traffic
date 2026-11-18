"""Stage a recorded highway-flow episode as input for scripts/render_i110_merge_replay.py (no simulation).

The Isaac replay renderer expects recording.json and network.net.xml side by side,
with policy and merge-event metadata in the recording. This copies the episode's
network, and writes the recording with its frames unchanged plus those metadata
keys taken from the recording's summary.json. A manifest records every hash.
"""
import argparse
import hashlib
import json
import shutil
from pathlib import Path

ADDED_KEYS = ['training', 'optimizer_steps', 'controller', 'policy_mode', 'policy', 'policy_sha256',
              'checkpoint_sha256', 'merge_events']


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def stage(recording_directory, output, car='agent_0', policy_mode=None):
    base, output = Path(recording_directory), Path(output)
    source = base / 'episode/recording.json'
    networks = sorted((base / 'episode').glob('episode-*/network.net.xml'))
    if len(networks) != 1:
        raise ValueError('Expected exactly one recorded episode network')
    summary = json.loads((base / 'summary.json').read_text(encoding='utf-8'))
    if summary.get('training') is not False or summary.get('deterministic') is not True:
        raise ValueError('Only a deterministic, non-training recording can be staged')
    merge = summary['stable_merge'][car]
    data = json.loads(source.read_text(encoding='utf-8'))
    frames = json.dumps(data['frames'])
    data.update(training=False, optimizer_steps=0, controller='frozen_learned',
                policy_mode=policy_mode or 'deterministic policy mean, no exploration noise',
                policy=summary['policy'], policy_sha256=summary['policy_sha256'],
                checkpoint_sha256=summary['policy_sha256'],
                merge_events=[dict(time=merge['entry_time_s'], agent=car, event='merge entry'),
                              dict(time=merge['lateral_merge_complete_time_s'], agent=car, event='lateral merge complete')])
    if json.dumps(data['frames']) != frames:
        raise RuntimeError('Recorded frames changed while staging')
    output.mkdir(parents=True, exist_ok=False)
    (output / 'recording.json').write_text(json.dumps(data), encoding='utf-8')
    shutil.copy2(networks[0], output / 'network.net.xml')
    manifest = dict(schema='round9_isaac_replay_input_v01',
                    note='Frames copied unchanged; only metadata keys added for the Isaac replay renderer',
                    source_recording=source.as_posix(), source_recording_sha256=sha256(source),
                    source_network=networks[0].as_posix(), network_sha256=sha256(networks[0]),
                    staged_recording_sha256=sha256(output / 'recording.json'), added_keys=ADDED_KEYS)
    (output / 'manifest.json').write_text(json.dumps(manifest, indent=2) + '\n', encoding='utf-8')
    return manifest


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('recording', type=Path, help='recording output directory holding summary.json and episode/')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--car', default='agent_0')
    parser.add_argument('--policy-mode')
    args = parser.parse_args(argv)
    print(json.dumps(stage(args.recording, args.output, args.car, args.policy_mode), indent=1))


if __name__ == '__main__':
    main()
