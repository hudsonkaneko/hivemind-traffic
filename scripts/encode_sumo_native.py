"""Encode genuine SUMO viewport PNGs without synthetic graphics or overlays."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('recording', type=Path)
    parser.add_argument('--ffmpeg', type=Path, required=True)
    args = parser.parse_args()
    directory = args.recording.resolve()
    if not directory.is_relative_to(ROOT / 'outputs'):
        parser.error('Recording must be inside project outputs/')
    manifest = json.loads((directory / 'recording.json').read_text())
    if not manifest['renderer'].startswith('Native SUMO GUI'):
        parser.error('Requires actual native SUMO viewport frames')
    frames = sorted((directory/'frames').glob('*.png'))
    expected = [f'{i:05d}.png' for i in range(manifest['frames'])]
    if [p.name for p in frames] != expected or any(p.stat().st_size == 0 for p in frames):
        raise ValueError('Native frame sequence is incomplete')
    target = directory/'sumo-native-replay.mp4'
    command = [str(args.ffmpeg.resolve()), '-hide_banner', '-n', '-framerate', str(manifest['fps']),
        '-i', str(directory/'frames/%05d.png'), '-vf', 'pad=ceil(iw/2)*2:ceil(ih/2)*2',
        '-c:v', 'libx264', '-preset', 'fast', '-crf', '18', '-pix_fmt', 'yuv420p',
        '-movflags', '+faststart', str(target)]
    encoded = subprocess.run(command, capture_output=True, text=True, timeout=120)
    (directory/'encoding.log').write_text(encoded.stderr,encoding='utf-8')
    encoded.check_returncode()
    verified = subprocess.run([str(args.ffmpeg.resolve()), '-v','error','-i',str(target),
        '-progress','pipe:1','-f','null','-'],capture_output=True,text=True,timeout=120)
    verified.check_returncode()
    counts = [int(line.split('=')[1]) for line in verified.stdout.splitlines() if line.startswith('frame=')]
    if not counts or counts[-1] != len(frames):
        raise RuntimeError('Decoded video frame count does not match native screenshots')
    manifest['video'] = dict(file=target.name,decoded_frames=counts[-1],fps=manifest['fps'],
        sha256=hashlib.sha256(target.read_bytes()).hexdigest(),
        encoder_sha256=hashlib.sha256(args.ffmpeg.read_bytes()).hexdigest(),
        processing='H.264 encoding; at most one pixel padding per dimension; no overlays or reconstructed geometry')
    (directory/'recording.json').write_text(json.dumps(manifest,indent=2)+'\n',encoding='utf-8')
    print(target)


if __name__ == '__main__':
    main()
