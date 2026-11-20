"""Put the top-down video's speed/target graph under a 3D replay render (no simulation).

Inputs: the 3D render folder from scripts/render_i110_merge_replay.py (frames/*.png and
camera-poses.json, one pose per frame with its recording time), and the recorded episode
the render came from (recording.json and actions.jsonl.gz, as for
scripts/render_highway_flow_topdown.py). Each output frame is the unchanged 3D frame with a
heads-up line on top, and below it the same graph as the top-down video: the followed car's
speed (green) against its merge target (orange) with the gate's +-2 m/s band, the 8 s
stable-merge window shaded, the gate's merge events marked, and a line at the frame's time.

With --physics-log (the isaac/physics-log.json of scripts/record_highway_flow_v14_physics.py) a
second strip shows, per 0.1 s decision, the acceleration commanded to the physics car (after
v14's limits) against the acceleration PhysX actually produced, with ticks where the drive or
brake torque limit was reached and where a wheel left the deck.
"""
import argparse
import json
import subprocess
import sys
from pathlib import Path

from PIL import Image, ImageDraw

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from scripts.render_highway_flow_topdown import COLORS, MPH, font, load  # noqa: E402

GRAPH_H = 200          # same height as the top-down video's graph strip (y 520..720)
ACCEL_H = 170          # the commanded vs measured acceleration strip (with --physics-log)
SHORT_CAUSES = dict(native_realization_unattributed='realized ≠ commanded', comfort_rate_limit='comfort rate limit')


def physics_rows(path):
    """(time at the middle of the decision, commanded, measured, torque-limited, wheel off the deck) per decision."""
    rows = json.loads(path.read_text(encoding='utf-8'))['rows']
    return [(round(r['time'] - .05, 3), r['commanded_acceleration'], r['measured_acceleration'],
             bool(r['saturated_ticks']), not all(r['on_ground'])) for r in rows]


def draw_acceleration(draw, rows, now, top, px_of, t0, t1, small):
    """The acceleration strip between y = top and top + ACCEL_H."""
    rows = [r for r in rows if t0 - .05 <= r[0] <= t1 + .05]
    if not rows:
        return
    values = [v for r in rows for v in r[1:3]]
    lo, hi = max(-6., min(-1., min(values)) - .3), min(4., max(1., max(values)) + .3)
    py = lambda a: top + 28 + (hi - max(lo, min(hi, a))) / (hi - lo) * (ACCEL_H - 50)  # noqa: E731
    draw.text((24, top + 6), 'Acceleration, m/s²: commanded after v14 limits (orange) vs PhysX measured (green) — '
              'red ticks: torque limit reached, purple: a wheel off the deck', font=small, fill=COLORS['muted'])
    for a in range(int(lo), int(hi) + 1):
        draw.line([(px_of(t0), py(a)), (px_of(t1), py(a))], fill=(60, 72, 88) if a else (110, 124, 140), width=1)
        draw.text((24, py(a) - 9), f'{a:+d}' if a else '0', font=small, fill=COLORS['muted'])
    upto = [r for r in rows if r[0] <= now + 1e-6]
    for index, color, width in ((1, COLORS['target'], 2), (2, COLORS['ego'], 3)):
        points = [(px_of(r[0]), py(r[index])) for r in upto]
        if len(points) > 1:
            draw.line(points, fill=color, width=width)
    base = top + ACCEL_H - 18
    for r in upto:
        if r[3]:
            draw.line([(px_of(r[0]), base), (px_of(r[0]), base + 6)], fill=COLORS['marker'], width=2)
        if r[4]:
            draw.line([(px_of(r[0]), base + 8), (px_of(r[0]), base + 14)], fill='#b48cff', width=2)
    draw.line([(px_of(now), top + 22), (px_of(now), top + ACCEL_H - 2)], fill='white', width=1)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('render', type=Path, help='3D render folder with frames/ and camera-poses.json')
    parser.add_argument('episode', type=Path, help='recorded episode folder with recording.json and episode-*/')
    parser.add_argument('--car', default='agent_0')
    parser.add_argument('--title', required=True)
    parser.add_argument('--summary', type=Path, required=True, help="the recording's summary.json")
    parser.add_argument('--ffmpeg', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--physics-log', type=Path, help='isaac/physics-log.json of a physics recording')
    args = parser.parse_args(argv)
    car = args.car
    accel = physics_rows(args.physics_log) if args.physics_log else None
    recording, rows, _ = load(args.episode)
    poses = json.loads((args.render / 'camera-poses.json').read_text())
    images = sorted((args.render / 'frames').glob('*.png'))
    if len(images) != len(poses) or not images:
        raise ValueError('Each 3D frame needs exactly one camera pose with its time')
    frames = [f for f in recording['frames'] if any(v['id'] == car for v in f['vehicles'])]
    ego = {round(f['time'], 1): next(v for v in f['vehicles'] if v['id'] == car) for f in frames}
    first, last = round(poses[0]['time'], 1), round(poses[-1]['time'], 1)
    times = [t for t in sorted(ego) if first <= t <= last]   # the graph spans the rendered window
    speeds = [ego[t]['speed'] for t in times]
    decisions = {}
    for row in rows:
        a = (row.get('agents') or {}).get(car)
        if a:
            target = (a['v04_reward_audit'].get('pre_action') or {}).get('target') or {}
            decisions[round(row['time'], 1)] = dict(
                target=target.get('target_m_s') if target.get('eligible') else None,
                request=a['requested_acceleration'], overrides=a['override_causes'])
    targets = [decisions.get(t, {}).get('target') for t in times]
    merge = (json.loads(args.summary.read_text()).get('stable_merge') or {}).get(car) or {}
    entry, merged = merge.get('entry_time_s'), merge.get('lateral_merge_complete_time_s')
    passed = next((t for t in times if ego[t]['stable_merge_status'] == 'succeeded'), None)
    top = (max(max(speeds), max([x for x in targets if x is not None] or [0.])) + 2.) * MPH + 4   # band included

    width, height = Image.open(images[0]).size
    total = height + GRAPH_H + (ACCEL_H if accel else 0)
    t0, t1 = times[0], times[-1]
    px_of = lambda t: 70 + (t - t0) / (t1 - t0) * (width - 110)  # noqa: E731
    py_of = lambda mph: height + GRAPH_H - 20 - mph / top * 150  # noqa: E731
    big, mid, small = font(26, True), font(19), font(15)
    out = args.output.resolve()
    (out / 'frames').mkdir(parents=True, exist_ok=False)
    for index, (path, pose) in enumerate(zip(images, poses)):
        now = round(pose['time'], 1)
        if now not in ego:
            raise ValueError(f'The followed car is not in the recording at t = {now}')
        me, info = ego[now], decisions.get(now, {})
        image = Image.new('RGB', (width, total), COLORS['bg'])
        image.paste(Image.open(path).convert('RGB'), (0, 0))
        draw = ImageDraw.Draw(image, 'RGBA')
        draw.rectangle((0, 0, width, 88), fill=(15, 22, 32, 190))
        draw.text((24, 8), args.title, font=big, fill=COLORS['text'])
        phase = ('stable merge passed' if me['stable_merge_status'] == 'succeeded' else
                 'stable merge failed' if me['stable_merge_status'] == 'failed' else
                 'stable-merge window (8 s)' if merged is not None and now >= merged else
                 'merging into mainline' if entry is not None and now >= entry else 'on the ramp')
        target = info.get('target')
        parts = [f't = {now:4.1f} s', f'speed {me["speed"] * MPH:4.1f} mph',
                 f'merge target {target * MPH:4.1f} mph' if target is not None else 'merge target -',
                 f'request {info.get("request", 0.):+.2f} m/s²', phase]
        draw.text((24, 50), '   |   '.join(parts), font=mid, fill=COLORS['muted'])
        if info.get('overrides'):
            text = 'override: ' + ', '.join(SHORT_CAUSES.get(c, c) for c in info['overrides'])
            draw.text((width - 24 - draw.textlength(text, font=mid), 50), text, font=mid, fill=COLORS['marker'])
        # speed vs target timeline, as in the top-down video
        g = height
        draw.text((24, g + 8), 'Speed (green) vs merge target (orange, ±2 m/s band) — shaded: 8 s stable-merge window',
                  font=small, fill=COLORS['muted'])
        if merged is not None:
            draw.rectangle((px_of(merged), g + 25, px_of(min(merged + 8., t1)), g + 180), fill=COLORS['window'])
        for i in range(1, len(times)):
            a, b = targets[i - 1], targets[i]
            if a is not None and b is not None:
                x0, x1 = px_of(times[i - 1]), px_of(times[i])
                draw.polygon([(x0, py_of((a + 2) * MPH)), (x1, py_of((b + 2) * MPH)),
                              (x1, py_of((b - 2) * MPH)), (x0, py_of((a - 2) * MPH))], fill=COLORS['band'])
        for i in range(1, len(times)):
            if targets[i - 1] is not None and targets[i] is not None:
                draw.line([(px_of(times[i - 1]), py_of(targets[i - 1] * MPH)),
                           (px_of(times[i]), py_of(targets[i] * MPH))], fill=COLORS['target'], width=2)
        upto = [(px_of(t), py_of(s * MPH)) for t, s in zip(times, speeds) if t <= now]
        if len(upto) > 1:
            draw.line(upto, fill=COLORS['ego'], width=3)
        for mark, label, left in ((entry, 'merge entry', True), (merged, 'merged', False),
                                  (passed, 'stable merge passed', False)):
            if mark is not None:
                draw.line([(px_of(mark), g + 25), (px_of(mark), g + 180)], fill=COLORS['muted'], width=1)
                x = px_of(mark) - 4 - draw.textlength(label, font=small) if left else px_of(mark) + 4
                draw.text((x, g + 160), label, font=small, fill=COLORS['text'])
        draw.line([(px_of(now), g + 20), (px_of(now), g + 182)], fill='white', width=1)
        for mph in range(0, int(top) + 1, 10):
            draw.text((24, py_of(mph) - 9), f'{mph}', font=small, fill=COLORS['muted'])
        if accel:
            draw_acceleration(draw, accel, now, height + GRAPH_H, px_of, t0, t1, small)
        image.save(out / 'frames' / f'{index:05d}.png')
    fps = round(1. / (poses[1]['time'] - poses[0]['time'])) if len(poses) > 1 else 10
    video = out / (out.name + '.mp4')
    subprocess.run([str(args.ffmpeg), '-hide_banner', '-loglevel', 'error', '-n', '-framerate', str(fps),
                    '-i', str(out / 'frames/%05d.png'), '-c:v', 'libx264', '-preset', 'medium', '-crf', '20',
                    '-pix_fmt', 'yuv420p', '-movflags', '+faststart', str(video)], check=True, timeout=600)
    (out / 'render.json').write_text(json.dumps(dict(schema='highway_flow_3d_graph_render_v01', car=car,
        frames=len(images), fps=fps, first_time_s=poses[0]['time'], last_time_s=poses[-1]['time'],
        graph_first_time_s=t0, graph_last_time_s=t1, merge_entry_s=entry, lateral_merge_s=merged,
        stable_merge_passed_s=passed, render=str(args.render.resolve()), episode=str(args.episode.resolve()),
        physics_log=str(args.physics_log.resolve()) if args.physics_log else None,
        video=video.name), indent=2) + '\n', encoding='utf-8')
    print(video)


if __name__ == '__main__':
    main()
