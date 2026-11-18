"""Render a recorded highway-flow episode as a top-down MP4 that follows one car (no simulation).

Inputs come only from one recorded episode directory: recording.json (native SUMO
poses of every vehicle, every 0.1 s), the episode's network.net.xml (lane geometry)
and actions.jsonl.gz (the followed car's merge target, request and overrides). Road
surfaces are drawn from SUMO lane shapes; cars are boxes at their native pose. It is
a schematic view, not a photoreal render.
"""
import argparse
import gzip
import json
import math
import subprocess
from pathlib import Path

import sumolib
from PIL import Image, ImageDraw, ImageFont

W, H = 1280, 720
ROAD_TOP, ROAD_BOTTOM = 96, 520
SCALE = 7.0  # pixels per metre
MPH = 2.23694
FONT_DIR = Path('C:/Windows/Fonts')
COLORS = dict(bg='#0f1620', grass='#1b2a1f', edge='#8b95a3', asphalt='#3b424c', ego='#3ddc84',
              controlled='#4aa3ff', traffic='#c9ced6', text='#e8edf3', muted='#9fb0c3', target='#ffb347',
              band='#4d3b22', window='#203247', marker='#ff6b6b')


def font(size, bold=False):
    name = 'segoeuib.ttf' if bold else 'segoeui.ttf'
    try:
        return ImageFont.truetype(str(FONT_DIR / name), size)
    except OSError:
        return ImageFont.load_default(size=size)


def load(episode):
    recording = json.loads((episode / 'recording.json').read_text())
    journals = sorted(episode.glob('episode-*/actions.jsonl.gz'))
    networks = sorted(episode.glob('episode-*/network.net.xml'))
    if len(journals) != 1 or len(networks) != 1:
        raise ValueError('Expected exactly one recorded episode with its journal and network')
    rows = [json.loads(line) for line in gzip.open(journals[0], 'rt')]
    return recording, rows, sumolib.net.readNet(str(networks[0]), withInternal=True)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('episode', type=Path, help='directory holding recording.json and episode-*/')
    parser.add_argument('--car', default='agent_0')
    parser.add_argument('--title', required=True)
    parser.add_argument('--summary', type=Path, required=True,
                        help="recording summary.json with the car's stable_merge entry and lateral-merge times")
    parser.add_argument('--ffmpeg', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args(argv)
    recording, rows, net = load(args.episode)
    car = args.car
    frames = [f for f in recording['frames'] if any(v['id'] == car for v in f['vehicles'])]
    if not frames:
        raise ValueError('The followed car never appears in the recording')
    decisions = {}
    for row in rows:
        a = (row.get('agents') or {}).get(car)
        if a:
            target = (a['v04_reward_audit'].get('pre_action') or {}).get('target') or {}
            decisions[round(row['time'], 1)] = dict(
                target=target.get('target_m_s') if target.get('eligible') else None,
                request=a['requested_acceleration'], overrides=a['override_causes'],
                merge_active=(a.get('v05_audit') or {}).get('merge_active'))

    ego = [next(v for v in f['vehicles'] if v['id'] == car) for f in frames]
    times = [f['time'] for f in frames]
    dx, dy = ego[-1]['x'] - ego[0]['x'], ego[-1]['y'] - ego[0]['y']
    norm = math.hypot(dx, dy)
    d = (dx / norm, dy / norm)          # overall travel direction -> screen right
    n = (d[1], -d[0])                   # driver's right -> screen down
    lanes = [(lane.getShape(), lane.getWidth()) for edge in net.getEdges() for lane in edge.getLanes()]
    merge = json.loads(args.summary.read_text())['stable_merge'][car]  # the gate's own event times
    entry, merged = merge['entry_time_s'], merge['lateral_merge_complete_time_s']
    passed = next((t for t, v in zip(times, ego) if v['stable_merge_status'] == 'succeeded'), None)
    speeds = [v['speed'] for v in ego]
    targets = [decisions.get(round(t, 1), {}).get('target') for t in times]
    top = max(max(speeds), max(x for x in targets if x is not None)) * MPH + 4

    big, mid, small = font(26, True), font(19), font(15)
    out = args.output.resolve()
    (out / 'frames').mkdir(parents=True, exist_ok=False)
    cu = cv = None
    t0, t1 = times[0], times[-1]
    px_of = lambda t: 70 + (t - t0) / (t1 - t0) * (W - 110)  # noqa: E731
    py_of = lambda mph: 700 - mph / top * 150  # noqa: E731
    for index, (frame, me) in enumerate(zip(frames, ego)):
        heading = math.radians(me['heading'])
        cx = me['x'] - math.sin(heading) * me['length'] / 2
        cy = me['y'] - math.cos(heading) * me['length'] / 2
        u, v = cx * d[0] + cy * d[1], cx * n[0] + cy * n[1]
        cu = u if cu is None else cu + .35 * (u - cu)
        cv = v if cv is None else cv + .08 * (v - cv)
        def screen(x, y):
            return (W * .4 + (x * d[0] + y * d[1] - cu) * SCALE,
                    (ROAD_TOP + ROAD_BOTTOM) / 2 + (x * n[0] + y * n[1] - cv) * SCALE)
        image = Image.new('RGB', (W, H), COLORS['bg'])
        draw = ImageDraw.Draw(image)
        draw.rectangle((0, ROAD_TOP, W, ROAD_BOTTOM), fill=COLORS['grass'])
        visible = []
        for shape, width in lanes:
            points = [screen(x, y) for x, y in shape]
            if any(-200 < px < W + 200 and ROAD_TOP - 200 < py < ROAD_BOTTOM + 200 for px, py in points):
                visible.append((points, width))
        for points, width in visible:
            draw.line(points, fill=COLORS['edge'], width=max(1, round(width * SCALE)), joint='curve')
        for points, width in visible:
            draw.line(points, fill=COLORS['asphalt'], width=max(1, round(width * SCALE) - 3), joint='curve')
        for vehicle in sorted(frame['vehicles'], key=lambda x: x['id'] == car):
            h = math.radians(vehicle['heading'])
            fx, fy = math.sin(h), math.cos(h)            # SUMO angle: clockwise from north
            rx, ry = fy, -fx
            bx, by = vehicle['x'], vehicle['y']          # front bumper
            hl, hw = vehicle['length'], vehicle['width'] / 2
            corners = [(bx + rx * hw, by + ry * hw), (bx - rx * hw, by - ry * hw),
                       (bx - fx * hl - rx * hw, by - fy * hl - ry * hw), (bx - fx * hl + rx * hw, by - fy * hl + ry * hw)]
            points = [screen(x, y) for x, y in corners]
            if not any(0 <= px <= W and ROAD_TOP <= py <= ROAD_BOTTOM for px, py in points):
                continue
            color = COLORS['ego'] if vehicle['id'] == car else COLORS['controlled'] if vehicle['controlled'] else COLORS['traffic']
            draw.polygon(points, fill=color, outline='white' if vehicle['id'] == car else '#20262e')
            if vehicle['id'] == car:
                lx = min(p[0] for p in points)
                ly = min(p[1] for p in points)
                draw.text((lx - 6, ly - 24), car, font=small, fill=COLORS['ego'])
        legend = [('ego', f'followed car ({car})'), ('controlled', 'other cars driven by the same policy'),
                  ('traffic', 'SUMO background traffic')]
        lx = 24
        for key, label in legend:
            draw.rectangle((lx, ROAD_TOP + 14, lx + 22, ROAD_TOP + 24), fill=COLORS[key])
            draw.text((lx + 30, ROAD_TOP + 9), label, font=small, fill=COLORS['muted'])
            lx += 50 + draw.textlength(label, font=small)
        # heads-up text
        now = round(frame['time'], 1)
        info = decisions.get(now, {})
        phase = ('stable merge passed' if me['stable_merge_status'] == 'succeeded' else
                 'stable-merge window (8 s)' if now >= merged else
                 'merging into mainline' if now >= entry else 'on ramp / acceleration lane')
        draw.rectangle((0, 0, W, ROAD_TOP), fill=COLORS['bg'])
        draw.text((24, 10), args.title, font=big, fill=COLORS['text'])
        target = info.get('target')
        parts = [f't = {now:4.1f} s', f'speed {me["speed"] * MPH:4.1f} mph',
                 f'merge target {target * MPH:4.1f} mph' if target is not None else 'merge target -',
                 f'request {info.get("request", 0.):+.2f} m/s²', phase]
        draw.text((24, 52), '   |   '.join(parts), font=mid, fill=COLORS['muted'])
        if info.get('overrides'):
            draw.text((W - 330, 52), 'override: ' + ', '.join(info['overrides']), font=mid, fill=COLORS['marker'])
        # speed vs target timeline
        draw.rectangle((0, ROAD_BOTTOM, W, H), fill=COLORS['bg'])
        draw.text((24, ROAD_BOTTOM + 8), 'Speed (green) vs merge target (orange, ±2 m/s band) — shaded: 8 s stable-merge window',
                  font=small, fill=COLORS['muted'])
        if merged is not None:
            draw.rectangle((px_of(merged), 545, px_of(min(merged + 8., t1)), 700), fill=COLORS['window'])
        for i in range(1, len(times)):
            a, b = targets[i - 1], targets[i]
            if a is not None and b is not None:
                x0, x1 = px_of(times[i - 1]), px_of(times[i])
                draw.polygon([(x0, py_of((a + 2) * MPH)), (x1, py_of((b + 2) * MPH)),
                              (x1, py_of((b - 2) * MPH)), (x0, py_of((a - 2) * MPH))], fill=COLORS['band'])
        for i in range(1, len(times)):
            if targets[i - 1] is not None and targets[i] is not None:
                draw.line([(px_of(times[i - 1]), py_of(targets[i - 1] * MPH)), (px_of(times[i]), py_of(targets[i] * MPH))],
                          fill=COLORS['target'], width=2)
        upto = [(px_of(t), py_of(s * MPH)) for t, s in zip(times[:index + 1], speeds[:index + 1])]
        if len(upto) > 1:
            draw.line(upto, fill=COLORS['ego'], width=3)
        for mark, label, y, left in ((entry, 'merge entry', 680, True), (merged, 'merged', 680, False),
                                     (passed, 'stable merge passed', 680, False)):
            if mark is not None:
                draw.line([(px_of(mark), 545), (px_of(mark), 700)], fill=COLORS['muted'], width=1)
                x = px_of(mark) - 4 - draw.textlength(label, font=small) if left else px_of(mark) + 4
                draw.text((x, y), label, font=small, fill=COLORS['text'])
        draw.line([(px_of(now), 540), (px_of(now), 702)], fill='white', width=1)
        for mph in range(0, int(top) + 1, 10):
            draw.text((24, py_of(mph) - 9), f'{mph}', font=small, fill=COLORS['muted'])
        image.save(out / 'frames' / f'{index:05d}.png')
    video = out / (out.name + '.mp4')
    subprocess.run([str(args.ffmpeg), '-hide_banner', '-loglevel', 'error', '-n', '-framerate', '10',
                    '-i', str(out / 'frames/%05d.png'), '-c:v', 'libx264', '-preset', 'medium', '-crf', '20',
                    '-pix_fmt', 'yuv420p', '-movflags', '+faststart', str(video)], check=True, timeout=300)
    (out / 'render.json').write_text(json.dumps(dict(schema='highway_flow_topdown_render_v01', car=car,
        frames=len(frames), fps=10, first_time_s=t0, last_time_s=t1, merge_entry_s=entry, lateral_merge_s=merged,
        stable_merge_passed_s=passed,
        source=str(args.episode.resolve()), video=video.name, schematic=True), indent=2) + '\n', encoding='utf-8')
    print(video)


if __name__ == '__main__':
    main()
