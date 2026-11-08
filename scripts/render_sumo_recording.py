"""Render recorded SUMO positions as an explicitly schematic animated GIF."""
import argparse
import json
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('recording', type=Path)
    args = parser.parse_args()
    directory = args.recording.resolve()
    if not directory.is_relative_to(ROOT / 'outputs'):
        parser.error('Recording must be under project outputs/')
    rows = json.loads((directory / 'trajectories.json').read_text())
    manifest = json.loads((directory / 'recording.json').read_text())
    colors = [(40,220,100), (70,160,255), (250,190,40), (240,80,180)]
    regular = ImageFont.load_default(size=17)
    small = ImageFont.load_default(size=14)
    title = ImageFont.load_default(size=26)
    images = []
    for row in rows:
        im = Image.new('RGB', (960,480), '#101722')
        draw = ImageDraw.Draw(im)
        draw.text((24,18), 'SUMO / MAPPO replay', font=title, fill='white')
        draw.text((24,52), 'Recorded simulation positions | Fixed lanes | 60 mph ceiling', font=regular, fill='#adbed1')
        draw.text((780,24), f"{row['time']:04.1f} / {len(rows)/10:.0f} s",font=regular,fill='white')
        active = {a:s for a,s in row['vehicles'].items() if a.startswith('agent_')}
        for i in range(4):
            agent = f'agent_{i}'
            x = 24 + i * 235
            draw.rounded_rectangle((x,88,x+215,139), radius=7, fill='#202d3c')
            speed = active[agent]['speed'] / .44704
            draw.text((x+10,96),f'Car {i+1}    {speed:4.1f} mph',font=regular,fill=colors[i])
            override = row['overrides'].get(agent,False)
            draw.text((x+10,118),'Safety override' if override else 'Policy command',font=small,
                      fill='#ffad60' if override else '#c5d2e1')
        xs = [s['x'] for s in active.values()]
        left, right = min(xs)-60, max(xs)+75
        scale = 960 / (right-left)
        draw.rectangle((0,170,960,380),fill='#354253')
        for y in (170,380): draw.line((0,y,960,y),fill='#e4e9ef',width=3)
        # Road markings move in the following camera's frame; world positions stay authoritative.
        for y in (240,310):
            for world_x in range(int(left//12)*12,int(right)+12,12):
                draw.line(((world_x-left)*scale,y,(world_x+5-left)*scale,y),fill='#d7dfe9',width=2)
        for agent,state in row['vehicles'].items():
            x = (state['x']-left)*scale
            if not -30 <= x <= 990: continue
            y = 345 - state['lane_index']*70
            controlled = agent.startswith('agent_')
            color = colors[int(agent[-1])] if controlled else (150,162,178)
            length = max(15,state['length']*scale)
            box=(int(x-length),y-12,int(x),y+12)
            draw.rounded_rectangle(box,radius=4,fill=color)
            draw.rectangle((int(x-7),y-8,int(x-3),y+8),fill='#1c2838')
            if controlled:
                draw.text((int(x-length),y-32),str(int(agent[-1])+1),font=small,fill=color)
                if row['overrides'].get(agent):
                    draw.rounded_rectangle((box[0]-3,box[1]-3,box[2]+3,box[3]+3),radius=5,outline='#ffad60',width=2)
        draw.text((24,400),'Colored cars: frozen trained actor   |   Gray: SUMO traffic   |   Orange outline: safety override',font=small,fill='#d7dfe9')
        draw.text((24,427),'2D trajectory visualization, not GUI screen capture. Lane spacing expanded for readability.',font=small,fill='#adbed1')
        draw.text((24,450),'Safety controls remain enabled. This policy has not passed the intervention acceptance gate.',font=small,fill='#ffad60')
        if len(images) in (0,300,599): im.save(directory / f'preview-{len(images):03d}.png')
        images.append(im.quantize(colors=128))
    target = directory / 'sumo-mappo-replay.gif'
    images[0].save(target,save_all=True,append_images=images[1:],duration=100,loop=0,optimize=False)
    with Image.open(target) as decoded:
        if decoded.n_frames != len(rows): raise RuntimeError('Encoded frame count mismatch')
    manifest['animation'] = dict(file=target.name,frames=len(rows),fps=10,format='GIF',lane_spacing='schematic')
    (directory/'recording.json').write_text(json.dumps(manifest,indent=2)+'\n')
    print(target)


if __name__ == '__main__':
    main()
