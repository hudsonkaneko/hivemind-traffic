"""Annotate actual renderer frames with native timestamps and measured speeds."""
from pathlib import Path
import argparse
import hashlib
import json
import subprocess
import time
from PIL import Image, ImageDraw, ImageFont

ROOT = Path(__file__).resolve().parents[1]
STUDY = ROOT/'outputs/highway-flow/v01'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--renders', type=Path, required=True)
    parser.add_argument('--comparison', type=Path, required=True)
    args = parser.parse_args()
    renders, folder = args.renders.resolve(), args.comparison.resolve()
    if not renders.is_relative_to(STUDY) or not folder.is_relative_to(STUDY):
        raise ValueError('Use project study paths')
    target = folder/'annotated-frames'
    target.mkdir(exist_ok=False)
    started = time.monotonic()
    font = ImageFont.truetype('C:/Windows/Fonts/segoeui.ttf', 21)
    headings = {'left':'Frozen MAPPO v05', 'right':'Flow reference v01'}
    panels = [(a,s) for a in ('agent_0','agent_4') for s in ('left','right')]
    reports = {pair:json.loads((renders/(pair[0]+'-'+pair[1])/'render-report.json').read_text()) for pair in panels}
    if any(r.get('passed') is not True or r['frames']!=300 for r in reports.values()):
        raise ValueError('Four verified300-frame renders required')
    records = {s:json.loads((folder/(s+'-5fps')/'recording.json').read_text()) for s in headings}
    frames = {s:[f for f in d['frames'] if .1-1e-7<=f['time']<=59.9+1e-7] for s,d in records.items()}
    if any(len(f)!=300 for f in frames.values()):
        raise ValueError('Exact shared60-second window required')
    for index in range(300):
        canvas = Image.new('RGB',(1600,988),'#111b26')
        draw = ImageDraw.Draw(canvas)
        for panel,(agent,side) in enumerate(panels):
            x,y=(panel%2)*800,(panel//2)*494
            source = renders/(agent+'-'+side)/'frames'/f'{index:05d}.png'
            with Image.open(source) as im:
                canvas.paste(im.convert('RGB').resize((800,450),Image.Resampling.LANCZOS),(x,y+44))
            sample=frames[side][index]
            car=next((v for v in sample['vehicles'] if v['id']==agent),None)
            motion=f"{car['speed']:.2f} m/s ({car['speed']/.44704:.1f} mph)" if car else 'No recorded pose'
            draw.text((x+12,y+9),f"{headings[side]} | {agent} | t={sample['time']:.1f}s | {motion}",font=font,fill='white')
        canvas.save(target/f'{index:05d}.png')
        if index in (50,100,150,200):
            canvas.save(folder/f'comparison-{index:03d}.png')
    ffmpeg=ROOT/'outputs/i110-merge-visual/v01/encoder/imageio_ffmpeg/binaries/ffmpeg-win-x86_64-v7.1.exe'
    video=folder/'i110-flow-comparison.mp4'
    encoded=subprocess.run([str(ffmpeg),'-hide_banner','-n','-framerate','5','-i',str(target/'%05d.png'),
        '-c:v','libx264','-preset','fast','-crf','18','-pix_fmt','yuv420p','-movflags','+faststart',str(video)],
        check=True,capture_output=True,text=True)
    (folder/'composition-encoding.log').write_text(encoded.stderr,encoding='utf-8')
    decoded=subprocess.run([str(ffmpeg),'-v','error','-i',str(video),'-progress','pipe:1','-f','null','-'],
        check=True,capture_output=True,text=True)
    counts=[int(x.split('=')[1]) for x in decoded.stdout.splitlines() if x.startswith('frame=')]
    if not counts or counts[-1]!=300:
        raise ValueError('Decoded comparison frame count differs')
    report=dict(training=False,simulation_run=False,frames=300,fps=5,start_time=.1,end_time=59.9,
        decoded_frames=counts[-1],pose_interpolation=False,annotation_source='Original measured native frame',
        video_sha256=hashlib.sha256(video.read_bytes()).hexdigest(),wall_seconds=time.monotonic()-started,
        limitation='Kinematic SimReady replay, illustrative roads; no physics or sensor inference validation')
    (folder/'composition.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report))


if __name__=='__main__':
    main()
