"""Render comparisons from saved arrays, without modifying predictions."""
from pathlib import Path
import numpy as np
from PIL import Image, ImageDraw
from traffic.segmentation_preview import PALETTE, _font


def render_comparison(output, source, case, stem, rows):
    output,source=Path(output),Path(source)
    with Image.open(source/case/(stem+'-rgb.png')) as image:rgb=np.array(image.convert('RGB'))
    truth=np.load(source/case/(stem+'-semantic.npy'),allow_pickle=False)
    masks={v:np.load(output/v/case/(stem+'-project.npy'),allow_pickle=False)
           for v in ('square512','letterbox512','letterbox1024')}
    with Image.open(output/'letterbox1024'/case/(stem+'-occupancy.png')) as im:occupancy=np.array(im)
    panels=[('Saved RGB',rgb),('Baseline: stretched 512',PALETTE[masks['square512']]),
            ('Aspect preserved: 512',PALETTE[masks['letterbox512']]),
            ('Simulator ground truth',PALETTE[truth]),('Aspect preserved: 1024',PALETTE[masks['letterbox1024']]),
            ('Sparse LiDAR + RGB labels (1024)',occupancy)]
    h,w=truth.shape;header=100;row_height=h+80;footer=125
    canvas=Image.new('RGB',(w*3,header+row_height*2+footer),(19,24,32));draw=ImageDraw.Draw(canvas)
    draw.text((16,14),'Saved-capture refinement / '+case+' / '+stem,fill='white',font=_font(26))
    draw.text((16,53),'Same frozen model and RGB; no training or new simulation. All variants shown; exploratory results.',fill=(200,210,225),font=_font(19))
    variants=[None,'square512','letterbox512',None,'letterbox1024',None]
    for i,(title,array) in enumerate(panels):
        x=(i%3)*w;y=header+(i//3)*row_height
        draw.text((x+12,y),title,fill='white',font=_font(22))
        variant=variants[i]
        if variant:
            r=next(r for r in rows if r['case']==case and r['frame']==stem and r['variant']==variant)
            b=r['boundary_metrics']['per_class']
            text='Boundary F1: road %.1f%% | vehicle %.1f%%'%(100*b['road']['f1'],100*b['vehicle']['f1'])
        elif i==5:text='Orange: unclassified occupied | blue: RGB vehicle'
        elif i==3:text='Ground truth is used for evaluation only'
        else:text='640 x 360 original saved camera image'
        draw.text((x+12,y+32),text,fill=(190,200,215),font=_font(17))
        canvas.paste(Image.fromarray(array),(x,y+65))
    y=header+row_height*2
    for n,line in enumerate([
        'Mask colors: green road / blue vehicle / yellow lane / orange obstacle / dark background.',
        'RGB model cannot emit lane or generic obstacle labels. Orange LiDAR points mean occupied geometry, not named objects.',
        'LiDAR is exact-pixel sparse evidence: untouched pixels are unknown, not proof of clear road. Raw RGB masks are not clipped.',
        'Boundary scores use a 2-pixel tolerance. These 24 correlated frames are not a held-out driving benchmark.'
    ]):draw.text((16,y+10+n*27),line,fill=(205,215,225),font=_font(18))
    target=output/(case+'-comparison.png')
    if target.exists():raise FileExistsError(target)
    canvas.save(target)
    return target
