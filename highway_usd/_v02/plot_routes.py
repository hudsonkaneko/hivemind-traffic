"""Illustrate four seeded re-entry choices on the same fixed road network."""
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from route_policy import generate_plans

folder=Path(__file__).resolve().parent
network=json.loads((folder/'navigation.json').read_text())
plans=generate_plans(network,seed=65,vehicles=256,exit_probability=1.)
selected={}
for plan in plans['plans']:
    if plan['start_side']=='East': selected.setdefault(plan['return_opportunity'],plan)
assert set(selected)=={1,2,3,4}
fig,axes=plt.subplots(2,2,figsize=(12,12),facecolor='#f4f6f3')
for opportunity,ax in zip(range(1,5),axes.flat):
    ax.set_facecolor('#f4f6f3')
    for key,lane in network['lanes'].items():
        if lane['kind']=='main': continue
        pts=np.array(lane['points']+([lane['points'][0]] if lane['closed'] else []))
        ax.plot(pts[:,0],pts[:,1],c='#c0c8c2',lw=1.3)
    plan=selected[opportunity]
    for edge in plan['edges']:
        e=network['edges'][edge]; pts=np.array(e['points'])
        color={'exit':'#d77924','return':'#19815b','collector':'#256da5','auxiliary':'#424d47'}[e['kind']]
        ax.plot(pts[:,0],pts[:,1],c=color,lw=3)
        mid=len(pts)//2
        if len(pts)>20:
            delta=pts[min(mid+10,len(pts)-1)]-pts[mid]; delta=delta/np.linalg.norm(delta)*45
            ax.arrow(pts[mid,0],pts[mid,1],delta[0],delta[1],head_width=22,color=color,length_includes_head=True)
    ax.set_aspect('equal'); ax.set_xlim(-590,590); ax.set_ylim(-590,590); ax.axis('off')
    ax.set_title(f'Rejoin at opportunity {opportunity}',fontsize=15,fontweight='bold',color='#284737')
    ax.text(0,20,f"{plan['route_length_m']/1000:.2f} km",ha='center',fontsize=20,color='#284737')
    ax.text(0,-55,'Planned trip, including approach and return',ha='center',fontsize=9,color='#627367')
fig.suptitle('Same roads, different exit trips',fontsize=23,fontweight='bold',color='#284737')
fig.text(.5,.025,'Orange: outward exit     Blue: collector travel     Green: return to highway',ha='center',fontsize=12)
fig.tight_layout(rect=[0,.05,1,.95]); fig.savefig(folder/'route_variants.png',dpi=160)
plt.close(fig)
(folder/'route_variants.json').write_text(json.dumps(dict(seed=65,plans=[selected[k] for k in range(1,5)]),indent=2)+'\n',newline='\n')
