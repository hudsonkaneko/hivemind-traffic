"""Build versioned SUMO highway layouts for curriculum/held-out transfer tests."""
from pathlib import Path
import json
import math
import subprocess
import hashlib
import xml.etree.ElementTree as ET
from traffic.sumo_backend import find_sumo_binary
from scripts.build_i110_sumo import write_xml

ROOT=Path(__file__).resolve().parents[1]


def build(output, *, lanes, length, bend, speed_mph):
    output=Path(output).resolve()
    if not output.is_relative_to(ROOT/'outputs') or output.exists():raise ValueError('Use a fresh project output directory')
    if lanes not in range(2,7):raise ValueError('Supported through-lane counts: 2..6')
    output.mkdir(parents=True)
    nodes,edges,connections=ET.Element('nodes'),ET.Element('edges'),ET.Element('connections')
    def position(s):return bend*math.sin(s/length*math.pi),-s
    for i in range(6):
        x,y=position(i*length/5);ET.SubElement(nodes,'node',id=f'n{i}',x=str(x),y=str(y),type='priority')
    # First three sections have one right auxiliary lane; it ends before sb4.
    for i in range(1,6):
        ET.SubElement(edges,'edge',{'id':f'sb{i}','from':f'n{i-1}','to':f'n{i}',
            'numLanes':str(lanes+(i<4)),'speed':str(speed_mph*.44704),'priority':'3'})
    for i in range(1,5):
        count=lanes+(i<4)
        for lane in range(count):
            if i==3 and lane==0:continue
            ET.SubElement(connections,'connection',{'from':f'sb{i}','to':f'sb{i+1}',
                'fromLane':str(lane),'toLane':str(lane-1 if i==3 else lane)})
    x,y=position(length/5)
    ET.SubElement(nodes,'node',id='ramp_start',x=str(x+60),y=str(y+160),type='priority')
    ET.SubElement(edges,'edge',{'id':'mlk_on','from':'ramp_start','to':'n1','numLanes':'1','speed':str(30*.44704),'priority':'2'})
    ET.SubElement(connections,'connection',{'from':'mlk_on','to':'sb2','fromLane':'0','toLane':'0'})
    for name,data in [('nodes.nod.xml',nodes),('edges.edg.xml',edges),('connections.con.xml',connections)]:write_xml(output/name,data)
    binary=find_sumo_binary().with_name('netconvert.exe' if __import__('os').name=='nt' else 'netconvert')
    command=[str(binary),'--node-files',str(output/'nodes.nod.xml'),'--edge-files',str(output/'edges.edg.xml'),
        '--connection-files',str(output/'connections.con.xml'),'--output-file',str(output/'network.net.xml'),
        '--no-turnarounds','true','--offset.disable-normalization','true']
    p=subprocess.run(command,capture_output=True,text=True,timeout=60)
    (output/'build.log').write_text(p.stdout+p.stderr);p.check_returncode()
    return dict(network=(output/'network.net.xml').relative_to(ROOT).as_posix(),
        network_sha256=hashlib.sha256((output/'network.net.xml').read_bytes()).hexdigest(),
        through_lanes=lanes,length_m=length,bend_m=bend,speed_limit_mph=speed_mph)


def main():
    root=ROOT/'outputs/highway-curriculum/v01'
    specs=[('train',n,1800+100*n,20*(n%2),60 if n%2 else 50) for n in (2,3,4,5)]
    specs += [('validation',n,1900+100*n,-30,55) for n in (2,4,5)]
    specs += [('test',n,2200+100*n,45,45 if n==6 else 60) for n in (3,5,6)]
    bank=[]
    for split,n,length,bend,speed in specs:
        ident=f'{split}-{n}-lane';entry=build(root/ident,lanes=n,length=length,bend=bend,speed_mph=speed)
        entry.update(id=ident,split=split);bank.append(entry)
    (root/'bank.json').write_text(json.dumps(bank,indent=2)+'\n')
    print(root/'bank.json')


if __name__=='__main__':main()
