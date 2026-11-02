"""Static semantic-support evidence: no model load, simulator or ABI conversion."""
import argparse
import ast
from collections import Counter
from datetime import datetime, timezone
from pathlib import Path
import shutil
import sys
import xml.etree.ElementTree as ET

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from scripts.report_i110_anticipation import read, write, sha


def network_summary(path):
    return {edge.get('id'):dict(lane_count=len(edge.findall('lane')),
        lane_lengths_m=[float(lane.get('length')) for lane in edge.findall('lane')])
        for edge in ET.parse(path).getroot().findall('edge') if edge.get('function')!='internal'}


def audit(output, analysis):
    output=Path(output).resolve();output.mkdir(parents=True,exist_ok=False)
    analysis=Path(analysis).resolve(); m=read(analysis/'manifest.json')
    if m['status']!='completed' or sha(analysis/'summary.json')!=m['outputs']['summary.json']:
        raise ValueError('Invalid shadow analysis')
    original=ROOT/'outputs/hudson-delivery-20261029/held-out-v1'
    original_manifest=read(original/'manifest.json')
    names=['environments/sumo_mappo_env.py','traffic/mappo_sumo.py',
        'experiments/configs/sumo_mappo_v2.json','scenarios/mappo_highway/network.net.xml']
    inputs={str(analysis/'manifest.json'):sha(analysis/'manifest.json'),
            str(analysis/'summary.json'):sha(analysis/'summary.json'),
            str(original/'manifest.json'):sha(original/'manifest.json')}
    for name in names:
        path=original/'source'/name
        if sha(path)!=original_manifest['outputs']['source/'+name]:
            raise ValueError('Original identity mismatch: '+name)
        inputs[str(path)]=sha(path)
    current_names=['traffic/i110_policy_interface.py','traffic/i110_baseline.py','traffic/smooth_acc.py',
        'outputs/i110-sumo-20261027/network-c/network.net.xml']
    for name in current_names:
        inputs[str(ROOT/name)]=sha(ROOT/name)
    counts=Counter()
    for run in read(analysis/'summary.json')['runs']:
        counts.update(run['semantic_packet_counts'])
    anchors={}
    for path in [original/'source/environments/sumo_mappo_env.py',ROOT/'traffic/i110_policy_interface.py']:
        anchors[str(path)]={node.name:dict(start=node.lineno,end=node.end_lineno)
            for node in ast.walk(ast.parse(path.read_text(encoding='utf-8')))
            if isinstance(node,(ast.FunctionDef,ast.AsyncFunctionDef)) and node.name in
            {'_neighbor','_observations','_safety_filter','reset','step','action_to_acceleration',
             'observe','neighbor','adjacent','require_actor_schema'}}
    result=dict(status='reviewed_no_transfer_admitted', actor_loaded=False, training=False,
        original_network=network_summary(original/'source/scenarios/mappo_highway/network.net.xml'),
        i110_network=network_summary(ROOT/current_names[-1]), observed_packets=dict(counts), source_anchors=anchors,
        verdict='No semantically exact supported I-110 restriction has been established for the unchanged original ABI/config. Four/five-lane packets cannot be consumed directly. A separately defined local-window adapter remains a proposal requiring explicit lane/route/time/history/guard semantics and support rejection tests.',
        scope='Static source/network and six development trace audit; not a proof that every possible restricted adapter is impossible. No policy execution or checkpoint bytes needed.',
        independent_gate='Original acceptance passes; strict numerical parity remains failed at frozen 1e-9 tolerance. This review neither retests nor resolves it.',
        input_sha256=inputs)
    write(output/'review.json',result)
    for name in ['scripts/audit_i110_transfer_support.py','scripts/report_i110_anticipation.py','traffic/i110_policy_interface.py','traffic/i110_baseline.py','traffic/smooth_acc.py']:
        dest=output/'source'/name;dest.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(ROOT/name,dest)
    write(output/'manifest.json',dict(status='completed',created_utc=datetime.now(timezone.utc).isoformat(),
        command=[sys.executable,*sys.argv],inputs=inputs,
        outputs={p.relative_to(output).as_posix():sha(p) for p in output.rglob('*') if p.is_file()}))
    print(result['verdict']);print(dict(counts))
    return result


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--analysis',type=Path,required=True)
    audit(**vars(parser.parse_args()))
