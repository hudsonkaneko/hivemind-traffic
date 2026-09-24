"""Verify and recompute two-car lifecycle captures without Isaac Sim."""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np
try:
    from .lidar_lifecycle_analysis import analyze,summarize
except ImportError:
    from lidar_lifecycle_analysis import analyze,summarize


def evaluate(run):
    manifest=json.loads((run/'manifest.json').read_text())
    for name,expected in manifest['output_hashes'].items():
        if hashlib.sha256((run/name).read_bytes()).hexdigest()!=expected:
            raise ValueError(f'Artifact hash mismatch: {name}')
    config=json.loads((run/'resolved-config.json').read_text())
    saved=json.loads((run/'scan-metrics.json').read_text())
    summary=json.loads((run/'summary.json').read_text())
    rows=[]
    echoes,unknown_ids={},{}
    mapped_paths=set()
    sensor_mapping={}
    sensor_matches=selected_hits=0
    for row in saved:
        with np.load(run/f"scan_{row['index']:04d}.npz") as data:
            scan={k:data[k] for k in data.files}
        for name in ['StableIdMap','StableIdMapDeltas']:
            sensor_mapping.update({int(k):v for k,v in row['maps'].get(name,{}).items()})
        mapping=({int(k):v for k,v in row['maps'].get('CameraStableIdMap',{}).items()}
                 if config.get('map_source','sensor')=='camera' else sensor_mapping)
        verify_labels(scan,mapping)
        if 'sensor_target_label' in scan:
            verify_labels(dict(scan,target_label=scan['sensor_target_label']),sensor_mapping)
        result=analyze(scan,config,manifest['variant'])
        rows.append(result)
        for mapping in row['maps'].values(): mapped_paths.update(mapping.values())
        if result['phase']=='transition_excluded': continue
        for key,count in result['echo_counts'].items(): echoes[key]=echoes.get(key,0)+count
        roi=(np.abs(scan['azimuth_deg'])<config['roi_half_angle_deg']) & (np.abs(scan['elevation_deg'])<config['roi_half_angle_deg'])
        valid=np.isfinite(scan['range_m']) & (scan['range_m']>0) & ((scan['flags']&64)!=0)
        selected_hits+=int((roi & valid).sum())
        sensor_matches+=int(scan.get('sensor_target_label',scan['target_label'])[roi & valid].sum())
        raw=np.ascontiguousarray(scan['object_id_raw']).view(np.uint8).reshape(-1,16)
        for key in raw[roi & valid & ~scan['target_label']]:
            number=str(int.from_bytes(key.tobytes(),'little'))
            unknown_ids[number]=unknown_ids.get(number,0)+1
    recomputed=summarize(rows,config,manifest['variant'],summary['callback_errors'])
    if recomputed != summary:
        raise ValueError('Offline summary differs from capture summary')
    return dict(run_id=manifest['run_id'],summary=recomputed,roi_echo_counts=echoes,
        map_source=config.get('map_source','sensor'),
        render_preflight=config.get('render_preflight',False),
        sensor_map_identity_agreement=sensor_matches/max(1,selected_hits),
        target_path_ever_in_map='/World/Target/Body' in mapped_paths,
        unmapped_or_wrong_target_roi_ids=unknown_ids,
        manifest_sha256=hashlib.sha256((run/'manifest.json').read_bytes()).hexdigest(),
        artifact_hashes_verified=True)


def verify_labels(scan,mapping):
    """Recompute exact renderer-ID labels; do not trust saved boolean labels."""
    raw=np.ascontiguousarray(scan['object_id_raw']).view(np.uint8).reshape(-1,16)
    labels=np.array([mapping.get(int.from_bytes(k.tobytes(),'little'))=='/World/Target/Body' for k in raw])
    if not np.array_equal(labels,scan['target_label']):
        raise ValueError('Saved labels differ from exact renderer ID mapping')


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('runs',type=Path,nargs='+')
    parser.add_argument('--output',type=Path)
    args=parser.parse_args()
    result=dict(analysis_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),runs=[evaluate(p) for p in args.runs])
    text=json.dumps(result,indent=2)
    if args.output:
        with args.output.open('x',encoding='utf-8') as stream: stream.write(text+'\n')
    print(text)
