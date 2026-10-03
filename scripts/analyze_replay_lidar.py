"""Recompute curved replay checks and corruption controls from immutable scans."""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np
try:
    from .replay_lidar_geometry import ReplayReference, summarize
except ImportError:
    from replay_lidar_geometry import ReplayReference, summarize


def evaluate(run, controls=True):
    manifest=json.loads((run/'manifest.json').read_text())
    for name,expected in manifest['output_hashes'].items():
        if hashlib.sha256((run/name).read_bytes()).hexdigest()!=expected:
            raise ValueError(f'Artifact hash mismatch: {name}')
    config=json.loads((run/'resolved-config.json').read_text())
    data=json.loads((run/'recording.json').read_text())
    reference=ReplayReference(data,config)
    records=json.loads((run/'scan-metrics.json').read_text())
    capture_summary=json.loads((run/'summary.json').read_text())
    variants=['original','timestamp_plus_250ms','azimuth_sign_reversed'] if controls else ['original']
    results={name:[] for name in variants}
    for row in records:
        with np.load(run/f"scan_{row['index']:04d}.npz") as saved:
            scan={key:saved[key].copy() for key in saved.files}
        for name in variants:
            metrics,_=reference.analyze(scan,time_shift=.25 if name=='timestamp_plus_250ms' else 0,
                                        reverse_angles=name=='azimuth_sign_reversed')
            results[name].append(dict(row,**metrics))
    reports={name:summarize(rows,config,capture_summary['callback_errors']) for name,rows in results.items()}
    for report in reports.values():
        report['shared_layers_unchanged']=capture_summary['shared_layers_unchanged']
        report['passed']=report['passed'] and report['shared_layers_unchanged']
    return dict(run_id=manifest['run_id'],recording_sha256=manifest['recording_sha256'],
                config=config,results=reports,
                caveat='Controls perturb the geometric reference time/angle only; captured timing metadata is unchanged. A rejected control does not prove the original passed.')


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run',type=Path)
    parser.add_argument('--no-controls',action='store_true')
    args=parser.parse_args()
    report=evaluate(args.run,not args.no_controls)
    print(json.dumps(report,indent=2))
    raise SystemExit(0 if report['results']['original']['passed'] else 1)
