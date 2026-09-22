"""Recompute a saved run, including deliberately broken timestamp/angle controls."""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np
from lidar_geometry import analyze_scan, summarize


def evaluate(run):
    config = json.loads((run / 'resolved-config.json').read_text())
    manifest = json.loads((run / 'manifest.json').read_text())
    for relative, expected_hash in manifest['output_hashes'].items():
        if hashlib.sha256((run / relative).read_bytes()).hexdigest() != expected_hash:
            raise ValueError(f'Artifact hash mismatch: {relative}')
    records = json.loads((run / 'scan-metrics.json').read_text())
    recorded_errors = json.loads((run / 'summary.json').read_text()).get('errors', [])
    moving = manifest['variant'] == 'moving'
    results = {'run_id': manifest['run_id'], 'variant': manifest['variant'],
               'config_sha256': hashlib.sha256((run / 'resolved-config.json').read_bytes()).hexdigest()}
    for variant in ['original', 'timestamp_plus_250ms', 'azimuth_sign_reversed']:
        recomputed = []
        for row in records:
            with np.load(run / f"scan_{row['index']:04d}.npz") as saved:
                scan = {key: saved[key].copy() for key in saved.files}
            if variant == 'timestamp_plus_250ms':
                scan['timestamp_ns'] = scan['timestamp_ns'] + 250_000_000
            if variant == 'azimuth_sign_reversed':
                scan['azimuth_deg'] *= -1
            metrics, _ = analyze_scan(scan, config, moving)
            recomputed.append(dict(row, **metrics, timestamp_ns=int(scan['timestamp_ns'])))
        results[variant] = summarize(recomputed, config, moving, recorded_errors)
    results['negative_controls_passed'] = all(not results[v]['passed'] for v in ['timestamp_plus_250ms', 'azimuth_sign_reversed'])
    return results


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run', type=Path)
    args = parser.parse_args()
    result = evaluate(args.run)
    print(json.dumps(result, indent=2))
    raise SystemExit(0 if result['original']['passed'] and result['negative_controls_passed'] else 1)
