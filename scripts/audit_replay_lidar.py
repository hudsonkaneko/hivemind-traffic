"""Read-only diagnostic of identity misses and range outliers; never changes gates."""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np
try:
    from .replay_lidar_geometry import ReplayReference
except ImportError:
    from replay_lidar_geometry import ReplayReference


def audit(run):
    manifest = json.loads((run/'manifest.json').read_text())
    for name, expected_hash in manifest['output_hashes'].items():
        if hashlib.sha256((run/name).read_bytes()).hexdigest() != expected_hash:
            raise ValueError(f'Artifact hash mismatch: {name}')
    config = json.loads((run/'resolved-config.json').read_text())
    reference = ReplayReference(json.loads((run/'recording.json').read_text()), config)
    rows = json.loads((run/'scan-metrics.json').read_text())[config['warmup_scans']:]
    confusion = np.zeros((len(reference.ids)+1, len(reference.ids)+1), dtype=np.int64)
    outlier_edges, examples = [], []
    for row in rows:
        with np.load(run/f"scan_{row['index']:04d}.npz") as saved:
            scan = {k:saved[k] for k in saved.files}
        expected, labels = reference.expected(scan)
        valid = np.isfinite(scan['range_m']) & (scan['range_m'] > 0) & ((scan['flags'] & 64) != 0)
        selected = np.isfinite(expected) & valid
        union = (np.isfinite(expected) | (scan['object_label'] >= 0)) & valid
        np.add.at(confusion, (labels[union]+1, scan['object_label'][union]+1), 1)
        indices = np.flatnonzero(selected & (np.abs(scan['range_m']-expected) > .2))
        if not indices.size:
            continue
        subset = {k: v[indices] if k in ['azimuth_deg','elevation_deg','offset_ns'] else v
                  for k,v in scan.items()}
        edges = reference.edge_distances(subset, expected[indices], labels[indices])
        outlier_edges.extend(edges.tolist())
        errors = np.abs(scan['range_m'][indices]-expected[indices])
        for j,index in enumerate(indices):
            examples.append(dict(scan=row['index'],ray=int(index),error_m=float(errors[j]),
                expected_m=float(expected[index]),observed_m=float(scan['range_m'][index]),
                edge_distance_m=float(edges[j]),expected_vehicle=reference.ids[labels[index]],
                observed_vehicle=reference.ids[scan['object_label'][index]] if scan['object_label'][index]>=0 else 'unknown_or_nonvehicle'))
    edges = np.asarray(outlier_edges)
    updates_path = run/'identity-updates.json'
    updates = json.loads(updates_path.read_text()) if updates_path.exists() else []
    return dict(run_id=manifest['run_id'],recording_sha256=manifest['recording_sha256'],
        config=config,command=manifest['command'],
        manifest_sha256=hashlib.sha256((run/'manifest.json').read_bytes()).hexdigest(),
        identity_updates=[dict(source=u['source'],callback_time_s=u['callback_time_s'],entry_count=len(u['entries'])) for u in updates],
        capture_summary=json.loads((run/'summary.json').read_text()),
        confusion_labels=['unknown_or_nonvehicle',*reference.ids],
        confusion_rows_expected_columns_observed=confusion.tolist(),
        outlier_count=len(edges),outlier_edge_max_m=float(edges.max()) if edges.size else None,
        outlier_edge_p95_m=float(np.percentile(edges,95)) if edges.size else None,
        outliers_within_2cm_of_edge=int((edges <= .02).sum()),
        largest_outliers=sorted(examples,key=lambda x:x['error_m'],reverse=True)[:10],
        caveat='Edge proximity is descriptive, not a cause diagnosis or exclusion rule. Original strict gates remain unchanged.')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run', type=Path, nargs='+')
    parser.add_argument('--output', type=Path, help='Write a new JSON report; refuses to overwrite an existing file')
    args = parser.parse_args()
    report = dict(audit_source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                  geometry_source_sha256=hashlib.sha256(Path(__file__).with_name('replay_lidar_geometry.py').read_bytes()).hexdigest(),
                  runs=[audit(run) for run in args.run])
    text = json.dumps(report,indent=2)
    if args.output:
        with args.output.open('x',encoding='utf-8') as stream:
            stream.write(text+'\n')
        print(f'Diagnostic report: {args.output}')
    else:
        print(text)
