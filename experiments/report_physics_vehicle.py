"""Verify a physics run's hashes and publish a compact, portable evidence summary."""
import argparse
import hashlib
import json
from pathlib import Path

from experiments.probe_support import ROOT, write_json


def summarize(run):
    run = Path(run).resolve()
    if not run.is_relative_to(ROOT/'outputs'/'physics_vehicle'):
        raise ValueError('Select a physics_vehicle run inside the canonical workspace')
    manifest = json.loads((run/'manifest.json').read_text())
    for rel,digest in manifest['output_hashes'].items():
        file = (run/rel).resolve()
        if not file.is_relative_to(run) or hashlib.sha256(file.read_bytes()).hexdigest() != digest:
            raise ValueError(f'Run artifact hash mismatch: {rel}')
    for rel,digest in manifest['source_hashes'].items():
        file = (run/'source'/rel).resolve()
        if not file.is_relative_to(run/'source') or hashlib.sha256(file.read_bytes()).hexdigest() != digest:
            raise ValueError(f'Captured source hash mismatch: {rel}')
    summary = json.loads((run/'summary.json').read_text())
    return dict(run_id=run.name, local_evidence=str(run.relative_to(ROOT)).replace('\\','/'),
                manifest_sha256=hashlib.sha256((run/'manifest.json').read_bytes()).hexdigest(),
                artifact_hashes_verified=True, git_commit=manifest['git_commit'],
                working_tree_dirty=bool(manifest['git_status']), source_hashes=manifest['source_hashes'],
                configuration=manifest['config'], isaac_version=manifest.get('isaac_version'),
                hardware=manifest.get('gpu_before'), model=json.loads((run/'vehicle-model.json').read_text()),
                result=summary)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run', type=Path)
    parser.add_argument('--output', type=Path, help='Optional new JSON file; existing files are never overwritten')
    args = parser.parse_args()
    result = summarize(args.run)
    if args.output:
        target = args.output.resolve()
        if not target.is_relative_to(ROOT) or target.exists():
            parser.error('Output must be a new file in this checkout')
        write_json(target,result)
    print(json.dumps(result,indent=2))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
