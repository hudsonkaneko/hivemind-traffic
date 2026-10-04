"""Hash-verify retained attempts and export compact stage evidence, including failures."""
import argparse
import hashlib
import json
from pathlib import Path

from experiments.probe_support import ROOT, write_json


def summarize(run):
    run = Path(run).resolve()
    if not run.is_relative_to(ROOT/'outputs') or not run.parent.name.startswith('vehicle_'):
        raise ValueError('Select a vehicle-stage run within this checkout outputs directory')
    manifest = json.loads((run/'manifest.json').read_text())
    for name, expected in manifest['output_hashes'].items():
        target = (run/name).resolve()
        if not target.is_relative_to(run) or hashlib.sha256(target.read_bytes()).hexdigest() != expected:
            raise ValueError(f'Artifact hash mismatch: {name}')
    for name, expected in manifest['source_hashes'].items():
        target = (run/'source'/name).resolve()
        if not target.is_relative_to(run/'source') or hashlib.sha256(target.read_bytes()).hexdigest() != expected:
            raise ValueError(f'Source hash mismatch: {name}')
    result = json.loads((run/'summary.json').read_text())
    # Reset state is preserved in raw artifacts. Keep useful episode metrics,
    # not hundreds of nearly identical nested coordinates, in the Git summary.
    if 'episodes' in result:
        result['episodes'] = [{k: v for k, v in episode.items()
                               if k not in ('final_state', 'closed_lifecycle')}
                              for episode in result['episodes']]
    result.pop('resets', None)
    return dict(run_id=run.name, local_evidence=run.relative_to(ROOT).as_posix(),
                manifest_sha256=hashlib.sha256((run/'manifest.json').read_bytes()).hexdigest(),
                artifact_hashes_verified=True, verified_artifact_count=len(manifest['output_hashes']),
                git_commit=manifest['git_commit'], working_tree_dirty=bool(manifest['git_status']),
                source_hashes=manifest['source_hashes'], configuration=manifest['config'],
                isaac_version=manifest.get('isaac_version'), hardware=manifest.get('gpu_before'), result=result)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('runs', nargs='+', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    target = args.output.resolve()
    if not target.is_relative_to(ROOT) or target.exists():
        parser.error('Output must be a new file inside this checkout')
    summaries = [summarize(run) for run in args.runs]
    write_json(target, dict(schema_version=1, attempts=summaries))
    print(json.dumps({'output': str(target), 'attempts': len(summaries),
                      'passed': sum(item['result']['passed'] for item in summaries)}, indent=2))


if __name__ == '__main__':
    main()
