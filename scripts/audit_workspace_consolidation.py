"""Read-only source comparison; write hash evidence under outputs/consolidation."""
import argparse
import hashlib
import json
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SKIP = {'.git', '.pytest_cache', '__pycache__', 'runtimes'}
TRANSIENT = {'scripts/consolidate_workspace.ps1', 'scripts/resume_consolidation.ps1'}


def files(root):
    for parent, dirs, names in __import__('os').walk(root, followlinks=False):
        dirs[:] = sorted(d for d in dirs if d not in SKIP and not d.startswith('.venv')
                         and not (Path(parent) / d).is_symlink())
        for name in sorted(names):
            path = Path(parent) / name
            rel = path.relative_to(root).as_posix()
            if rel not in TRANSIENT and not path.is_symlink():
                yield rel, path


def digest(path):
    with path.open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--archive', type=Path, required=True)
    args = parser.parse_args()
    output = ROOT / 'outputs/consolidation'
    output.mkdir(parents=True, exist_ok=True)
    summary = {}
    for name in ('highwaysim', 'hivemind-traffic', 'highwaysim-backup-20260921-161242'):
        source = args.archive / name
        if not source.is_dir():
            raise FileNotFoundError(source)
        def compare(item):
            rel, path = item
            target = ROOT / rel
            original = digest(path)
            current = digest(target) if target.is_file() else None
            return {'path': rel, 'source_sha256': original,
                         'active_sha256': current,
                         'status': 'missing' if current is None else
                                   'identical' if original == current else 'different'}
        with ThreadPoolExecutor(max_workers=8) as pool:
            rows = list(pool.map(compare, files(source)))
        counts = {key: sum(r['status'] == key for r in rows)
                  for key in ('identical', 'different', 'missing')}
        summary[name] = {'source': str(source), 'files': len(rows), **counts}
        (output / f'{name}.json').write_text(json.dumps(rows, indent=2) + '\n')
        print(name, counts, flush=True)
    (output / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n')
    return int(any(s['missing'] for s in summary.values()))


if __name__ == '__main__':
    raise SystemExit(main())
