"""Fetch the fixed official NVIDIA people assets into the project-local cache."""
import argparse,hashlib,json,urllib.request
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--check',action='store_true');args=p.parse_args()
    manifest=json.loads((ROOT/'configs/perception/segmentation_dataset_assets_v02.json').read_text())
    if Path.cwd().resolve()!=ROOT:raise ValueError('Run from project root')
    for row in manifest['sources']:
        path=(ROOT/row['path']).resolve()
        if not path.is_relative_to(ROOT/'runtimes/perception-assets'):raise ValueError('Cache path escapes project')
        if not row['url'].startswith('https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/6.0/Isaac/People/Characters/'):raise ValueError('Unexpected asset source')
        if not path.exists():
            if args.check:raise FileNotFoundError(path)
            with urllib.request.urlopen(row['url'],timeout=60) as response:data=response.read(64*1024*1024+1)
            if len(data)>64*1024*1024 or hashlib.sha256(data).hexdigest()!=row['sha256']:raise ValueError('Downloaded asset size/hash failure')
            path.parent.mkdir(parents=True,exist_ok=True);temp=path.with_name(path.name+'.partial');temp.write_bytes(data);temp.replace(path)
        if hashlib.sha256(path.read_bytes()).hexdigest()!=row['sha256']:raise ValueError('Existing cache checksum mismatch: '+str(path))
    if not args.check:
        target=ROOT/'runtimes/perception-assets/source-manifest.json';target.write_text(json.dumps(manifest,indent=2))
    print(json.dumps(dict(verified=True,files=len(manifest['sources']),training=False)))

if __name__=='__main__':main()
