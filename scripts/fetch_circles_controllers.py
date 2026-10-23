"""Download a pinned public CIRCLES model inventory into ignored project outputs."""
from pathlib import Path
import hashlib
import json
import urllib.request

ROOT = Path(__file__).resolve().parents[1]
REVISION = '3b3ee3b91afdae93075c8b90334d65748cd9ff74'
MODELS = {
    'cra': 'controllers/cra/model.onnx',
    'rl_decel3': 'controllers/09_04_rl_finaldecn3p0_maxacc0p25/super_resolution.onnx',
    'rl_ttc4': 'controllers/09_04_rl_ttc4p0scale2p0_maxacc0p25/super_resolution.onnx',
    'rll_ttc7': 'controllers/09_04_rll_ttc7p0scale1p8_maxacc0p25/super_resolution.onnx',
    'rll_ttc6p5': 'controllers/09_05_rll_ttc6p5scale1p8_maxacc0p75/super_resolution.onnx',
    'cra_interp': 'controllers/cra_acellinterp_gapinterp/super_resolution.onnx',
    'max80mph': 'controller_max_80mph/super_resolution.onnx',
}


def fetch():
    if Path.cwd().resolve() != ROOT:
        raise RuntimeError('Run from the checkout root')
    output = ROOT / 'outputs/circles_control/models' / REVISION
    output.mkdir(parents=True, exist_ok=True)
    rows = []
    files = {**{name + '.onnx': path for name, path in MODELS.items()},
             'LICENSE': 'LICENSE', 'cra-README.md': 'controllers/cra/README.md',
             'NetworkReader.cpp': 'onnx2ros/src/NetworkReader.cpp'}
    for name, path in files.items():
        url = f'https://raw.githubusercontent.com/CIRCLES-consortium/algos-stack/{REVISION}/{path}'
        data = urllib.request.urlopen(url, timeout=30).read()
        target = output / name
        if target.exists() and target.read_bytes() != data:
            raise RuntimeError('Refusing to replace divergent download: ' + name)
        target.write_bytes(data)
        rows.append(dict(file=name, source=url, sha256=hashlib.sha256(data).hexdigest(), bytes=len(data)))
    (output / 'manifest.json').write_text(json.dumps(dict(revision=REVISION, files=rows), indent=2)+'\n', encoding='utf-8')
    print(output)


if __name__ == '__main__':
    fetch()
