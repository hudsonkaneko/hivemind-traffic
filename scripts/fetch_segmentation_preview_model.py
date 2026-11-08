"""Fetch one pinned pretrained model for offline preview; never executes weights.

Only the three public model files below are downloaded. Existing caches are
verified and never silently repaired or replaced. The full model remains in
ignored project runtimes; no simulator, training library or remote code loads.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import re
from urllib.request import urlopen
import uuid


ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / 'configs/perception/pretrained_segmentation_preview_v01.json'
MODEL_ID = 'nvidia/segformer-b0-finetuned-cityscapes-1024-1024'
REVISION = '21b3847fae21ddee674abd31129307b6a1235bd9'
WEIGHTS_SHA256 = '027ed78d8ff9c535df5a66361c92b9673cca3cb923cbeb8c5802385b6b93194c'
WEIGHTS_BYTES = 14957601
FILES = ('config.json', 'preprocessor_config.json', 'pytorch_model.bin')
LABELS = ('road', 'sidewalk', 'building', 'wall', 'fence', 'pole', 'traffic light',
          'traffic sign', 'vegetation', 'terrain', 'sky', 'person', 'rider',
          'car', 'truck', 'bus', 'train', 'motorcycle', 'bicycle')


def validate_config(config):
    if (config.get('version') != 'pretrained-segmentation-preview-v01'
            or config.get('training') is not False
            or config.get('new_simulator_runs') is not False):
        raise ValueError('Explicit inference-only/no-simulator configuration required')
    expected = dict(id=MODEL_ID, revision=REVISION, weights_sha256=WEIGHTS_SHA256,
                    weights_bytes=WEIGHTS_BYTES)
    model = config.get('model', {})
    if not isinstance(model, dict) or any(model.get(key) != value for key, value in expected.items()):
        raise ValueError('Model identity, revision and weight integrity pins must match the reviewed model')
    return expected


def _url(name):
    return f'https://huggingface.co/{MODEL_ID}/resolve/{REVISION}/{name}'


def _cache_directory():
    directory = ROOT / 'runtimes/perception-preview/models' / REVISION
    if not directory.resolve().is_relative_to(ROOT.resolve()):
        raise ValueError('Model cache must remain inside the project')
    return directory


def _file_record(path, name):
    if not path.is_file() or not path.resolve().is_relative_to(path.parent.resolve()):
        raise ValueError('Missing or escaping cached model file: ' + name)
    digest = hashlib.sha256()
    size = 0
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
            size += len(chunk)
    record = dict(url=_url(name), sha256=digest.hexdigest(), bytes=size)
    if name == 'pytorch_model.bin' and (size != WEIGHTS_BYTES or record['sha256'] != WEIGHTS_SHA256):
        raise ValueError('Pretrained weight size/SHA-256 does not match the published pin')
    return record


def _validate_metadata(directory):
    model = json.loads((directory/'config.json').read_text(encoding='utf-8'))
    labels = {str(index): label for index, label in enumerate(LABELS)}
    if (model.get('model_type') != 'segformer' or model.get('id2label') != labels
            or model.get('architectures') != ['SegformerForSemanticSegmentation']):
        raise ValueError('Downloaded model metadata does not describe the reviewed 19-class head')
    processor = json.loads((directory/'preprocessor_config.json').read_text(encoding='utf-8'))
    expected = dict(do_normalize=True, do_resize=True, size=512, resample=2,
                    reduce_labels=False, image_mean=[.485,.456,.406], image_std=[.229,.224,.225])
    if any(processor.get(key) != value for key, value in expected.items()):
        raise ValueError('Downloaded preprocessing differs from the reviewed model defaults')


def _verify_cache(directory):
    manifest_path = directory/'download-manifest.json'
    if not manifest_path.is_file():
        raise ValueError('Existing cache has no download manifest; preserved without modification: ' + str(directory))
    manifest = json.loads(manifest_path.read_text(encoding='utf-8'))
    if (manifest.get('schema_version') != 1 or manifest.get('model_id') != MODEL_ID
            or manifest.get('revision') != REVISION or set(manifest.get('files', {})) != set(FILES)):
        raise ValueError('Cached model manifest does not match the reviewed model')
    for name in FILES:
        recorded = manifest['files'][name]
        if (not isinstance(recorded, dict)
                or not re.fullmatch('[0-9a-f]{64}', str(recorded.get('sha256', '')))
                or _file_record(directory/name, name) != recorded):
            raise ValueError('Cached file failed integrity verification; preserved: ' + name)
    _validate_metadata(directory)
    return manifest


def fetch_model(config):
    """Return ``(local_directory, manifest)`` after download or cache verification."""
    validate_config(config)
    directory = _cache_directory()
    if directory.exists():
        return directory, _verify_cache(directory)
    directory.parent.mkdir(parents=True, exist_ok=True)
    staging = directory.parent / ('.download-' + REVISION + '-' + uuid.uuid4().hex[:8])
    staging.mkdir()
    records = {}
    try:
        for name in FILES:
            maximum = WEIGHTS_BYTES if name == 'pytorch_model.bin' else 65536
            received = 0
            with urlopen(_url(name), timeout=60) as response, (staging/name).open('xb') as target:
                while chunk := response.read(65536):
                    received += len(chunk)
                    if received > maximum:
                        raise ValueError('Downloaded file exceeded its bounded size: ' + name)
                    target.write(chunk)
            records[name] = _file_record(staging/name, name)
        _validate_metadata(staging)
        manifest = dict(schema_version=1, model_id=MODEL_ID, revision=REVISION,
            downloaded_utc=datetime.now(timezone.utc).isoformat(), files=records,
            training=False, new_simulator_runs=False, weights_executed=False,
            source_model_card=f'https://huggingface.co/{MODEL_ID}/blob/{REVISION}/README.md',
            license_name='NVIDIA Source Code License for SegFormer; noncommercial research/evaluation',
            license_url='https://github.com/NVlabs/SegFormer/blob/master/LICENSE')
        (staging/'download-manifest.json').write_text(json.dumps(manifest, indent=2)+'\n', encoding='utf-8')
        if directory.exists():
            raise FileExistsError('Another process created the model cache; staged download preserved')
        staging.rename(directory)
        return directory, _verify_cache(directory)
    except Exception as error:
        if staging.exists():
            (staging/'download-failure.json').write_text(json.dumps(dict(error=repr(error),
                recorded_files=records, training=False, new_simulator_runs=False), indent=2)+'\n', encoding='utf-8')
        raise


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=CONFIG)
    parser.add_argument('--check', action='store_true', help='Validate pins and show URLs without downloading')
    args = parser.parse_args(argv)
    if Path.cwd().resolve() != ROOT.resolve():
        parser.error('Run from the project root')
    config_path = args.config.resolve()
    if not config_path.is_relative_to(ROOT.resolve()):
        parser.error('Configuration must remain inside the project')
    config = json.loads(config_path.read_text(encoding='utf-8'))
    validate_config(config)
    if args.check:
        print(json.dumps(dict(model_id=MODEL_ID, revision=REVISION,
            cache=str(_cache_directory().relative_to(ROOT)), files={name:_url(name) for name in FILES},
            training=False, new_simulator_runs=False), indent=2))
        return 0
    directory, manifest = fetch_model(config)
    print(json.dumps(dict(directory=str(directory.relative_to(ROOT)), files=manifest['files'],
                         training=False, new_simulator_runs=False), indent=2))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
