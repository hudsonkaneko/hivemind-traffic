"""Build a separate six-road v05 bank with explicit short/long ramp geometry.

The v02 right-entry topology and its crossing/foe checks are reused. Historical
networks are untouched. This command compiles geometry and does not run traffic,
evaluate a policy, or train a model.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import xml.etree.ElementTree as ET

from scripts.build_highway_curriculum_v2 import ROOT, build, sha256


SCHEMA = 'highway_environment_v05_network_bank_v1'


def specifications():
    """Existing mainline dimensions, new train ramps and intermediate validation."""
    return [dict(split='train', lanes=n, length=1800 + 100 * n, bend=20 * (n % 2),
                 speed_mph=60 if n % 2 else 50, ramp_upstream_distance=120. if n % 2 == 0 else 240.)
            for n in (2, 3, 4, 5)] + [
        dict(split='validation', lanes=n, length=1900 + 100 * n, bend=-30,
             speed_mph=55, ramp_upstream_distance=180.) for n in (2, 4)]


def build_bank(output):
    output = Path(output).resolve()
    if not output.is_relative_to(ROOT / 'outputs') or output.exists():
        raise ValueError('Bank output must be a fresh project outputs directory')
    output.mkdir(parents=True)
    layouts = []
    for specification in specifications():
        options = dict(specification)
        split = options.pop('split')
        ident = f'{split}-{options["lanes"]}-lane-ramp{options["ramp_upstream_distance"]:g}'
        entry = build(output / ident, **options)
        network = ROOT / entry['network']
        ramp = ET.parse(network).find("./edge[@id='mlk_on']/lane")
        if ramp is None:
            raise ValueError('Compiled bank network lost its ramp')
        entry.update(id=ident, split=split, ramp_upstream_distance_m=options['ramp_upstream_distance'],
                     compiled_ramp_length_m=float(ramp.attrib['length']))
        layouts.append(entry)
    bank = dict(schema=SCHEMA, layouts=layouts,
        source_sha256={name: sha256(ROOT / name) for name in
                       ('scripts/build_highway_environment_v05_bank.py', 'scripts/build_highway_curriculum_v2.py')},
        historical_networks_preserved=True, imported_i110_unchanged=True,
        traffic_run=False, learning_performed=False, held_out_test_run=False,
        scope='Six compiled development roads; varied upstream ramp distance, unchanged v02 topology checks')
    manifest = output / 'bank.json'
    manifest.write_text(json.dumps(bank, indent=2, sort_keys=True) + '\n', encoding='utf-8')
    return dict(bank=manifest.relative_to(ROOT).as_posix(), sha256=sha256(manifest), layouts=len(layouts))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=ROOT / 'outputs/highway-environment/v05/network-bank')
    args = parser.parse_args(argv)
    print(json.dumps(build_bank(args.output), sort_keys=True))


if __name__ == '__main__':
    main()
