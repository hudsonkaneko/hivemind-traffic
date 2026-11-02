"""Prepare a self-contained reviewable road/replay ZIP; never upload it."""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import shutil
import subprocess
import sys
import zipfile

from pxr import Usd

ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from scripts.report_i110_anticipation import read,write,sha


def package(scene,output):
    scene,output=Path(scene).resolve(),Path(output).resolve()
    output.mkdir(parents=True,exist_ok=False)
    dest=output/'scene';shutil.copytree(scene,dest)
    # Keep source identity while replacing workstation-only provenance paths.
    manifest=read(dest/'manifest.json')
    manifest['source_layer_sha256']={'source-road/'+Path(k).name:v for k,v in manifest['source_layer_sha256'].items()}
    write(dest/'manifest.json',manifest)
    provenance=read(dest/'road/provenance.json');old=provenance['source_manifest']
    provenance['source_manifest']={k:old[k] for k in ('geometry_sha256','origin_6340','z_offset_m','units','up_axis','limitations')}
    write(dest/'road/provenance.json',provenance)
    shutil.copyfile(ROOT/'scripts/view_visual_twin.py',output/'view_comparison.py')
    docs=output/'progress';docs.mkdir()
    for name in ('i110-preview-2026-11-02.md','i110-team-handoff-2026-11-02.md',
                 'sam-roadmap-2026-11-02.md','repository-review-2026-11-02.md',
                 'i110-geometry-contact-refinement.md','continuation-2026-10-29.md'):
        shutil.copyfile(ROOT/'documentation'/name,docs/name)
    for name in ('i110-preview-20261102.json','repo-review-20261102.json'):
        target=docs/'evidence'/name;target.parent.mkdir(exist_ok=True)
        shutil.copyfile(ROOT/'documentation/evidence'/name,target)
    traffic=output/'traffic';traffic.mkdir()
    shutil.copyfile(ROOT/'experiments/configs/i110_preview_v1.json',traffic/'study-plan.json')
    shutil.copyfile(ROOT/'scenarios/i110/sumo-road-profiles.json',traffic/'sumo-road-profiles.json')
    for label,run in [('baseline',ROOT/'outputs/i110-ramp-cutin-20261029/development-v1/ramp-44'),
                      ('preview',ROOT/'outputs/i110-preview-20261102/development-v1/ramp-44-r240')]:
        target=traffic/label;target.mkdir()
        for name in ('network.net.xml','routes.rou.xml','scenario.sumocfg','resolved-config.json','metrics.json'):
            shutil.copyfile(run/name,target/name)
    (output/'Open Comparison.cmd').write_text('''@echo off
setlocal
cd /d "%~dp0"
if not defined HIVEMIND_ISAAC_PYTHON (
  echo Set HIVEMIND_ISAAC_PYTHON to the Python executable in your installed Isaac Sim environment.
  pause
  exit /b 1
)
"%HIVEMIND_ISAAC_PYTHON%" view_comparison.py --scene scene/world.usda
if errorlevel 1 pause
''',encoding='utf-8')
    (output/'README.md').write_text('''# I-110 baseline / preview comparison

Prepared locally for Sam's team on 2 November 2026. This ZIP is a reviewable
artifact, not evidence that it has been uploaded to GitHub.

Use an installed Isaac Sim Python (tested with Isaac Sim 6.0.1). Set
`HIVEMIND_ISAAC_PYTHON` to its executable, then double-click `Open Comparison.cmd`.
Alternatively run that Python with `view_comparison.py --scene scene/world.usda`.
No SUMO, PeMS login, private vehicle collection or training runtime is needed
for playback. The road alone is `scene/road/world.usda`; open it directly in Isaac.

The viewer starts paused at source time 48 s, before the difficult ramp encounter.
Left/blue = unchanged smooth ACC; right/green = experimental 240 m preview;
orange = ramp-origin traffic. Play/Pause, scrub, rate, vehicle selection and
Follow/Overview/Free camera are available. Both roads use one simulation clock.

The preview is NOT accepted: it still fails comfort and introduces new control-case
failures and large cut-in traffic costs. See `progress/i110-preview-2026-11-02.md`.
The source-fitted road has provisional geometry and epoch limitations; replay is
kinematic, not Isaac physical driving. Vehicle yaw/pitch is reconstructed from
lane tangents, while recorded source x/y/z remains exact under display offsets.

Included: self-contained road/marking/support layers, both recorded motion streams,
procedural reusable vehicles, portable viewer, exact demonstrated pair's traffic
inputs/metrics, frozen study contract and dated progress/roadmap reports.
The full historical raw studies and all other treatment traces remain local.
Repository-relative links in the progress notes refer to the accompanying source
checkout when they are not included in this small artifact.

Road attribution: USGS 3DEP `CA_LosAngeles_1_B23` source elevation context,
May 2024 NAIP image-interpreted envelopes, and OpenStreetMap contributors for
the initial nominal alignment. No raw lidar/imagery, private supplied car,
trained weights, credentials or workstation runtime is included.
Road fitting and reconstruction limits are in `scene/road/provenance.json`.
The current source repository is https://github.com/samfrausto/hivemind-traffic ;
the public feature branch is older until the prepared handoff is published.
''',encoding='utf-8')
    # Validate portability after moving the copied composition to its own root.
    root=Usd.Stage.Open(str(dest/'world.usda'))
    assert not root.GetCompositionErrors()
    assets=[]
    for layer in root.GetUsedLayers():
        for dependency in layer.GetExternalReferences():
            path=(Path(layer.realPath).parent/dependency).resolve()
            assert path.is_file() and path.is_relative_to(output)
            assets.append(path.relative_to(output).as_posix())
    git=['git','-c','safe.directory='+ROOT.as_posix()]
    record=dict(status='prepared locally; not uploaded',created_utc=datetime.now(timezone.utc).isoformat(),
        git_commit=subprocess.check_output(git+['rev-parse','HEAD'],cwd=ROOT).decode().strip(),
        git_status=subprocess.check_output(git+['status','--short'],cwd=ROOT).decode('utf-8'),
        source_scene_manifest_sha256=sha(scene/'manifest.json'),generator_sha256=sha(__file__),
        viewer_sha256=sha(output/'view_comparison.py'),all_composition_dependencies_inside_package=True,
        reference_checks=len(assets),files={p.relative_to(output).as_posix():sha(p) for p in output.rglob('*') if p.is_file()})
    write(output/'package-manifest.json',record)
    archive=output.with_suffix('.zip')
    with zipfile.ZipFile(archive,'x',compression=zipfile.ZIP_DEFLATED,compresslevel=6) as z:
        for p in sorted(output.rglob('*')):
            if p.is_file():z.write(p,output.name+'/'+p.relative_to(output).as_posix())
    with zipfile.ZipFile(archive) as z:assert z.testzip() is None
    archive_record=dict(status='prepared locally; not uploaded',archive=archive.name,
        sha256=sha(archive),bytes=archive.stat().st_size,files=len(record['files'])+1,
        package_manifest_sha256=sha(output/'package-manifest.json'),portable_reference_checks=len(assets))
    write(output.parent/(output.name+'-archive.json'),archive_record)
    print(json.dumps(archive_record));return archive_record


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--scene',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    package(**vars(p.parse_args()))
