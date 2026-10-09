"""Compare two fresh, completed loop-showcase runs without changing their evidence.

This checks same-machine numerical repeatability, not cross-platform determinism
or learned-policy competence. Full resolved settings and captured source hashes
must match. Git commits may differ: actual captured source content, not the Git
label, defines the code/asset comparison. Run with --first PATH --second PATH.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path

from experiments.loop_showcase_config import assess, validate_config
from experiments.probe_support import ROOT, finish, run_package


INPUT_FILES=('summary.json','manifest.json','trajectory.json','resolved-config.json','scene-contract.json')
SOURCES=['experiments/loop_showcase_repeatability.py','experiments/loop_showcase_config.py',
         'experiments/probe_support.py']
POSITION_TOLERANCE_M=.02
SPEED_TOLERANCE_M_S=.02


def _digest(data):
    return hashlib.sha256(data).hexdigest()


def _hashes(value):
    return (isinstance(value,dict) and bool(value) and all(isinstance(name,str) and name
        and isinstance(digest,str) and len(digest)==64 and all(c in '0123456789abcdef' for c in digest)
        for name,digest in value.items()))


def _same(left,right):
    try:return json.dumps(left,sort_keys=True,allow_nan=False)==json.dumps(right,sort_keys=True,allow_nan=False)
    except (TypeError,ValueError):return False


def resolve_run(path):
    candidate=Path(path).resolve()
    if candidate.parent!=(ROOT/'outputs/loop_showcase').resolve() or not candidate.is_dir():
        raise ValueError('Inputs must be immediate run directories under canonical outputs/loop_showcase')
    return candidate


def _contained_file(root,name):
    # Recorded manifests may contain Windows separators even when inspected on
    # a different machine. Absolute/parent escapes are never accepted.
    normalized=name.replace('\\','/')
    if ':' in normalized or normalized.startswith('/'):
        raise ValueError('Absolute evidence path is not allowed: '+name)
    path=(root/normalized).resolve()
    if not path.is_relative_to(root.resolve()) or not path.is_file():
        raise ValueError('Missing or escaped evidence file: '+name)
    return path


def load_run(path):
    directory=resolve_run(path);documents={};fingerprints={};errors=[];verified={}
    for name in INPUT_FILES:
        data=_contained_file(directory,name).read_bytes()
        documents[name]=json.loads(data);fingerprints[name]=_digest(data)
        verified[name]=fingerprints[name]
    manifest=documents['manifest.json']
    if not isinstance(manifest,dict):
        errors.append('Manifest must be an object')
    else:
        outputs=manifest.get('output_hashes')
        if not _hashes(outputs):errors.append('Output hashes are missing or malformed')
        else:
            for name in INPUT_FILES:
                if name!='manifest.json' and outputs.get(name)!=fingerprints[name]:
                    errors.append('Missing/mismatched required output hash: '+name)
            for name,expected in outputs.items():
                try:
                    normal=name.replace('\\','/')
                    actual=verified.get(normal)
                    if actual is None:
                        actual=_digest(_contained_file(directory,name).read_bytes());verified[normal]=actual
                    if actual!=expected:errors.append('Output hash mismatch: '+name)
                except (OSError,ValueError) as error:errors.append(str(error))
        sources=manifest.get('source_hashes')
        if not _hashes(sources):errors.append('Source hashes are missing or malformed')
        else:
            source_root=directory/'source'
            if not source_root.resolve().is_relative_to(directory):errors.append('Source snapshot escapes run')
            else:
                for name,expected in sources.items():
                    try:
                        actual=_digest(_contained_file(source_root,name).read_bytes())
                        if actual!=expected:errors.append('Source snapshot hash mismatch: '+name)
                    except (OSError,ValueError) as error:errors.append(str(error))
    return dict(run_id=directory.name,relative_path=directory.relative_to(ROOT).as_posix(),
        summary=documents['summary.json'],manifest=manifest,trajectory=documents['trajectory.json'],
        config=documents['resolved-config.json'],scene_contract=documents['scene-contract.json'],
        file_hashes=fingerprints,load_errors=errors,verified_output_count=len(verified))


def _runtime_identity(run):
    manifest=run['manifest'];scene=run['scene_contract'];gpu=manifest.get('gpu_before',{})
    vehicle=scene.get('vehicle',{})
    identity=dict(isaac_version=manifest.get('isaac_version'),supervisor_python=manifest.get('python'),
        platform=manifest.get('platform'),logical_cpus=manifest.get('logical_cpus'),
        gpu_name=gpu.get('name'),gpu_driver=gpu.get('driver'),
        prepared_vehicle_factory_sha256=vehicle.get('factory_sha256'))
    for name,value in identity.items():
        if name=='logical_cpus':
            if type(value) is not int or value<=0:raise ValueError('Missing/invalid logical CPU count')
        elif not isinstance(value,str) or not value.strip():raise ValueError('Missing runtime identity: '+name)
    if not _hashes({'factory':identity['prepared_vehicle_factory_sha256']}):
        raise ValueError('Malformed prepared vehicle build Factory hash')
    return identity


def compare_evidence(first,second):
    failures=[];metrics={};identities={}
    if not isinstance(first,dict) or not isinstance(second,dict):
        return dict(passed=False,failures=['Two loaded evidence objects are required'])
    runs=(first,second);ids=[run.get('run_id') for run in runs]
    if not all(isinstance(v,str) and v for v in ids) or ids[0]==ids[1]:
        failures.append('Two distinct independent run IDs are required')
    if not _same(first.get('config'),second.get('config')):failures.append('Full resolved configurations differ')
    manifests=[run.get('manifest') for run in runs]
    if (not all(isinstance(m,dict) and _hashes(m.get('source_hashes')) for m in manifests)
            or manifests[0].get('source_hashes')!=manifests[1].get('source_hashes')):
        failures.append('Nonempty exact source hashes must match')
    for label,run in zip(('first','second'),runs):
        try:
            if run.get('load_errors'):raise ValueError('Input integrity errors: '+repr(run['load_errors']))
            cfg=run['config'];manifest=run['manifest'];summary=run['summary']
            if not isinstance(cfg,dict):raise ValueError('Resolved configuration is missing')
            validate_config(cfg)
            if cfg['mode']!='showcase':raise ValueError('Repeatability requires normal showcase mode')
            if (not isinstance(manifest,dict) or manifest.get('status')!='passed'
                    or not manifest.get('finished_utc') or not _same(manifest.get('config'),cfg)):
                raise ValueError('Manifest must be completed, passed and match its resolved configuration')
            command=manifest.get('command')
            if (not isinstance(command,list) or len(command)<4 or command[-2]!='--output'
                    or not isinstance(command[-1],str) or Path(command[-1].replace('\\','/')).name!=run['run_id']):
                raise ValueError('Manifest lacks a distinct supervised process launch for this run')
            if (not isinstance(summary,dict) or summary.get('passed') is not True
                    or type(summary.get('exit_code')) is not int or summary['exit_code']!=0
                    or summary.get('source_changed_during_run')!=[]):
                raise ValueError('Summary must pass, exit zero and preserve captured sources')
            gates=summary.get('gates')
            if not isinstance(gates,dict) or not gates or any(value is not True for value in gates.values()):
                raise ValueError('Summary contains missing, false or malformed acceptance gates')
            passes=summary.get('passed_vehicle_ids');changes=summary.get('completed_lane_changes')
            if (not isinstance(passes,list) or not all(isinstance(v,str) and v for v in passes)
                    or len(passes)!=len(set(passes)) or type(summary.get('completed_passes')) is not int
                    or summary['completed_passes']!=len(passes)):
                raise ValueError('Summary pass identities/count are missing or inconsistent')
            rows=run.get('trajectory')
            reassessed=assess(rows,cfg,summary.get('contact_report_count'),passes,changes)
            if not reassessed['passed']:
                raise ValueError('Raw trajectory fails acceptance: '+repr(reassessed.get('gates')))
            # Additional fields are independent records of the event counters;
            # they must finish at the summary count and never move backwards.
            for name,expected in (('completed_passes',len(passes)),('completed_lane_changes',changes)):
                counts=[row.get(name) for row in rows]
                if (any(type(v) is not int or v<0 for v in counts) or counts[-1]!=expected
                        or any(b<a for a,b in zip(counts,counts[1:]))):
                    raise ValueError('Trajectory event counter mismatch: '+name)
            identities[label]=_runtime_identity(run)
            metrics[label]=dict(samples=len(rows),completed_passes=len(passes),passed_vehicle_ids=sorted(passes),
                completed_lane_changes=changes,distance_m=rows[-1]['traveled_distance_m'])
        except (KeyError,TypeError,ValueError,OverflowError,AttributeError) as error:
            failures.append(label+': '+str(error))
    if len(identities)==2 and identities['first']!=identities['second']:
        failures.append('Recorded runtime/machine identity differs')
    if len(metrics)==2:
        for field in ('completed_passes','passed_vehicle_ids','completed_lane_changes'):
            if metrics['first'][field]!=metrics['second'][field]:failures.append('Event outcomes differ: '+field)
    result=dict(passed=False,run_ids=ids,failures=failures,input_metrics=metrics,runtime_identities=identities,
        acceptance=dict(position_tolerance_m=POSITION_TOLERANCE_M,speed_tolerance_m_s=SPEED_TOLERANCE_M_S),
        scope='Two fresh supervised showcase processes, same full configuration and actual captured source/asset hashes plus recorded runtime/hardware; not cross-platform determinism',
        git_policy='Git commit labels may differ only because equality is based on exact captured source hashes')
    if failures:return result
    position=[math.dist(a['position_m'],b['position_m']) for a,b in zip(first['trajectory'],second['trajectory'])]
    speed=[abs(a['speed_m_s']-b['speed_m_s']) for a,b in zip(first['trajectory'],second['trajectory'])]
    if max(position)>POSITION_TOLERANCE_M:failures.append('Aligned position difference exceeds tolerance')
    if max(speed)>SPEED_TOLERANCE_M_S:failures.append('Aligned speed difference exceeds tolerance')
    result.update(passed=not failures,samples=len(position),physics_hz=first['config']['physics_hz'],
        max_position_difference_m=max(position),max_speed_difference_m_s=max(speed),
        max_position_difference_tick=position.index(max(position))+1,max_speed_difference_tick=speed.index(max(speed))+1)
    return result


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--first',type=Path,required=True);parser.add_argument('--second',type=Path,required=True)
    args=parser.parse_args(argv)
    try:directories=[resolve_run(args.first),resolve_run(args.second)]
    except ValueError as error:parser.error(str(error))
    config=dict(schema_version=1,study='loop_showcase_repeatability',
        position_tolerance_m=POSITION_TOLERANCE_M,speed_tolerance_m_s=SPEED_TOLERANCE_M_S,
        inputs=[dict(run_id=p.name,relative_path=p.relative_to(ROOT).as_posix()) for p in directories])
    loaded=[];error=None
    try:
        for directory,record in zip(directories,config['inputs']):
            run=load_run(directory);loaded.append(run);record['file_hashes']=run['file_hashes']
    except (OSError,ValueError) as exc:error=str(exc)
    output,manifest=run_package('loop_showcase_repeatability',config,SOURCES)
    result=compare_evidence(*loaded) if error is None else dict(passed=False,failures=[error])
    result['input_evidence']=config['inputs'];finish(output,manifest,result)
    print(json.dumps(dict(output=str(output),**result),indent=2,allow_nan=False))
    return 0 if result['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
