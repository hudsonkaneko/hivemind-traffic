"""Assess matched physical motion and measured timing across showcase profiles.

This is an observational cross-profile assessment, never a repeatability claim.
Inputs are real, completed run directories; captured evidence is never modified.
"""
from __future__ import annotations

import argparse
from copy import deepcopy
import json
import math
import re

from experiments import loop_showcase_repeatability as repeat


CONFIG_DIFFERENCES=frozenset(('version','render_hz','evidence_profile','decorative_wheel_distance_m'))
SOURCE_DIFFERENCES=frozenset(('scripts/demo_loop_showcase.py','scripts/physics_loop_showcase.py',
    'experiments/loop_showcase_config.py','traffic/showcase_performance.py','traffic/loop_showcase_control.py',
    'visualization/loop_showcase_view.py','usd/showcase_fleet.py'))
REQUIRED_COMMON_SOURCES=frozenset(('traffic/driver_control.py','traffic/path_following.py',
    'traffic/loop_driving.py','traffic/physx_vehicle.py','traffic/physics_session.py',
    'traffic/rendered_physics_session.py','traffic/vehicle_contacts.py','traffic/showcase_clearance.py',
    'usd/physical_scene.py','usd/showcase_vehicle.py'))
CONTROL_FIELDS=('phase','throttle','brake','steering_rad','fallback','command_expires_tick',
    'controller_fallback','controller_status','controller_reason','lane',
    'completed_lane_changes','completed_passes')
FLEET_PRESENTATION_FIELDS=frozenset(('episode_id','cache_readback','wheel_visual_update_every_steps','wheel_visual_hz'))


def _physical_contract(run):
    """Normalize only verified scene packaging locations and runtime layer IDs."""
    scene=deepcopy(run['scene_contract'])
    directory=(repeat.ROOT/run['relative_path']/'scene').resolve().as_posix()
    composition=scene.get('vehicle',{}).get('scene_composition',{})
    if 'scene_directory' in composition:
        if composition['scene_directory'].replace('\\','/')!=directory:
            raise ValueError('Vehicle scene directory is outside its recorded run packaging')
        composition['scene_directory']='scene/'
    validation=scene.get('environment',{}).get('validation',{})
    if 'layers' in validation:
        normalized=[]
        for layer in validation['layers']:
            path=layer.replace('\\','/')
            if path.startswith(directory+'/'):
                normalized.append('scene/'+path[len(directory)+1:])
            elif re.fullmatch(r'anon:[0-9A-Fa-f]+:(?:World1\.usd|World1-session\.usda)',path):
                normalized.append('anon:'+path.rsplit(':',1)[-1])
            else:normalized.append(path)
        validation['layers']=sorted(normalized)
    return scene


def compare_motion(left,right):
    """Load and verify two real runs before allowing declared profile differences."""
    first,second=repeat.load_run(left),repeat.load_run(right)
    validation=repeat.compare_evidence(first,second)
    # Retain every per-run schema, integrity, safety, runtime and event check.
    # Only equality of these two explicitly assessed sets is handled here.
    failures=[failure for failure in validation['failures'] if failure not in
        ('Full resolved configurations differ','Nonempty exact source hashes must match')]
    configs=[run['config'] for run in (first,second)]
    source_maps=[run['manifest'].get('source_hashes',{}) for run in (first,second)]
    source_changes=sorted(name for name in set(source_maps[0])|set(source_maps[1])
                          if source_maps[0].get(name)!=source_maps[1].get(name))
    config_changes=sorted(name for name in set(configs[0])|set(configs[1])
                          if not repeat._same(configs[0].get(name),configs[1].get(name)))
    if any(cfg.get('version') not in ('v05','v06') for cfg in configs):
        failures.append('Cross-profile assessment requires v05/v06 showcase runs')
    if set(config_changes)-CONFIG_DIFFERENCES:
        failures.append('Unmatched physical or timing settings: '+repr(sorted(set(config_changes)-CONFIG_DIFFERENCES)))
    if set(source_changes)-SOURCE_DIFFERENCES:
        failures.append('Undeclared source differences: '+repr(sorted(set(source_changes)-SOURCE_DIFFERENCES)))
    for label,run,sources in zip(('first','second'),(first,second),source_maps):
        if not repeat._hashes(sources):failures.append(label+': invalid source hashes')
        if not REQUIRED_COMMON_SOURCES.issubset(sources):
            failures.append(label+': missing required common physics source snapshots')
        asset=run['scene_contract'].get('source_vehicle_hashes')
        if not repeat._hashes(asset):failures.append(label+': missing prepared vehicle asset hashes')
        if not repeat._hashes(run['scene_contract'].get('fleet',{}).get('asset_hashes')):
            failures.append(label+': missing background fleet asset hashes')
    try:contracts=[_physical_contract(run) for run in (first,second)]
    except (ValueError,TypeError,AttributeError) as error:
        failures.append(str(error));contracts=[first['scene_contract'],second['scene_contract']]
    for field in ('source_vehicle_hashes','vehicle','controller','environment','spawn'):
        if (field not in contracts[0] or field not in contracts[1]
                or not repeat._same(contracts[0][field],contracts[1][field])):
            failures.append('Scene physical contract differs or is missing: '+field)
    fleets=[run['scene_contract'].get('fleet') for run in (first,second)]
    if (not all(isinstance(fleet,dict) for fleet in fleets) or not repeat._same(
            {k:v for k,v in fleets[0].items() if k not in FLEET_PRESENTATION_FIELDS},
            {k:v for k,v in fleets[1].items() if k not in FLEET_PRESENTATION_FIELDS})):
        failures.append('Fleet physical contract differs or is missing')
    result=dict(passed=False,scope='Matched cross-profile motion/timing assessment; not configuration repeatability or source equivalence',
        run_ids=[first['run_id'],second['run_id']],failures=failures,
        config_differences=config_changes,source_differences=[dict(path=name,first=source_maps[0].get(name),
            second=source_maps[1].get(name)) for name in source_changes],
        acceptance=dict(position_tolerance_m=.01,speed_tolerance_m_s=.01,control_decisions='exact every physics tick'),
        input_metrics=validation.get('input_metrics'),runtime_identities=validation.get('runtime_identities'))
    if failures:return result
    rows=first['trajectory'],second['trajectory']
    if len(rows[0])!=len(rows[1]):
        failures.append('Trajectory sample counts differ');return result
    position=[];speed=[]
    for tick,(a,b) in enumerate(zip(*rows),1):
        if any(field not in a or field not in b or not repeat._same(a[field],b[field]) for field in CONTROL_FIELDS):
            failures.append('Control decisions differ or are missing at tick '+str(tick));break
        position.append(math.dist(a['position_m'],b['position_m']))
        speed.append(abs(a['speed_m_s']-b['speed_m_s']))
    if failures:return result
    result.update(samples=len(position),max_position_difference_m=max(position),max_speed_difference_m_s=max(speed))
    if max(position)>.01:failures.append('Aligned position difference exceeds 0.01 m')
    if max(speed)>.01:failures.append('Aligned speed difference exceeds 0.01 m/s')
    timings={}
    for label,run in zip(('first','second'),(first,second)):
        profile=run['summary'].get('runtime_profile')
        if (not isinstance(profile,dict) or profile.get('iterations')!=len(run['trajectory'])
                or profile.get('non_overlapping_stages') is not True or not profile.get('stages')):
            failures.append(label+': complete exclusive runtime timing is required');continue
        wall=profile.get('loop_wall_s')
        if type(wall) not in (int,float) or not math.isfinite(wall) or wall<=0:
            failures.append(label+': invalid runtime wall time');continue
        invalid_stages=False
        for stage in profile['stages'].values():
            if (not isinstance(stage,dict) or type(stage.get('count')) is not int
                    or not 0<stage['count']<=len(run['trajectory'])
                    or any(type(stage.get(key)) not in (int,float) or not math.isfinite(stage[key])
                           or stage[key]<0 for key in ('total_s','p95_ms'))):
                invalid_stages=True;break
        if invalid_stages:
            failures.append(label+': invalid per-stage timing/counts');continue
        preview=run['summary'].get('preview_timing')
        if preview is not None and (not isinstance(preview,dict) or
                type(preview.get('completed_frames')) is not int or
                preview['completed_frames']!=run['config']['duration_s']*run['config']['render_hz']):
            failures.append(label+': incomplete preview timing');continue
        timings[label]=dict(render_hz=run['config']['render_hz'],physics_samples=len(run['trajectory']),
            expected_rendered_frames=run['config']['duration_s']*run['config']['render_hz'],
            measured_rendered_frames=preview['completed_frames'] if preview else None,
            visual_audits=len(run['background_visual_checks']),loop_wall_s=wall,
            real_time_factor=run['config']['duration_s']/wall,stages=profile['stages'],
            iteration_timing=profile.get('iteration_timing'),preview_timing=preview,
            profiling_overhead=run['summary'].get('profiling_overhead'))
    result.update(passed=not failures,timings=timings)
    return result


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--first',required=True);parser.add_argument('--second',required=True)
    args=parser.parse_args(argv)
    try:result=compare_motion(args.first,args.second)
    except (OSError,ValueError,KeyError,TypeError) as error:
        result=dict(passed=False,failures=[str(error)])
    print(json.dumps(result,indent=2,allow_nan=False))
    return 0 if result['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
