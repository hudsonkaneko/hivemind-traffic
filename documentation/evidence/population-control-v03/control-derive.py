"""Offline derivation of immutable v03 control probes, never starts SUMO."""
import argparse
import csv
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import uuid

ROOT=Path(__file__).resolve().parents[3]
INITIAL=ROOT/'outputs/population-control-v03/verify-20261105T053939Z-7e23d78b'
REFINED=ROOT/'outputs/population-control-v03/verify-20261105T054418Z-b5f89825'


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def extract(directory):
    raw=json.loads((directory/'control-summary.json').read_text())
    episodes=[]
    for episode in raw['episodes']:
        r=episode['report'];s=episode['spec']
        path=directory/(s['id']+'.csv')
        assert digest(path)==episode['raw_csv_sha256']
        episodes.append(dict(spec=s,decisions=episode['decisions'],overrides=episode['override_count'],
            override_rate=episode['override_count']/episode['decisions'],
            mean_fraction_of_speed_limit=r['mean_fraction_of_speed_limit'],
            successful_merges=r['successful_merges'],passes=r['passes'],
            collisions=r['collisions'],teleports=r['teleports'],overspeed_steps=r['overspeed_steps'],
            sampled_emergency_steps=r['emergency_braking_observed'],
            native_warning_lines=episode['native_warnings']['warning_lines'],
            max_teacher_reward_target_error=episode['max_teacher_reward_target_error'],
            teacher_reward_target_mae=episode['teacher_reward_target_mae'],
            max_abs_acceleration=episode['max_abs_acceleration'],
            trace_sha256=episode['trace_sha256'],raw_csv_sha256=digest(path),
            episode_directory=episode['episode_directory']))
    return dict(raw_run=directory.relative_to(ROOT).as_posix(),summary_sha256=digest(directory/'control-summary.json'),
        resolved_config_sha256=digest(directory/'resolved-config.json'),source_commit=raw['source_commit'],
        source_sha256_before=raw['source_sha256_before'],source_sha256_after=raw['source_sha256_after'],
        sources_unchanged=raw['source_sha256_before']==raw['source_sha256_after'],
        actor_sha256=raw['actor_sha256'],actor_sha256_after=raw['actor_sha256_after'],
        frozen_actor_unchanged=raw['frozen_actor_unchanged'],runtime=raw['runtime'],sumo_version=raw['episodes'][0]['runtime'],
        wall_seconds=raw['wall_seconds'],repeat_trace_equal=raw['repeat_trace_equal'],episodes=episodes)


def find(run, name):
    return next(e for e in run['episodes'] if e['spec']['id']==name)


def plot(path, initial, refined):
    svg=['<svg xmlns="http://www.w3.org/2000/svg" width="1100" height="1090" viewBox="0 0 1100 1090">',
        '<rect width="1100" height="1090" fill="#fbfaf7"/>',
        '<style>text{font:16px sans-serif;fill:#25313a}.small{font-size:13px}</style>',
        '<text x="55" y="35" font-size="22">Current-map merge correction: requested and realized acceleration</text>',
        '<text x="55" y="60" class="small">Train3-lane merge; seed6520003;1 controlled car /16 total; safeguards unchanged</text>']
    for index,(directory,name,label) in enumerate(((INITIAL,'train-3-lane-merge-v2-teacher','Historical v2 teacher'),
            (INITIAL,'train-3-lane-merge-v3-teacher','Initial v3: stopline context, before native0.1m offset correction'),
            (REFINED,'train-3-lane-merge-v3-teacher','Refined v3: source-calibrated native stopline offset'))):
        rows=list(csv.DictReader((directory/(name+'.csv')).open()))
        rows=[r for r in rows if 7.5<=float(r['time_after'])<=12.5]
        top=100+index*225
        svg+=[f'<text x="80" y="{top-12}">{label}</text>']
        for value in (-4.5,-2,0,2):
            y=top+(2.5-value)/7.5*170
            svg += [f'<path d="M80 {y:.3f}H1050" stroke="#ddd"/>',f'<text x="28" y="{y+5:.3f}" class="small">{value}</text>']
        for value in (8,9,10,11,12):
            x=80+(value-7.5)/5*970
            svg += [f'<text x="{x-8:.2f}" y="{top+192}" class="small">{value}s</text>']
        for key,color,style in [('realized','#266e9b',''),('requested','#bd621d','stroke-dasharray="5 3"')]:
            points=' '.join(f"{80+(float(r['time_after'])-7.5)/5*970:.3f},{top+(2.5-float(r[key]))/7.5*170:.3f}" for r in rows)
            svg += [f'<polyline points="{points}" fill="none" stroke="{color}" stroke-width="2" {style}/>']
    svg+=['<text x="80" y="772" class="small">Blue solid: realized acceleration; orange dashed: requested acceleration (m/s²)</text>',
          '<text x="55" y="812">Teacher override rate in matched training-only probes</text>']
    pairs=[('Passing1AV',find(initial,'train-3-lane-passing-v2-teacher'),find(initial,'train-3-lane-passing-v3-teacher')),
           ('Merge1AV',find(refined,'train-3-lane-merge-v2-teacher'),find(refined,'train-3-lane-merge-v3-teacher')),
           ('Merge3-lane4AV',find(refined,'train-3-lane-merge-v2-teacher-multi'),find(refined,'train-3-lane-merge-v3-teacher-multi')),
           ('Merge5-lane4AV',find(refined,'train-5-lane-merge-v2-teacher-multi'),find(refined,'train-5-lane-merge-v3-teacher-multi'))]
    for i,(label,old,new) in enumerate(pairs):
        x=90+i*250
        for j,(e,color) in enumerate(((old,'#bd621d'),(new,'#266e9b'))):
            pct=e['override_rate']*100;h=pct/2.5*145
            svg += [f'<rect x="{x+j*65}" y="{1000-h:.3f}" width="48" height="{max(h,1):.3f}" fill="{color}"/>',
                f'<text x="{x+j*65-2}" y="{991-h:.3f}" class="small">{pct:.3f}%</text>']
        svg += [f'<text x="{x-10}" y="1025" class="small">{label}</text>']
    svg += ['<text x="65" y="1060" class="small">Orange: v2; blue: v3. Passing uses initial run; merge uses refined run. No learned weights changed.</text>','</svg>']
    path.write_text('\n'.join(svg),encoding='utf-8')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path)
    args=parser.parse_args()
    output=((ROOT/args.output) if args.output else ROOT/'outputs/population-control-v03'/f'derived-{datetime.now(timezone.utc):%Y%m%dT%H%M%SZ}-{uuid.uuid4().hex[:8]}').resolve()
    if not output.is_relative_to((ROOT/'outputs').resolve()) or output.exists():
        raise ValueError('Output must be a NEW directory beneath project outputs')
    output.mkdir(parents=True)
    initial,refined=extract(INITIAL),extract(REFINED)
    result=dict(schema='population_control_v3_evidence',learning=False,scope='train-only bounded development probes; no held-out acceptance',
        initial=initial,refined=refined,total_wall_seconds=initial['wall_seconds']+refined['wall_seconds'],
        total_decisions=sum(e['decisions'] for run in (initial,refined) for e in run['episodes']),
        safeguards=dict(speed_mode=31,lane_change_mode=512,step_seconds=.1,speed_ceiling_mph=60,headway_floor_seconds=1.5),
        observation_semantics='highway_graph_entry_and_lane_merge_v3',reward_version='highway_stage1_shared_longitudinal_target_v3',
        notes=['93-field shape retained; merge-context semantics changed, so old actor is transfer initialization, not drop-in equivalent',
            'Initial run omitted runner dependency snapshot while root was editing runner instrumentation; control runtime source hashes remained stable',
            'Refined run includes unchanged runner dependency snapshot. Later edge-level stop-offset validation does not alter tested default maps; root final preflight verifies final source',
            'Initial v3 is preserved as superseded calibration; refinement adds source-defined POSITION_EPS0.1m, not a fitted outcome threshold',
            'v3 currently rejects minor-link visibility other than4.5m and edge/lane stop offsets above0.1m; fixed controlled type has no larger vehicle/JM override',
            'Existing policy protection thresholds and override definition are unchanged; zero teacher overrides do not imply an accepted learned actor'],
        sources=['https://raw.githubusercontent.com/eclipse-sumo/sumo/v1_27_1/src/microsim/cfmodels/MSCFModel.cpp',
            'https://raw.githubusercontent.com/eclipse-sumo/sumo/v1_27_1/src/microsim/MSVehicle.cpp',
            'https://raw.githubusercontent.com/eclipse-sumo/sumo/v1_27_1/src/config.h.cmake',
            'https://raw.githubusercontent.com/eclipse-sumo/sumo/v1_27_1/src/microsim/cfmodels/MSCFModel_ACC.cpp',
            'https://sumo.dlr.de/docs/Simulation/Intersections.html'])
    (output/'control-summary.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n',encoding='utf-8')
    plot(output/'control-results.svg',initial,refined)
    print(json.dumps(dict(output=str(output),decisions=result['total_decisions'],wall_seconds=result['total_wall_seconds'])))


if __name__=='__main__':
    main()
