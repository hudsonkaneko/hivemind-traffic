"""Rescore frozen development journals only; never launch a simulator or optimizer."""
from __future__ import annotations

import argparse
from collections import Counter
import csv
import gzip
import json
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from experiments import run_highway_flow_rl_v03 as runner
from traffic.highway_flow_rl_v03_contract import validate_config
from traffic.highway_flow_rl_v03_rewards import (BrakingDiagnostic, map_evidence,
    safe_speed_target, speed_error_reward, jerk_reward)
from environments.highway_environment_v05 import ContinuousMotionLaneGraph


def summary(rows):
    return dict(decisions=len(rows), target_eligible=sum(r['target_eligible'] for r in rows),
        measured_post_speed=sum(r['next_speed_m_s'] is not None for r in rows),
        original_reward=sum(r['original_reward'] for r in rows),
        revised_reward=sum(r['revised_reward'] for r in rows),
        original_jerk_cost=sum(r['old_jerk'] for r in rows),
        huber_jerk_cost=sum(r['new_jerk'] for r in rows),
        original_cruise_reward=sum(r['old_cruise'] for r in rows),
        speed_error_cost=sum(r['speed_error'] for r in rows),
        original_detour_cost=sum(r['old_detour'] for r in rows),
        revised_detour_cost=sum(r['new_detour'] for r in rows),
        avoidable_braking_decisions=sum(r['avoidable_braking'] for r in rows),
        avoidable_braking_events=sum(r['persistent_event'] for r in rows),
        exclusions=dict(Counter(r['exclusion'] for r in rows if r['exclusion'])))


def audit_case(source, config, output):
    journals=list(source.glob('evaluation/case-00/episode-*/actions.jsonl.gz'))
    if len(journals)!=1: raise ValueError('Expected one saved development journal')
    path=journals[0]; network=path.parent/'network.net.xml'
    report_path=source/'evaluation/00.json'; saved_config=source/'resolved-config.json'
    report=json.loads(report_path.read_text())
    old=json.loads(saved_config.read_text())
    matches=[r for r in old['validation_rows'] if r['seed']==report['seed'] and r['case']['id']==report['case_id']]
    if len(matches)!=1: raise ValueError('Trace must match one development row, never held-out')
    row=matches[0]
    if runner.file_digest(ROOT/row['case']['network'])!=row['case']['network_sha256']:
        raise ValueError('Saved source network provenance changed')
    graph=ContinuousMotionLaneGraph(network)
    graph.speed_limits={k:min(v,row['speed_limit_m_s']) for k,v in graph.speed_limits.items()}
    with gzip.open(path,'rt') as stream: decisions=[json.loads(line) for line in stream]
    # These next-decision snapshots are actual recorded post-step states.
    states={(round(a['v02_reward_audit']['pre_action']['time_s'],6),agent):
            a['v02_reward_audit']['pre_action']['ego_state']
            for d in decisions for agent,a in d['agents'].items()}
    recording=source/'evaluation/case-00/recording.json'
    inputs=[path,network,report_path,saved_config,ROOT/row['case']['network']]
    if recording.exists():
        inputs.append(recording)
        for f in json.loads(recording.read_text())['frames']:
            for s in f['vehicles']: states.setdefault((round(f['time'],6),s['id']),s)
    rows=[]; diagnostic=BrakingDiagnostic(); max_identity_error=0.; verified=0
    weights=config['v03_reward_weights']
    for d in decisions:
        for agent,info in d['agents'].items():
            pre=info['v02_reward_audit']['pre_action']; state=pre['ego_state']
            if len(pre['observation117'])!=117: raise ValueError('Missing actual actor observation')
            target=safe_speed_target(state,pre['context'],map_evidence(graph,state))
            post=states.get((round(d['time'],6),agent))
            next_speed=post['speed'] if post is not None else None
            if next_speed is not None and info['realized_acceleration'] is not None:
                error=abs(next_speed-state['speed']-.1*info['realized_acceleration'])
                max_identity_error=max(error,max_identity_error); verified+=1
                if error>1e-6: raise ValueError('Recorded velocity/acceleration linkage failed')
            terms=info['reward_terms']; event=info['v02_reward_audit']['detour_event']
            detour=bool(event and event['evidence']['optional_detour'])
            if bool(terms['route_detour'])!=detour: raise ValueError('First-motion event accounting drift')
            jerk=jerk_reward(info['realized_jerk_m_s3'],weights['jerk_huber'])
            speed=speed_error_reward(target,next_speed,weights['speed_error'])
            lane=-weights['route_detour']*detour
            diagnosis=diagnostic.observe(agent,pre['time_s'],target,state['speed'],
                info['requested_acceleration'],info['feasible_acceleration_ceiling'])
            original=sum(terms.values())
            revised=original-terms['jerk']-terms['cruise_speed']-terms['route_detour']+jerk+speed+lane
            rows.append(dict(agent=agent,time_s=d['time'],speed_m_s=state['speed'],next_speed_m_s=next_speed,
                requested_acceleration=info['requested_acceleration'],realized_acceleration=info['realized_acceleration'],
                realized_jerk=info['realized_jerk_m_s3'],safe_ceiling=info['feasible_acceleration_ceiling'],
                target_eligible=target['eligible'],target_m_s=target['target_m_s'],
                exclusion=';'.join(target['reasons']),obligations=';'.join(target['obligations']),
                bounds=json.dumps(target['bounds'],separators=(',',':')),
                original_reward=original,revised_reward=revised,old_jerk=terms['jerk'],new_jerk=jerk,
                old_cruise=terms['cruise_speed'],speed_error=speed,
                old_detour=terms['route_detour'],new_detour=lane,
                avoidable_braking=diagnosis['braking'],persistent_event=diagnosis['persistent_event']))
    csv_path=output/(source.name+'.csv')
    with csv_path.open('w',newline='',encoding='utf-8') as stream:
        writer=csv.DictWriter(stream,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
    focal=[r for r in rows if r['agent']=='agent_0']
    return dict(case_id=report['case_id'],seed=report['seed'],task=report['task'],
        aggregate=summary(rows),focal=summary(focal),per_car={a:summary([r for r in rows if r['agent']==a]) for a in diagnostic.rows},
        avoidable_braking=diagnostic.rows,
        focal_windows={name:summary([r for r in focal if lo<=r['time_s']<=hi])
            for name,lo,hi in [('first_dip',6.6,10.2),('second_dip',17.4,21.1)]} if report['case_id']=='v05-validation-i110-merge' else {},
        linkage_checks=verified,max_speed_identity_error_m_s=max_identity_error,
        inputs={p.relative_to(ROOT).as_posix():runner.file_digest(p) for p in inputs},
        raw_csv=csv_path.relative_to(ROOT).as_posix(),raw_csv_sha256=runner.file_digest(csv_path))


def run(config_path,output):
    started=time.monotonic();config_path=runner.project_path(config_path);output=runner.project_path(output)
    if output.exists() or not output.is_relative_to(runner.OUTPUT_ROOT):
        raise ValueError('Use a fresh v03 output directory')
    cfg=validate_config(json.loads(config_path.read_text()))
    fingerprint=runner.file_digest(config_path)
    sources=runner.hashes(); sources[Path(__file__).relative_to(ROOT).as_posix()]=runner.file_digest(Path(__file__))
    with runner.LedgerSession(cfg,'offline_reward_audit',output,fingerprint,started=started,episodes=0) as session:
        output.mkdir(parents=True)
        runner.save(output/'resolved-config.json',cfg)
        try:
            cases=[]
            for source in sorted((ROOT/'outputs/highway-flow-rl/v02').glob('development-case-*')):
                runner.check_deadline(session.deadline)
                cases.append(audit_case(source,cfg,output))
            if len(cases)!=5: raise ValueError('Expected the five frozen development cases')
            result=dict(schema='highway_flow_rl_v03_saved_trace_audit_v1',training=False,optimizer_steps=0,
                new_simulator_runs=0,native_preflight_passed=False,driving_improvement_measured=False,
                scope='Offline reward rescore of frozen development actor; rewards cannot change these trajectories',
                terminal_speed_note='No invented post-state: final unobserved/disappeared samples receive no speed error; coverage reported',
                config_sha256=fingerprint,source_hashes=sources,cases=cases,
                previous_charged_seconds=session.totals['total'],elapsed_seconds=session.elapsed(),
                total_charged_seconds=session.totals['total']+session.elapsed())
            runner.save(output/'metrics.json',result)
        except BaseException:
            import traceback
            runner.save(output/'failure.json',dict(traceback=traceback.format_exc(),training=False,new_simulator_runs=0))
            raise
    return result


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config',type=Path,required=True);parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args(); result=run(args.config,args.output)
    print(json.dumps({k:result[k] for k in ('training','new_simulator_runs','elapsed_seconds','total_charged_seconds')}))
