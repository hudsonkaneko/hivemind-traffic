"""Resolve the reviewed v05 pilot against existing immutable road manifests."""
from copy import deepcopy
import hashlib
import json
from pathlib import Path

from experiments.highway_v05_curriculum import CURRICULUM_DEFAULTS
from experiments.highway_v05_support import validate_config

ROOT = Path(__file__).resolve().parents[1]


def prepare():
    if Path.cwd().resolve() != ROOT:
        raise ValueError('Run from the project root')
    old = json.loads((ROOT/'experiments/configs/highway_population_v4.json').read_text())
    bank_path = ROOT/'outputs/highway-environment/v05/network-bank/bank.json'
    bank = json.loads(bank_path.read_text())
    cfg = deepcopy(old)
    cfg.update(schema='highway_environment_v05', revision='continuous-maneuver-environment-v05',
        training_authorized=False, readiness=None, output_directory='outputs/highway-environment/v05',
        reward_version='highway_environment_v05_stable_merge',
        objective_version='shared_v3_near_limit_and_stable_maneuver_v05',
        curriculum=deepcopy(CURRICULUM_DEFAULTS),
        model=dict(observation_size=99,state_size=2400,num_agents=24,driving_local_size=51,
                   driving_message_size=8,driving_mask_start=30,population_critic=True),
        environment=dict(horizon=800,control_target_version='v3',lane_change_duration=3.,
                         stabilization_seconds=8.,native_warmup_seconds=0.,traffic_mode='sustained'))
    cfg['training'].update(seed=9507,seed_start=6800000,seed_end=6800999,
        max_agent_transitions=250000,max_wall_seconds=3600,final_evaluation_reserve_seconds=600,
        schedule='Resumable competence grades; retained all-task mixture; max20% fresh failed-training retries',
        counting='Critic/PPO active decisions and interrupted reservations share250k; initial/final evaluations share3600s')
    cfg['population'].pop('training_counts',None)
    cfg['population'].update(training_fractions=[.125,.25,.5,.75,1.],
        scope='1..24 AV fixed initial cohort;100% cases initial-only; later native inflow in mixed cases; actual fraction logged',
        assignment='Seeded initial cohort; no live controller handoff; six nearest fresh messages')
    cfg['contract'].update(actor_observation_size=99,critic_state_size=2400,max_agents=24,
        observation_semantics='highway_environment_v05_actor_99',
        neighboring_lane_context='Measured lateral position/velocity projects an entering vehicle into the affected lane corridor; source occupancy persists until clear. Actor, teacher, reward target and guard share this observed context; no future intentions.',
        warm_start_compatibility='Explicit insertion of six zero-weight motion inputs before messages; fresh24-slot critic; v3semantic transfer',
        motion='Native SUMO3-second lane changes; observable committed target; no pose interpolation; no learned steering',
        post_merge='8s complete window after lateralcompletion;1.5s headway,3s TTC,<=5% interventions,>=25% allowed-distance progress',
        motion_fields=['native_lateral_position_over_lane_width','native_lateral_speed_over_3',
            'committed_lane_direction','maneuver_in_progress','maneuver_remaining_distance_fraction','time_since_completion_over_5'])
    cfg['reward_extensions']=dict(stable_merge_bonus=2.,unnecessary_reversal_penalty=.5,
        note='One-shot stable bonus; evidence-backed unnecessary reversal only; legacy entryevent remains distinct')
    cfg['training_cases']=deepcopy(old['training_cases'])
    for layout in bank['layouts']:
        if layout['split']=='train':
            for task in cfg['training']['tasks']:
                cfg['training_cases'].append(dict(layout,id=layout['id']+'-'+task,task=task,density='medium'))
    validations=[r['case'] for r in old['validation_cases']]
    layouts=[r for r in bank['layouts'] if r['split']=='validation']
    cfg['validation_cases']=[]
    for group in range(6):
        for task in ('cruise','following','passing','merge'):
            if group==4:
                case=deepcopy(next(c for c in old['training_cases'] if c['task']==task and 'i110' in c['id']))
                case.update(split='validation',id='v05-validation-i110-'+task,
                            partition_note='Shared engineering road; disjoint validation demand seeds, not held-out geometry')
            elif group==5:
                case=deepcopy(next(c for c in validations if c['task']==task and c['through_lanes']==5))
            else:
                case=dict(layouts[group%2],id=f'v05-val-{group}-{task}',task=task,density='medium')
            total=[8,16,24,24,16,24][group]
            count=[1,4,12,24,8,18][group]
            if task in ('following','passing') and count==total: count=total-1
            cfg['validation_cases'].append(dict(case=case,controlled_count=count,total_vehicles=total,
                seed=6900000+len(cfg['validation_cases']),speed_limit_m_s=[12.,18.,22.352,26.8224,18.,26.8224][group],
                communication='none' if group==0 else 'local',packet_loss=.3 if group==5 else 0.,
                difficulty=[0,0,1,1,2,2][group],traffic_mode='sustained' if group in (2,4,5) else 'initial'))
    cfg['evaluation'].update(protocol='24fixed initial/final whole-episode cases; new treatment, not a like-for-like oldenvironment comparison',
        nominal_gate='Existing safety/intervention/efficiency gates plus>=90% completed8s stablemerges permergecase',
        tested_population_counts=sorted({r['controlled_count'] for r in cfg['validation_cases']}),
        native_baseline='ZeroAV initial nativecohorts8/16/24; not a learning episode',
        limitations='Single-seed bounded development; heldoutunchanged; nophysical/sensor qualification or livehandoff')
    cfg['evaluation'].pop('unseen_population_counts',None)
    cfg['preflight']=dict(learning=False,max_wall_seconds=900,
        requirements='Full nominal teacher tasks; continuousmotion;1/12/24slots;lateinflow;API/seed/export/zero-update gradient checks')
    cfg['network_bank']=dict(path=bank_path.relative_to(ROOT).as_posix(),
        sha256=hashlib.sha256(bank_path.read_bytes()).hexdigest(),
        scope='New120/240m upstream train ramps,180m validation ramps; existing2..5lane andI110bankretained')
    validate_config(cfg)
    path=ROOT/'experiments/configs/highway_environment_v05.json'
    path.write_text(json.dumps(cfg,indent=2)+'\n',encoding='utf-8')
    return path


if __name__=='__main__':
    print(prepare())
