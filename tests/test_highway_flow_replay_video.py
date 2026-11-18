"""Round-9 replay video tooling: pinned inputs, top-down renderer and 3D camera option (no SUMO, no Isaac)."""
import gzip
import hashlib
import json
import re
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
RECORDER = ROOT / 'scripts/record_highway_flow_rl_round9_episode.py'
EVIDENCE = ROOT / 'documentation/evidence/highway-merge-training-v06'
NETWORK = """<net version="1.20">
  <location netOffset="0,0" convBoundary="0,-200,0,0" origBoundary="0,-200,0,0" projParameter="!"/>
  <edge id="e" from="a" to="b" priority="1">
    <lane id="e_0" index="0" speed="20" length="200" width="3.2" shape="-1.6,0 -1.6,-200"/>
    <lane id="e_1" index="1" speed="20" length="200" width="3.2" shape="1.6,0 1.6,-200"/>
  </edge>
  <junction id="a" type="dead_end" x="0" y="0" incLanes="" intLanes="" shape="0,0"/>
  <junction id="b" type="dead_end" x="0" y="-200" incLanes="e_0 e_1" intLanes="" shape="0,-200"/>
</net>
"""


def _pinned(name):
    return re.search(rf"^{name} = '([^']+)'", RECORDER.read_text(encoding='utf-8'), re.M).group(1)


def test_recorder_pins_the_round9_config_policy_and_i110_validation_row():
    raw = (ROOT / 'configs/training/highway-flow-rl-v06-round9.json').read_bytes().replace(b'\r\n', b'\n')
    assert hashlib.sha256(raw).hexdigest() == _pinned('CONFIG_SHA256')
    row = json.loads(raw.decode('utf-8'))['validation_rows'][0]  # --case-index default
    assert row['case']['id'] == 'v05-validation-i110-merge' and row['seed'] == 6900019
    assert _pinned('POLICY') == 'outputs/highway-flow-rl/v06/round-09/last.pt'
    source = RECORDER.read_text(encoding='utf-8')
    assert "'visual_recording'" in source and 'max_diagnostic_episodes=before + 1' in source  # one charged episode


def test_published_replay_evidence_matches_the_round9_validation_and_the_pinned_policy():
    video = json.loads((EVIDENCE / 'round-09-replay-video.json').read_text(encoding='utf-8'))
    recording = video['recording']
    assert recording['training'] is False and recording['policy_sha256'] == _pinned('POLICY_SHA256')
    assert recording['config_sha256'] == _pinned('CONFIG_SHA256')
    assert recording['all_compared_metrics_match'] is True
    assert all(item['equal'] for item in recording['comparison'].values())
    assert recording['ledger'] == dict(command='visual_recording', category='preparation', episodes=1,
                                       diagnostic_episodes_after=128, training_decisions=0)
    for render in (video['top_down'], video['isaac_3d']):
        assert render['source_recording_sha256'] == recording['recording_sha256'] and render['limitation']
    assert video['isaac_3d']['passed'] and video['isaac_3d']['max_position_error_m'] == 0.
    assert video['isaac_3d']['decoded_frames'] == video['isaac_3d']['frames'] == 300
    assert all((EVIDENCE / name).is_file() for name in video['screenshots'].values())


def _episode(directory, frames=6):
    run = directory / 'episode-1-test'
    run.mkdir(parents=True)
    (run / 'network.net.xml').write_text(NETWORK, encoding='utf-8')
    recorded, rows = [], []
    for i in range(frames):
        t = round(.1 * (i + 1), 1)
        ego = dict(id='agent_0', x=-1.6 if i < 3 else 1.6, y=-10. - 2. * i, heading=180., speed=15. + i, length=4.6,
                   width=1.8, controlled=True, merge_complete=i >= 3,
                   stable_merge_status='succeeded' if i == frames - 1 else 'pending')
        other = dict(id='car_1', x=1.6, y=-40. - 2. * i, heading=180., speed=18., length=4.6, width=1.8, controlled=False,
                     merge_complete=False, stable_merge_status=None)
        recorded.append(dict(time=t, vehicles=[ego, other]))
        rows.append(dict(time=t, agents=dict(agent_0=dict(
            requested_acceleration=.5, override_causes=['speed_ceiling'] if i == 4 else [],
            v04_reward_audit=dict(pre_action=dict(target=dict(eligible=i != 1, target_m_s=18.))),
            v05_audit=dict(merge_active=True)))))
    (directory / 'recording.json').write_text(json.dumps(dict(frames=recorded)), encoding='utf-8')
    with gzip.open(run / 'actions.jsonl.gz', 'wt') as journal:
        journal.write('\n'.join(json.dumps(row) for row in rows) + '\n')
    summary = directory / 'summary.json'
    summary.write_text(json.dumps(dict(stable_merge=dict(agent_0=dict(
        succeeded=True, entry_time_s=.2, lateral_merge_complete_time_s=.4)))), encoding='utf-8')
    return summary


def test_topdown_renderer_draws_every_recorded_frame_and_reports_the_gate_times(monkeypatch):
    pytest.importorskip('sumolib')
    image = pytest.importorskip('PIL.Image')
    from scripts import render_highway_flow_topdown as renderer
    base = ROOT / 'outputs/pytest-replay-video'  # inside outputs/, like every other generated file
    import shutil
    shutil.rmtree(base, ignore_errors=True)
    summary = _episode(base / 'episode')
    calls = []
    monkeypatch.setattr(renderer.subprocess, 'run', lambda command, **options: calls.append(command))
    try:
        renderer.main([str(base / 'episode'), '--summary', str(summary), '--title', 'test', '--ffmpeg', 'ffmpeg',
                       '--output', str(base / 'render')])
        frames = sorted((base / 'render/frames').glob('*.png'))
        assert [f.name for f in frames] == [f'{i:05d}.png' for i in range(6)]
        with image.open(frames[0]) as first, image.open(frames[-1]) as last:
            assert first.size == last.size == (renderer.W, renderer.H)
            assert first.tobytes() != last.tobytes()  # the car and the speed trace moved
            ego = tuple(int(renderer.COLORS['ego'][i:i + 2], 16) for i in (1, 3, 5))
            road = last.crop((0, renderer.ROAD_TOP + 40, renderer.W, renderer.ROAD_BOTTOM))
            assert ego in {color for _, color in road.getcolors(maxcolors=1 << 20)}  # followed car is drawn on the road
        report = json.loads((base / 'render/render.json').read_text(encoding='utf-8'))
        assert report['frames'] == 6 and report['schematic'] is True
        assert (report['merge_entry_s'], report['lateral_merge_s'], report['stable_merge_passed_s']) == (.2, .4, .6)
        assert len(calls) == 1 and calls[0][-1].endswith('render.mp4') and '10' in calls[0]  # 10 fps = recorded cadence
        with pytest.raises(FileExistsError):  # never overwrite an earlier render
            renderer.main([str(base / 'episode'), '--summary', str(summary), '--title', 'test', '--ffmpeg', 'ffmpeg',
                           '--output', str(base / 'render')])
    finally:
        shutil.rmtree(base, ignore_errors=True)


def test_isaac_staging_keeps_frames_and_adds_only_declared_metadata():
    import shutil
    from scripts import stage_highway_flow_isaac_replay as staging
    base = ROOT / 'outputs/pytest-replay-staging'
    shutil.rmtree(base, ignore_errors=True)
    try:
        summary = _episode(base / 'episode')
        merge = json.loads(summary.read_text(encoding='utf-8'))
        (base / 'summary.json').write_text(json.dumps(dict(merge, training=False, deterministic=True, policy='p.pt',
                                                           policy_sha256='ab' * 32)), encoding='utf-8')
        manifest = staging.stage(base, base / 'isaac-input')
        source = json.loads((base / 'episode/recording.json').read_text(encoding='utf-8'))
        staged = json.loads((base / 'isaac-input/recording.json').read_text(encoding='utf-8'))
        assert staged['frames'] == source['frames'] and set(staged) - set(source) == set(staging.ADDED_KEYS)
        assert [event['time'] for event in staged['merge_events']] == [.2, .4] and staged['training'] is False
        assert (base / 'isaac-input/network.net.xml').read_text(encoding='utf-8') == NETWORK
        assert manifest['staged_recording_sha256'] == staging.sha256(base / 'isaac-input/recording.json')
        with pytest.raises(FileExistsError):
            staging.stage(base, base / 'isaac-input')
        (base / 'summary.json').write_text(json.dumps(dict(merge, training=True, deterministic=True)), encoding='utf-8')
        with pytest.raises(ValueError):
            staging.stage(base, base / 'other')
    finally:
        shutil.rmtree(base, ignore_errors=True)


def test_isaac_replay_camera_offset_defaults_to_the_original_view_and_rejects_bad_values(monkeypatch):
    from scripts import render_i110_merge_replay as replay
    source = Path(replay.__file__).read_text(encoding='utf-8')
    assert "'--camera-offset', type=float, nargs=3, default=[62., 46., 78.]" in source
    assert 'eye = target+Gf.Vec3d(*args.camera_offset)' in source
    monkeypatch.chdir(ROOT)
    for offset in (['10', '10', '0'], ['nan', '1', '5']):
        monkeypatch.setattr(sys, 'argv', ['render', '--recording', 'outputs/none', '--output', 'outputs/pytest-none',
                                          '--ffmpeg', 'ffmpeg', '--camera-offset', *offset])
        with pytest.raises(SystemExit):
            replay.main()
    assert not (ROOT / 'outputs/pytest-none').exists()
