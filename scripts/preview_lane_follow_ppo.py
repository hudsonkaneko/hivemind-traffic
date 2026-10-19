"""Visible inference-only native worker; use the bounded preview supervisor."""
from copy import deepcopy
import argparse
import asyncio
import hashlib
import importlib.metadata
import json
from pathlib import Path
import platform
import sys
import struct
import time
import traceback
import zlib

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from experiments.probe_support import process_sample, write_json
from experiments.lane_follow_ppo_protocol import checkpoint_identity, digest, load_protocol, verify_sources
from experiments.preview_lane_follow_ppo import STUDY, compare_trace, preview_sources, verify_preview_job
from experiments.report_lane_follow_ppo import _read
from experiments.verify_lane_following_lab import EXPECTED_ACCEPTANCE
from scripts.run_lane_follow_ppo import deterministic_action, normalization_state, transition_record
from scripts.probe_vehicle_lab import lab_runtime_provenance


def wait_for_capture(capture, *, render, guard, flush, clock=time.monotonic,
                     timeout_s=15, schedule=asyncio.ensure_future):
    """Pump render-only updates until the native async capture explicitly ends."""
    future = schedule(capture.wait_for_result(completion_frames=30))
    until = clock() + timeout_s
    try:
        while not future.done():
            guard()
            if clock() >= until:
                raise TimeoutError('Viewport capture completion deadline')
            render()
        if not future.result():
            raise RuntimeError('Native viewport capture reported failure')
        guard()
        # Installed viewport.utility tests use this after wait_for_result to
        # finish renderer-owned file writes before opening/hashing the PNG.
        flush()
        guard()
    finally:
        if not future.done():
            future.cancel()


def verify_capture_png(path, expected_dimensions=(1440, 900)):
    """Check complete PNG framing, dimensions and every chunk CRC without Kit."""
    payload = Path(path).read_bytes()
    if payload[:8] != b'\x89PNG\r\n\x1a\n':
        raise ValueError('Capture is not a PNG')
    offset, dimensions, seen_data = 8, None, False
    while offset + 12 <= len(payload):
        length = struct.unpack('>I', payload[offset:offset+4])[0]
        kind = payload[offset+4:offset+8]
        end = offset + 12 + length
        if end > len(payload):
            raise ValueError('Capture PNG has an incomplete chunk')
        data = payload[offset+8:offset+8+length]
        checksum = struct.unpack('>I', payload[offset+8+length:end])[0]
        if zlib.crc32(kind + data) & 0xffffffff != checksum:
            raise ValueError('Capture PNG chunk checksum mismatch')
        if offset == 8:
            if kind != b'IHDR' or length != 13:
                raise ValueError('Capture PNG lacks its initial image header')
            dimensions = struct.unpack('>II', data[:8])
        seen_data |= kind == b'IDAT'
        if kind == b'IEND':
            if length != 0 or end != len(payload) or not seen_data or dimensions != expected_dimensions:
                raise ValueError('Capture PNG dimensions/end marker differ')
            return {'width': dimensions[0], 'height': dimensions[1], 'bytes': len(payload),
                    'sha256': hashlib.sha256(payload).hexdigest()}
        offset = end
    raise ValueError('Capture PNG is incomplete: no IEND')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    output = args.output.resolve()
    if not output.is_relative_to(ROOT / 'outputs' / STUDY) or not output.is_dir():
        parser.error('Use the bounded preview supervisor')
    if (output / 'native-result.json').exists():
        parser.error('A completed preview package is immutable')
    job, manifest = _read(output / 'resolved-config.json'), _read(output / 'manifest.json')
    if manifest['status'] != 'running' or manifest['config'] != job or set(manifest['source_hashes']) != set(preview_sources()):
        raise ValueError('Preview requires its exact running supervisor package')
    verify_sources(manifest)
    verify_preview_job(job)
    with (output / 'worker-started.json').open('x', encoding='utf-8') as stream:
        json.dump({'checkpoint_identity': job['checkpoint_identity']}, stream)
    protocol = load_protocol()
    app = env = viewport = raw = model = normalizer = None
    rows, captures, handles = [], [], []
    started, deadline = time.perf_counter(), time.monotonic() + 285
    result = {'passed': False, 'mode': job['mode'], 'training_started': False, 'optimizer_steps': 0,
              'excluded_from_selection_and_evaluation_counts': True}
    try:
        from isaaclab.app import AppLauncher
        launcher = AppLauncher({'headless': False, 'device': 'cpu', 'visualizer': ['kit'],
            'kit_args': '--/app/window/width=1440 --/app/window/height=900 '
                        '--/app/runLoops/main/rateLimitEnabled=false --/app/vsync=false'})
        app = launcher.app
        from traffic.physics_session import prepare_vehicle_runtime
        result['runtime_setup'] = prepare_vehicle_runtime()
        import isaaclab
        import torch
        from stable_baselines3 import PPO
        from stable_baselines3.common.vec_env import VecNormalize
        from environments.lane_following_lab import LaneFollowingLabCfg, LaneFollowingLabEnv
        from environments.lane_follow_sb3 import make_lane_follow_sb3_wrapper
        from experiments.lane_follow_ppo_evaluation import reduce_episode
        from omni.kit.viewport.utility import get_active_viewport, capture_viewport_to_file
        from omni.kit.renderer_capture import acquire_renderer_capture_interface
        torch.set_num_threads(4)
        result['runtime'] = lab_runtime_provenance(isaaclab, torch)
        result['versions'] = {name: importlib.metadata.version(name) for name in
                             ('stable-baselines3', 'gymnasium', 'torch', 'numpy', 'isaaclab', 'isaaclab_rl')}
        pinned = protocol['read_only_runtime_inventory']
        if (any(version != pinned[package.replace('-', '_')] for package, version in result['versions'].items())
                or platform.python_version() != pinned['python']
                or result['runtime']['isaaclab_git_commit'] != pinned['isaaclab_git']
                or result['runtime']['isaaclab_git_dirty'] is not False):
            raise RuntimeError('Preview runtime differs from the pinned pilot')
        cfg = LaneFollowingLabCfg()
        cfg.route_id, cfg.seed, cfg.reset_perturb = 'left_r60', 2101, True
        cfg.routes_file = str(ROOT / protocol['task']['route_file'])
        cfg.scene_directory = str(output / 'scene')
        cfg.render_during_step = True
        cfg.sim.log_dir = str(ROOT / 'logs' / STUDY / output.name / 'isaac')
        env = LaneFollowingLabEnv(cfg)
        raw = make_lane_follow_sb3_wrapper(env)
        write_json(output / 'initial-scene-contract.json', env.scene_contract())
        write_json(output / 'vehicle-model.json', env.vehicle.metadata)
        checkpoint = ROOT / job['checkpoint_path']
        if checkpoint_identity(checkpoint) != job['checkpoint_identity']:
            raise RuntimeError('Selected checkpoint changed before unpickling')
        model = PPO.load(checkpoint / 'model.zip', device='cpu')
        normalizer = VecNormalize.load(checkpoint / 'vecnormalize.pkl', raw)
        normalizer.training, normalizer.norm_reward = False, False
        before_norm = normalization_state(normalizer)

        def parameter_hash():
            return hashlib.sha256(b''.join(value.detach().cpu().numpy().tobytes()
                for value in model.policy.state_dict().values())).hexdigest()

        before_params = parameter_hash()

        def refuse_learning(*args, **kwargs):
            result['prohibited_learning_attempt'] = True
            raise RuntimeError('This illustrative replay forbids learning and optimizer steps')

        model.learn = refuse_learning
        model.policy.optimizer.step = refuse_learning
        viewport = get_active_viewport()
        if viewport is None:
            raise RuntimeError('Preview requires an active native viewport')
        viewport.resolution = (1440, 900)

        def guard():
            if (output / 'STOP').exists() or time.monotonic() >= deadline:
                raise TimeoutError('Preview supervisor/deadline stop')

        def capture(name, mode, *, requested_s=None, label):
            guard()
            camera = env.render_snapshot(mode, render=False)
            viewport.camera_path = camera['camera_path']
            before_tick = env.episode_tick
            for _ in range(8):
                env.sim.render()
            path = output / name
            capture_handle = capture_viewport_to_file(viewport, str(path))
            handles.append(capture_handle)
            wait_for_capture(capture_handle, render=env.sim.render, guard=guard,
                flush=acquire_renderer_capture_interface().wait_async_capture,
                timeout_s=min(15, max(0, deadline - time.monotonic())))
            image = verify_capture_png(path)
            if env.episode_tick != before_tick:
                raise RuntimeError('Capture advanced physics')
            captures.append({'file': name, **image, 'native_completion_confirmed': True,
                'requested_sim_seconds': requested_s,
                'label': label, 'camera_state_at_request_not_exact_frame_timestamp': True, **camera})
            write_json(output / 'captures.json', captures)

        observation, extras = env.reset(seed=4101, options={'perturb': True})
        first = deepcopy(extras['reset'])
        write_json(output / 'seed-4101-reset.json', first)
        raw_observation = observation['policy'].cpu().numpy()
        capture('preview-00s-follow.png', 'follow', requested_s=0, label='Initial perturbed reset before policy steps')
        captured = {0}
        print('LAB_LANE_TASK_READY', flush=True)
        for _ in range(2100):
            guard()
            requested, action = deterministic_action(model, normalizer, raw_observation)
            before = time.perf_counter()
            returned = raw.step(action)
            row = transition_record(env, returned, requested, action, time.perf_counter() - before)
            rows.append(row)
            raw_observation = returned[0]
            # Lab has already reset when done=True: never mislabel its new pose.
            if returned[2][0]:
                break
            viewport.camera_path = env.render_snapshot('follow', render=False)['camera_path']
            for second in job['capture_sim_seconds']:
                if second not in captured and row['tick'] >= second * 120:
                    capture(f'preview-{second:02d}s-follow.png', 'follow', requested_s=second,
                            label='Selected policy illustrative replay')
                    captured.add(second)
        write_json(output / 'seed-4101-trajectory.json', rows)
        result['assessment'] = reduce_episode(rows, first, env.route, dict(EXPECTED_ACCEPTANCE))
        result['evaluation_frozen'] = before_norm == normalization_state(normalizer) and before_params == parameter_hash()
        result['normalization_before'] = before_norm
        result['normalization_after'] = normalization_state(normalizer)
        result['policy_parameters_sha256'] = before_params
        result['checkpoint_identity'] = job['checkpoint_identity']
        terminal = bool(rows and (rows[-1]['terminated'] or rows[-1]['truncated']))
        capture('preview-scene-overview.png', 'overview',
                label='Post-auto-reset scene overview; not terminal state' if terminal else 'Current nonterminal scene overview')
        result['overview_post_auto_reset'] = terminal
        write_json(output / 'inspection-ready.json', {'inspection_hold_wall_seconds': 20,
            'episode_tick': env.episode_tick, 'episode_serial': env.episode_serial,
            'post_auto_reset': terminal, 'training_started': False})
        print('LAB_LANE_PPO_PREVIEW_INSPECTION_READY=' + str(output), flush=True)
        until = time.monotonic() + 20
        while time.monotonic() < until:
            guard()
            env.sim.render()
            time.sleep(.025)
        reference = ROOT / job['reference_run']
        result['heldout_replay_comparison'] = compare_trace(rows, first,
            _read(reference / 'seed-4101-trajectory.json'), _read(reference / 'seed-4101-reset.json'))
        result['captures_skipped'] = [dict(requested_sim_seconds=s,
            reason='Not reached before terminal auto-reset') for s in job['capture_sim_seconds'] if s not in captured]
        result['scene_contract'] = env.scene_contract()
        write_json(output / 'scene-contract.json', result['scene_contract'])
        result['passed'] = bool(result['assessment']['evidence_valid'] and result['evaluation_frozen']
            and terminal and result['scene_contract']['passed'] and result['scene_contract']['static_hashes_unchanged']
            and not result.get('prohibited_learning_attempt') and len(captures) >= 2)
    except Exception as error:
        result.update(passed=False, error=repr(error), traceback=traceback.format_exc())
        traceback.print_exc()
    finally:
        write_json(output / 'seed-4101-trajectory.json', rows)
        write_json(output / 'captures.json', captures)
        try:
            if viewport is not None and env is not None:
                viewport.camera_path = '/OmniverseKit_Persp'
                env.sim.render()
            handles.clear()
            viewport = raw = model = normalizer = None
            if env is not None:
                env.close()
                result['viewport_cleanup'] = deepcopy(env.viewport_cleanup)
                result['contact_reporting'] = deepcopy(env.contact_reporting)
                result['passed'] = bool(result.get('passed') and env.viewport_cleanup['passed']
                    and env.contact_reporting['cleanup']['passed'])
                env = None
        except Exception as error:
            result.update(passed=False, close_error=repr(error))
            traceback.print_exc()
        result.update(total_wall_seconds=time.perf_counter() - started, process=process_sample())
        write_json(output / 'native-result.json', result)
        print('LAB_LANE_PPO_PREVIEW_RESULT=' + json.dumps(result), flush=True)
        if app is not None:
            app.close()
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
