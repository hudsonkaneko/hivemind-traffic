"""Approved, single-run lane PPO protocol. Importing this never starts a runtime."""
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import re

from experiments.probe_support import ROOT
from experiments.verify_lane_following_lab import source_files as task_source_files

PILOT_ID = 'lane-follow-ppo-2101-v1'
PROPOSAL_SHA256 = '5999396ed15fbdd15f82a4f49779da8c2f3458e3118a477ce0219d908681dafd'
EVIDENCE_SHA256 = '10fc4c221028683e9176fd34e9b6f4955f9afbe7109b13da70e785215caab9b2'
APPROVAL_PATH = 'experiments/lane-follow-ppo-pilot.json'


def digest(path):
    with Path(path).open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def load_protocol(root=ROOT):
    """Bind approval to the exact reviewed proposal and prior native evidence."""
    root = Path(root).resolve()
    approval = json.loads((root / APPROVAL_PATH).read_text(encoding='utf-8'))
    expected = {
        'schema_version': 1, 'pilot_id': PILOT_ID,
        'proposal': 'experiments/lane-follow-ppo-proposal.json',
        'proposal_sha256': PROPOSAL_SHA256,
        'authorization': {'approved': True, 'date': '2026-10-19',
            'user_message': 'yes start training',
            'scope': 'The reviewed single-run PPO pilot, checkpoint selection and matched held-out evaluation; no automatic extension or resume.'},
        'validation_evidence': 'documentation/evidence/lab-lane-following-20261019.json',
        'validation_evidence_sha256': EVIDENCE_SHA256,
        'smoke_process_limit_s': 300, 'training_process_limit_s': 3600,
        'evaluation_process_limit_s': 600, 'training_entrypoint': 'scripts/run_lane_follow_ppo.py',
    }
    if json.dumps(approval, sort_keys=True) != json.dumps(expected, sort_keys=True):
        raise ValueError('The approval record differs from the reviewed one-run protocol')
    if digest(root / approval['proposal']) != PROPOSAL_SHA256:
        raise ValueError('Reviewed proposal bytes changed; do not silently change approved parameters')
    if digest(root / approval['validation_evidence']) != EVIDENCE_SHA256:
        raise ValueError('Prior native validation evidence changed')
    proposal = json.loads((root / approval['proposal']).read_text(encoding='utf-8'))
    result = deepcopy(proposal)
    result.update(status='APPROVED_SINGLE_PILOT', launch_enabled=True,
        trainer_entrypoint=approval['training_entrypoint'], authorization=approval,
        historical_proposal_sha256=PROPOSAL_SHA256)
    return result


def source_files():
    return task_source_files() + [
        APPROVAL_PATH, 'experiments/lane-follow-ppo-proposal.json',
        'experiments/lane_follow_ppo_protocol.py', 'experiments/run_lane_follow_ppo.py',
        'experiments/lane_follow_ppo_evaluation.py', 'environments/lane_follow_ppo.py',
        'scripts/run_lane_follow_ppo.py',
        'documentation/evidence/lab-lane-following-20261019.json',
    ]


def verify_sources(manifest, root=ROOT):
    changed = [name for name, expected in manifest['source_hashes'].items()
               if not (Path(root) / name).is_file() or digest(Path(root) / name) != expected]
    if changed:
        raise ValueError('Pilot sources changed: ' + ', '.join(changed))


def _sha256(value):
    return isinstance(value, str) and re.fullmatch('[0-9a-f]{64}', value) is not None


def _link(path):
    return path.is_symlink() or bool(getattr(path, 'is_junction', lambda: False)())


def _run_inventory(directory):
    """Enumerate without following links; only the enclosing manifest is exempt."""
    pending, files = [directory], set()
    while pending:
        current = pending.pop()
        for path in current.iterdir():
            if _link(path):
                raise ValueError('Training package contains a symlink or junction')
            if path.is_dir():
                pending.append(path)
            elif path.is_file():
                if path != directory / 'manifest.json':
                    files.add(path.relative_to(directory).as_posix())
            else:
                raise ValueError('Training package contains a nonregular entry')
    return files


def _completed_training_run(directory, state, identity, root):
    run = directory.parent.parent
    manifest = json.loads((run / 'manifest.json').read_text(encoding='utf-8'))
    summary = json.loads((run / 'summary.json').read_text(encoding='utf-8'))
    resolved = json.loads((run / 'resolved-config.json').read_text(encoding='utf-8'))
    approved = load_protocol(root)
    if (manifest['status'] != 'passed' or summary['passed'] is not True
            or summary['mode'] != 'train' or summary['training_started'] is not True
            or manifest['config'] != resolved or resolved['mode'] != 'train'
            or resolved['pilot_id'] != PILOT_ID or resolved['protocol'] != approved
            or resolved['route_id'] != 'left_r60' or resolved['seeds'] != [2101]
            or resolved['stage'] is not None or resolved['controller'] != 'ppo'
            or resolved['checkpoint_path'] is not None or resolved['checkpoint_identity'] is not None):
        raise ValueError('Checkpoint requires the completed approved training run')
    final = summary['training_state']
    if (final['status'] != 'completed' or final['stop_reason'] is not None
            or type(final['control_steps']) is not int or final['control_steps'] != 65536
            or type(final['completed_updates']) is not int or final['completed_updates'] != 32
            or type(final['optimizer_steps']) is not int or not 0 < final['optimizer_steps'] <= 5120
            or type(summary['optimizer_steps']) is not int
            or summary['optimizer_steps'] != final['optimizer_steps']
            or state['optimizer_steps'] > final['optimizer_steps']
            or final['checkpoints'] != ['update-08', 'update-16', 'update-24', 'update-32']):
        raise ValueError('Training did not finish the exact approved budget')
    if (type(summary['exit_code']) is not int or summary['exit_code'] != 0
            or summary['stop_reason'] is not None or summary['source_unchanged'] is not True
            or summary['kit_after']['available'] is not True
            or type(summary['kit_after']['count']) is not int or summary['kit_after']['count'] != 0
            or summary['viewport_cleanup']['passed'] is not True
            or summary['contact_reporting']['cleanup']['passed'] is not True
            or summary['scene_contract']['passed'] is not True
            or summary['scene_contract']['static_hashes_unchanged'] is not True
            or summary['unexpected_runtime_errors'] or summary['usd_reference_warning_count'] != 0
            or summary['gpu_monitor_errors']):
        raise ValueError('Training process, scene, or cleanup was not accepted')
    hashes = manifest['source_hashes']
    if (not isinstance(hashes, dict) or set(hashes) != set(source_files())
            or not all(_sha256(value) for value in hashes.values())):
        raise ValueError('Training source inventory differs from the current pilot')
    verify_sources(manifest, root=root)
    raw_inventory = manifest['output_hashes']
    if not isinstance(raw_inventory, dict):
        raise ValueError('Malformed training output inventory')
    inventory = {name.replace('\\', '/'): value for name, value in raw_inventory.items()}
    if (len(inventory) != len(raw_inventory) or set(inventory) != _run_inventory(run)
            or not all(_sha256(value) for value in inventory.values())):
        raise ValueError('Training output inventory changed')
    for name, expected in inventory.items():
        path = run / name
        if not path.resolve().is_relative_to(run) or digest(path) != expected:
            raise ValueError('Training artifact hash mismatch: ' + name)
    if any(inventory.get('source/' + name.replace('\\', '/')) != expected
           for name, expected in hashes.items()):
        raise ValueError('Captured training sources differ from the manifest')
    claim_path = root / 'outputs/lab_lane_ppo' / (PILOT_ID + '.claim.json')
    claim = json.loads(claim_path.read_text(encoding='utf-8'))
    if (_link(claim_path) or manifest['training_claim'] != claim_path.relative_to(root).as_posix()
            or claim['pilot_id'] != PILOT_ID or claim['output'] != run.relative_to(root).as_posix()
            or claim['approval_sha256'] != digest(root / APPROVAL_PATH)
            or claim['smoke'] != resolved['smoke_evidence']):
        raise ValueError('Training package does not own the one-run approval claim')
    initial_scene = json.loads((run / 'initial-scene-contract.json').read_text(encoding='utf-8'))
    provenance = state['provenance']
    expected_provenance = dict(proposal_sha256=PROPOSAL_SHA256,
        source_hashes=hashes, runtime=summary['runtime'], versions=summary['versions'],
        asset_hashes=initial_scene['static_hashes'],
        git_commit=manifest['git_commit'], git_status=manifest['git_status'])
    if (state['config'] != approved or provenance != expected_provenance
            or initial_scene['passed'] is not True
            or not isinstance(provenance['asset_hashes'], dict) or not provenance['asset_hashes']
            or not all(_sha256(value) for value in provenance['asset_hashes'].values())
            or summary['scene_contract']['static_hashes'] != provenance['asset_hashes']):
        raise ValueError('Checkpoint source/runtime/asset/approval provenance differs')
    pinned = approved['read_only_runtime_inventory']
    expected_versions = {package: pinned[key] for package, key in (
        ('stable-baselines3', 'stable_baselines3'), ('gymnasium', 'gymnasium'),
        ('torch', 'torch'), ('numpy', 'numpy'), ('isaaclab', 'isaaclab'), ('isaaclab_rl', 'isaaclab_rl'))}
    runtime = summary['runtime']
    if (summary['versions'] != expected_versions or runtime['python_version'] != pinned['python']
            or runtime['torch_version'] != pinned['torch']
            or runtime['isaaclab_git_commit'] != pinned['isaaclab_git']
            or runtime['isaaclab_git_dirty'] is not False):
        raise ValueError('Checkpoint runtime differs from the approved installed runtime')
    checkpoints = summary['checkpoints']
    if (not isinstance(checkpoints, list) or len(checkpoints) != 4
            or [entry['identity']['update'] for entry in checkpoints] != [8, 16, 24, 32]
            or not any(entry == dict(path=directory.relative_to(root).as_posix(), identity=identity)
                       for entry in checkpoints)):
        raise ValueError('Checkpoint identity differs from completed training inventory')


def checkpoint_identity(directory, *, root=ROOT, require_completed_run=True):
    """Verify before unpickling; only the active worker may skip run completion.

    ``False`` validates a newly written checkpoint for the active worker's final
    inventory. Inference callers keep the default and require the immutable,
    completed training package plus current sources and approval provenance.
    """
    try:
        return _checkpoint_identity(directory, root=root, require_completed_run=require_completed_run)
    except (KeyError, TypeError, AttributeError) as error:
        raise ValueError('Malformed checkpoint or training evidence') from error


def _checkpoint_identity(directory, *, root, require_completed_run):
    if type(require_completed_run) is not bool:
        raise ValueError('Completion validation switch must be boolean')
    root = Path(root).resolve()
    original = Path(directory).absolute()
    directory = original.resolve()
    if not directory.is_relative_to(root / 'outputs/lab_lane_ppo') or not directory.is_dir():
        raise ValueError('Checkpoint must be inside this project pilot outputs')
    if (original != directory or any(_link(path) for path in (directory, *directory.parents) if path.is_relative_to(root))
            or directory.parent.name != 'checkpoints'
            or directory.parent.parent.parent != root / 'outputs/lab_lane_ppo'):
        raise ValueError('Checkpoint must have the canonical nonlinked training layout')
    # Core checkpoint metadata/inventory is verified here before unpickling.
    metadata = json.loads((directory / 'checkpoint.json').read_text(encoding='utf-8'))
    if (type(metadata.get('schema_version')) is not int or metadata.get('schema_version') != 1
            or metadata.get('complete') is not True
            or metadata.get('exact_native_mid_episode_resume') is not False):
        raise ValueError('Checkpoint is not a complete post-update package')
    inventory = metadata['files']
    if (not isinstance(inventory, dict) or set(inventory) != {'model.zip', 'vecnormalize.pkl', 'rng-state.pkl', 'state.json'}
            or not all(_sha256(value) for value in inventory.values())):
        raise ValueError('Checkpoint inventory differs from the declared package')
    entries = list(directory.iterdir())
    if any(_link(path) or not path.is_file() for path in entries):
        raise ValueError('Checkpoint must contain regular files only')
    actual = {p.name for p in entries}
    if actual != set(inventory) | {'checkpoint.json'}:
        raise ValueError('Checkpoint contains missing or unexpected files')
    for name, expected in inventory.items():
        if digest(directory / name) != expected:
            raise ValueError('Checkpoint hash mismatch: ' + name)
    state = json.loads((directory / 'state.json').read_text(encoding='utf-8'))
    update, steps = state['completed_updates'], state['control_steps']
    if (type(update) is not int or update not in (8, 16, 24, 32)
            or type(steps) is not int or steps != update * 2048
            or directory.name != f'update-{update:02d}'
            or state.get('checkpoint_phase') != 'after_completed_train_update'):
        raise ValueError('Checkpoint must follow a declared completed optimizer update')
    if (state['proposal_sha256'] != PROPOSAL_SHA256
            or type(state['training_seed']) is not int or state['training_seed'] != 2101):
        raise ValueError('Checkpoint does not belong to the approved pilot')
    if type(state['optimizer_steps']) is not int or not 0 < state['optimizer_steps'] <= update * 160:
        raise ValueError('Checkpoint optimizer count is outside the approved bound')
    identity = {'update': update, 'control_steps': steps,
            'model_sha256': inventory['model.zip'],
            'normalization_sha256': inventory['vecnormalize.pkl']}
    if require_completed_run:
        _completed_training_run(directory, state, identity, root)
    return identity
