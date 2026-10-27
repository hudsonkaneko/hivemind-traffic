"""Locate the published MAPPO v02 actor and verify its reproduction contract.

Read-only Git inspection; never downloads or deserializes model binaries and
never starts training. Refresh remote refs separately before a fresh audit.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import re
import subprocess

ROOT = Path(__file__).resolve().parents[1]
REPORT_COMMIT = 'd12c168c72c3a7441fb274af3f483de1609dac19'
EVIDENCE = 'documentation/evidence/sumo-mappo-v2-20261026.json'


def git(*args):
    return subprocess.check_output(['git','-c',f'safe.directory={ROOT.as_posix()}',*args],cwd=ROOT)


def checkpoint_identity(path, expected):
    path = Path(path)
    if not path.is_file():
        return dict(present=False, sha256=None, matches_published_actor=False)
    digest=hashlib.sha256(path.read_bytes()).hexdigest()
    return dict(present=True,sha256=digest,matches_published_actor=digest==expected)


def recover_source_bytes(blob, expected):
    """Recover a recorded Windows worktree byte hash without changing content."""
    if hashlib.sha256(blob).hexdigest()==expected:
        return blob, 'git_blob_exact'
    crlf=blob.replace(b'\r\n',b'\n').replace(b'\n',b'\r\n')
    if hashlib.sha256(crlf).hexdigest()==expected:
        return crlf, 'git_LF_to_recorded_CRLF'
    return blob, 'unresolved_content_mismatch'


def audit(output, checkpoint=None):
    output=Path(output).resolve()
    output.mkdir(parents=True,exist_ok=False)
    raw=git('show',f'{REPORT_COMMIT}:{EVIDENCE}')
    evidence=json.loads(raw)
    (output/'published-evidence.json').write_bytes(raw)
    source_commit=evidence['source_commit']
    checks={}
    for name, expected in evidence['source_sha256'].items():
        blob=git('show',f'{source_commit}:{name}')
        source,recovery=recover_source_bytes(blob,expected)
        dest=output/'source'/name
        dest.parent.mkdir(parents=True,exist_ok=True)
        dest.write_bytes(source)
        actual=hashlib.sha256(source).hexdigest()
        checks[name]=dict(expected=expected,actual=actual,match=actual==expected,
            git_blob_sha256=hashlib.sha256(blob).hexdigest(),recovery=recovery)
    config=json.loads((output/'source/experiments/configs/sumo_mappo_v2.json').read_text())
    digest=hashlib.sha256(json.dumps(config,sort_keys=True,separators=(',',':'),allow_nan=False).encode()).hexdigest()
    path=Path(checkpoint) if checkpoint is not None else ROOT/evidence['policy_path']
    identity=checkpoint_identity(path,evidence['policy_sha256'])
    objects=git('rev-list','--all','--objects').decode().splitlines()
    candidates=[row for row in objects if re.search(r'\.(pt|pth|onnx|ckpt|safetensors|zip)$',row,re.I)]
    refs=git('for-each-ref','--format=%(refname) %(objectname)','refs/remotes/origin','refs/tags').decode().splitlines()
    workflow_names=git('ls-tree','-r','--name-only',REPORT_COMMIT,'.github/workflows').decode().splitlines()
    for name in workflow_names:
        dest=output/'source'/name;dest.parent.mkdir(parents=True,exist_ok=True)
        dest.write_bytes(git('show',f'{REPORT_COMMIT}:{name}'))
    result=dict(schema_version=1,audited_utc=datetime.now(timezone.utc).isoformat(),
        report_commit=REPORT_COMMIT,training_source_commit=source_commit,published_run=evidence['run_id'],
        published_policy_path=evidence['policy_path'],checked_policy_path=str(path),
        expected_policy_sha256=evidence['policy_sha256'],checkpoint=identity,
        source_hash_checks=checks,config_digest=digest,config_digest_matches=digest==evidence['config_sha256'],
        fetched_refs=refs,history_binary_candidates=candidates,
        original_fixture_reproduction_ready=bool(identity['matches_published_actor'] and all(c['match'] for c in checks.values()) and digest==evidence['config_sha256']),
        corridor_transfer_ready=False,
        minimum_inference_bundle=['Exact policy.pt TorchScript actor with published SHA256',
            'Pinned original environment/backend and fixed normalization, configuration and seeds (present in Git)',
            'SUMO/TraCI 1.27.1 and compatible PyTorch runtime; run actor on CPU'],
        optional_for_independent_numerical_audit=['Original per-episode held-out metrics/actions and resolved runtime manifest'],
        only_needed_for_training_resume=['checkpoints/best.pt or latest.pt with model/optimizer/config/RNG state'],
        interface=dict(actor_input=27,actor_output=1,hidden_sizes=[128,128],actor_format='frozen TorchScript deterministic tanh actor',
            observation_scaling='Fixed in SumoMappoEnv._observations; no learned normalizer',
            action_formula='acceleration_m_s2 = -4.5 + (normalized_action + 1) * 3.25',
            dt_s=.1,original_lanes=3,original_route_length_m=5000,original_horizon_s=60,
            speed_mode=31,lane_change_mode=512,local_guard='speed ceiling, own-lane headway, nonnegative speed',
            information='exact simulator state; not sensor inference; no communication'),
        transfer_blockers=['Weights absent or identity unverified unless checkpoint.matches_published_actor is true',
            'lane_index/2 and lane-exists masks assume three lanes; I-110 has four/five',
            'Neighbor distances use edge-local lanePosition on one straight edge; I-110 needs route-continuous graph-aware distances',
            '5 km remaining-route fraction and 60 s elapsed fraction differ from an 815 m open corridor',
            'Fixed-lane acceleration-only actor cannot choose merges or exit lanes',
            'Local shield plus SUMO safety must be logged and compared explicitly with native ACC internals'],
        result_interpretation='Published 20/20 remains reported, not independently reproduced. No random actor or new training substituted.')
    (output/'audit.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({k:result[k] for k in ('original_fixture_reproduction_ready','corridor_transfer_ready','checkpoint','config_digest_matches')}))
    return result


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',required=True,type=Path)
    p.add_argument('--checkpoint',type=Path)
    audit(**vars(p.parse_args()))
