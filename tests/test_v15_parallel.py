"""v15 Phase 0: the TraCI/libsumo switch, the run comparison, and (where the frozen LA networks and
libsumo are installed) serial TraCI = serial libsumo = parallel workers = the trainer's own record."""
import gzip
import importlib.util
import json
import shutil
import uuid
from pathlib import Path

import pytest

from environments import highway_flow_rl_v13_env, i110_merge_env
from experiments import v15_parallel as par
from traffic import sumo_coupling

ROOT = Path(__file__).resolve().parents[1]
HAS_LIBSUMO = importlib.util.find_spec('libsumo') is not None
TRAINER_RECORD = ROOT / 'outputs/highway-flow-rl/v14/round-24/initial-validation'


@pytest.fixture
def restore_traci():
    yield
    sumo_coupling.use('traci')


def test_traci_is_the_default_and_rebinds_both_environment_modules(restore_traci, monkeypatch):
    import traci
    monkeypatch.delenv(sumo_coupling.ENV_VAR, raising=False)
    assert sumo_coupling.use() == 'traci' and sumo_coupling.current() == 'traci'
    assert i110_merge_env.traci is traci and highway_flow_rl_v13_env.traci is traci


def test_unknown_backends_are_refused(restore_traci):
    with pytest.raises(ValueError):
        sumo_coupling.use('socket')


@pytest.mark.skipif(not HAS_LIBSUMO, reason='libsumo is not installed in this environment')
def test_libsumo_stands_in_for_traci_with_one_simulation_per_process(restore_traci, monkeypatch):
    monkeypatch.setenv(sumo_coupling.ENV_VAR, 'libsumo')
    assert sumo_coupling.use() == 'libsumo'
    module = i110_merge_env.traci
    assert isinstance(module, sumo_coupling.LibsumoTraci) and highway_flow_rl_v13_env.traci is module
    import libsumo
    assert module.TraCIException is libsumo.TraCIException
    assert libsumo.getVersion()[1] == sumo_coupling.PINNED_VERSION
    with pytest.raises(KeyError):
        module.getConnection('default')
    module.label = 'open'
    with pytest.raises(RuntimeError):
        module.start(['sumo'])
    module.label = None


def _fake_run(path, rows, value=1.0, directory='a'):
    path.mkdir(parents=True)
    (path / 'summary.json').write_text(json.dumps(dict(rows=rows)))
    for index in rows:
        row = path / f'row-{index:02d}'
        trace = row / 'episode' / 'episode-1-x' / 'actions.jsonl.gz'
        trace.parent.mkdir(parents=True)
        (row / 'report.json').write_text(json.dumps(dict(case_id='c', overrides=1, episode_directory=directory)))
        with gzip.open(trace, 'wt') as handle:
            for step in range(3):
                agent = dict(requested_acceleration=value if step == 2 else 0.0)
                handle.write(json.dumps(dict(time=0.2 + 0.1 * step, agents=dict(agent_0=agent))) + '\n')


def test_compare_ignores_only_the_episode_directory(tmp_path):
    _fake_run(tmp_path / 'a', [0, 1], directory='outputs/a')
    _fake_run(tmp_path / 'b', [1, 2], directory='outputs/b')
    result = par.compare(tmp_path / 'a', tmp_path / 'b')
    assert result['identical'] and result['rows'] == [1] and result['decisions'] == 3


def test_compare_finds_the_first_differing_step_and_report_keys(tmp_path):
    _fake_run(tmp_path / 'a', [0])
    _fake_run(tmp_path / 'b', [0], value=1.0000001)
    report = tmp_path / 'b' / 'row-00' / 'report.json'
    report.write_text(json.dumps(dict(json.loads(report.read_text()), overrides=2)))
    result = par.compare(tmp_path / 'a', tmp_path / 'b')
    assert not result['identical']
    assert dict(row=0, where='report', keys=['overrides']) in result['differences']
    assert dict(row=0, where='trace', first_step=2, steps=(3, 3)) in result['differences']


def test_compare_with_no_shared_rows_is_not_identical(tmp_path):
    _fake_run(tmp_path / 'a', [0])
    _fake_run(tmp_path / 'b', [1])
    assert par.compare(tmp_path / 'a', tmp_path / 'b')['identical'] is False


def test_outputs_must_stay_inside_project_outputs(tmp_path):
    with pytest.raises(ValueError):
        par.run_rows([0], tmp_path / 'run')


def _networks_present():
    from traffic import highway_flow_rl_v14_contract as contract
    rows = contract.build_config(24)['validation_rows']
    return all((ROOT / rows[i]['case']['network']).is_file() for i in (8, 9))


@pytest.mark.skipif(not HAS_LIBSUMO, reason='libsumo is not installed in this environment')
@pytest.mark.timeout(600)
def test_serial_traci_serial_libsumo_and_workers_give_identical_rows():
    if not _networks_present():
        pytest.skip('the frozen LA networks are not in this checkout (outputs/ is not in Git)')
    out = ROOT / 'outputs' / 'test-runs' / f'pytest-v15-parity-{uuid.uuid4().hex[:8]}'
    try:
        rows = [8, 9]
        par.run_rows(rows, out / 'traci', backend='traci')
        par.run_rows(rows, out / 'libsumo', backend='libsumo')
        summary = par.run_rows(rows, out / 'workers', backend='libsumo', workers=2)
        assert summary['workers'] == 2 and summary['clock_seconds'] > 0                  # the ledger's clock time
        assert summary['worker_seconds'] > 0 and summary['simulated_seconds'] > 0
        for other in ('libsumo', 'workers'):
            result = par.compare(out / 'traci', out / other)
            assert result['identical'], result['differences']
            assert result['rows'] == rows and result['decisions'] > 0
        if TRAINER_RECORD.with_suffix('.json').is_file():
            assert par.compare(out / 'workers', TRAINER_RECORD)['identical']
    finally:
        shutil.rmtree(out, ignore_errors=True)
