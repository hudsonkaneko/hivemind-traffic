import json

import pytest

from traffic.evidence_chunks import CHUNK_FORMAT, EvidenceChunkWriter


def test_checkpoint_chunks_are_immutable_and_preserve_all_records(tmp_path):
    writer = EvidenceChunkWriter(tmp_path)
    rows = [{'tick': tick, 'x': float(tick)} for tick in range(1, 241)]
    planner = [{'tick': tick, 'path': f'route-{tick}'} for tick in range(0, 240, 12)]

    first = writer.write_checkpoint(rows[:120], planner[:10])
    second = writer.write_checkpoint(rows, planner)

    assert first.name == 'checkpoint-00001.json'
    assert second.name == 'checkpoint-00002.json'
    first_chunk = json.loads(first.read_text(encoding='utf-8'))
    second_chunk = json.loads(second.read_text(encoding='utf-8'))
    assert first_chunk['format'] == second_chunk['format'] == CHUNK_FORMAT
    assert (first_chunk['first_tick'], first_chunk['last_tick']) == (1, 120)
    assert (second_chunk['first_tick'], second_chunk['last_tick']) == (121, 240)
    assert first_chunk['trajectory'] + second_chunk['trajectory'] == rows
    assert first_chunk['planner'] + second_chunk['planner'] == planner
    assert writer.summary()['durable_through_tick'] == 240
    assert writer.summary()['saved_rows'] == len(rows)
    assert writer.summary()['saved_planner_records'] == len(planner)


def test_checkpoint_directory_must_be_unique(tmp_path):
    EvidenceChunkWriter(tmp_path)
    with pytest.raises(FileExistsError):
        EvidenceChunkWriter(tmp_path)


def test_publication_collision_never_overwrites_or_advances(tmp_path):
    writer = EvidenceChunkWriter(tmp_path)
    target = writer.directory / 'checkpoint-00001.json'
    target.write_text('preserve this file', encoding='utf-8')
    rows = [{'tick': tick} for tick in range(1, 121)]
    planner = [{'tick': tick} for tick in range(0, 120, 12)]

    with pytest.raises(FileExistsError):
        writer.write_checkpoint(rows, planner)

    assert target.read_text(encoding='utf-8') == 'preserve this file'
    assert writer.summary()['saved_rows'] == 0
    assert not list(writer.directory.glob('.checkpoint-*.tmp'))


def test_failed_atomic_publication_leaves_no_partial_chunk(tmp_path, monkeypatch):
    writer = EvidenceChunkWriter(tmp_path)
    rows = [{'tick': tick} for tick in range(1, 121)]
    planner = [{'tick': tick} for tick in range(0, 120, 12)]
    monkeypatch.setattr('traffic.evidence_chunks.os.link',
                        lambda *_: (_ for _ in ()).throw(OSError('fixture failure')))

    with pytest.raises(OSError, match='fixture failure'):
        writer.write_checkpoint(rows, planner)

    assert writer.summary()['saved_rows'] == 0
    assert not list(writer.directory.glob('checkpoint-*.json'))
    assert not list(writer.directory.glob('.checkpoint-*.tmp'))


def test_checkpoint_requires_contiguous_new_rows(tmp_path):
    writer = EvidenceChunkWriter(tmp_path)
    rows = [{'tick': tick} for tick in range(1, 121)]
    planner = [{'tick': 0}]
    writer.write_checkpoint(rows, planner)
    with pytest.raises(ValueError, match='contiguous'):
        writer.write_checkpoint(rows + [{'tick': 122}], planner)
