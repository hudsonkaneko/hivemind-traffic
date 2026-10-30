import copy
import json
import pytest
from experiments import verify_semantic_lidar_fusion as runner


@pytest.fixture
def config(tmp_path,monkeypatch):
    cfg=json.loads(runner.CONFIG.read_text())
    (tmp_path/cfg['source_vehicle']).mkdir(parents=True)
    monkeypatch.setattr(runner,'ROOT',tmp_path)
    return cfg


def test_protocol_is_bounded_static_then_moving_and_no_training(config):
    runner.validate_config(config)
    assert config['cases'][0]['mode']=='static'
    assert [c['moving_speed_m_s'] for c in config['cases']]==[0,2,10]
    assert config['fusion']['max_time_delta_s']==.05


@pytest.mark.parametrize('key,value',[('training',True),('frames',100),('max_wall_seconds_per_case',10000),('source_vehicle','../escape')])
def test_rejects_training_budget_and_external_prepared_assets(config,key,value):
    config[key]=value
    with pytest.raises(ValueError):runner.validate_config(config)


def test_never_starts_moving_before_static(config):
    config['cases']=config['cases'][1:]
    with pytest.raises(ValueError,match='Static'):runner.validate_config(config)


def test_rejects_escaping_capture_identity(config):
    config['cases'][0]['scene_id']='C:escape'
    with pytest.raises(ValueError,match='identity'):runner.validate_config(config)


@pytest.mark.parametrize('key,value',[('capture_start_s', .1), ('frame_stride_s', .08), ('frame_stride_s', .05)])
def test_rejects_unready_or_unaligned_capture_times(config,key,value):
    config[key]=value
    with pytest.raises(ValueError,match='timing'):runner.validate_config(config)


def test_prepared_asset_car_label_has_explicit_vehicle_mapping():
    from scripts.capture_semantic_lidar_fusion import canonical_classes
    assert canonical_classes(['car','vehicle']) == {'vehicle'}
    assert canonical_classes(['pedestrian','unknown']) == {'pedestrian','unknown'}


def test_capture_uses_native_clock_after_startup_physics_ticks():
    from scripts.capture_semantic_lidar_fusion import aligned_capture_schedule
    ticks, schedule=aligned_capture_schedule(2,2/120,1.2,12,8)
    assert ticks == [154,166,178,190,202,214,226,238]
    assert all((2+tick)%12==0 for tick in ticks)
    assert ticks[0]/120>=1.2
